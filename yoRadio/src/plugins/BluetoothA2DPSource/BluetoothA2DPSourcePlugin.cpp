/*
  BluetoothA2DPPlugin.cpp
  Přepis pluginu pro yoRadio — thread-safe ring buffer pro A2DP,
  správná práce s počtem frame (stereo interleaved int16_t),
  non-blocking a rychlé vyplnění ticha.
*/

#include <Arduino.h>
#include "BluetoothA2DPSource.h"
#include "A2DPVolumeControl.h"
#include "../../core/player.h"
#include "../../pluginsManager/pluginsManager.h"

// -------------------------------------------------------------
//  CONFIG
// -------------------------------------------------------------
// Ring buffer velikost (v stereo samples, tedy sample = single int16_t).
// Zvoleno 4096 samples = 2048 frames (stereo frames). Velikost musí být mocnina 2.
#ifndef A2DP_RING_SAMPLES
  #define A2DP_RING_SAMPLES 4096
#endif

static_assert((A2DP_RING_SAMPLES & (A2DP_RING_SAMPLES - 1)) == 0, "A2DP_RING_SAMPLES must be power of two");

// -------------------------------------------------------------
//  GLOBALS
// -------------------------------------------------------------
static BluetoothA2DPSource btSender;

// ring buffer: interleaved stereo int16_t samples [L,R,L,R,...]
static int16_t a2dp_ring[A2DP_RING_SAMPLES];
static volatile uint32_t rb_head = 0; // write index (points to next sample to write)
static volatile uint32_t rb_tail = 0; // read index (points to next sample to read)
static const uint32_t RB_MASK = A2DP_RING_SAMPLES - 1;

// spinlock for atomic updates (ESP32 specific)
static portMUX_TYPE rb_lock = portMUX_INITIALIZER_UNLOCKED;

// silence buffer (frames) — reuse small block for fast memset-like copies
static const uint16_t SILENCE_FRAMES = 256;
static Frame silenceFrames[SILENCE_FRAMES]; // implicitly zero-initialized

// keep track of whether plugin inited
static bool bt_initialized = false;

// -------------------------------------------------------------
//  RING BUFFER HELPERS (non-blocking)
// -------------------------------------------------------------

// Return number of free samples (int16 samples) available for writing
static inline uint32_t rb_free_samples()
{
  uint32_t h = rb_head;
  uint32_t t = rb_tail;
  return (t - h - 1) & RB_MASK;
}

// Return number of available samples to read
static inline uint32_t rb_available_samples()
{
  uint32_t h = rb_head;
  uint32_t t = rb_tail;
  return (h - t) & RB_MASK;
}

// Write up to 'samples' int16 samples from src into ring buffer.
// Returns number of samples actually written (may be less if buffer nearly full).
// Non-blocking; safe to call from producer context (uses portMUX critical).
static size_t rb_write_samples(const int16_t* src, size_t samples)
{
  if (!src || samples == 0) return 0;

  size_t written = 0;
  portENTER_CRITICAL(&rb_lock);
  {
    uint32_t free_s = rb_free_samples();
    if (free_s == 0) {
      // no space
      portEXIT_CRITICAL(&rb_lock);
      return 0;
    }
    size_t to_write = (size_t)min((uint32_t)samples, free_s);

    // write possibly wrapping in two chunks
    uint32_t head = rb_head;
    uint32_t first_chunk = min<uint32_t>(to_write, A2DP_RING_SAMPLES - (head & RB_MASK));
    // copy first chunk
    memcpy(&a2dp_ring[head & RB_MASK], src, first_chunk * sizeof(int16_t));
    head = (head + first_chunk) & RB_MASK;
    written += first_chunk;

    size_t remaining = to_write - first_chunk;
    if (remaining > 0) {
      memcpy(&a2dp_ring[head & RB_MASK], src + first_chunk, remaining * sizeof(int16_t));
      head = (head + remaining) & RB_MASK;
      written += remaining;
    }

    // update head (we keep indices mod buffer size by mask)
    rb_head = (rb_head + written) & RB_MASK;
  }
  portEXIT_CRITICAL(&rb_lock);

  return written;
}

// Read up to 'samples' int16 samples into dst from ring buffer.
// Returns number of samples actually read. Non-blocking.
static size_t rb_read_samples(int16_t* dst, size_t samples)
{
  if (!dst || samples == 0) return 0;

  size_t read = 0;
  portENTER_CRITICAL(&rb_lock);
  {
    uint32_t avail = rb_available_samples();
    if (avail == 0) {
      portEXIT_CRITICAL(&rb_lock);
      return 0;
    }
    size_t to_read = (size_t)min((uint32_t)samples, avail);

    uint32_t tail = rb_tail;
    uint32_t first_chunk = min<uint32_t>(to_read, A2DP_RING_SAMPLES - (tail & RB_MASK));
    memcpy(dst, &a2dp_ring[tail & RB_MASK], first_chunk * sizeof(int16_t));
    tail = (tail + first_chunk) & RB_MASK;
    read += first_chunk;

    size_t remaining = to_read - first_chunk;
    if (remaining > 0) {
      memcpy(dst + first_chunk, &a2dp_ring[tail & RB_MASK], remaining * sizeof(int16_t));
      tail = (tail + remaining) & RB_MASK;
      read += remaining;
    }

    rb_tail = (rb_tail + read) & RB_MASK;
  }
  portEXIT_CRITICAL(&rb_lock);

  return read;
}

// Convenience: write interleaved frames (frames count). src length must be frames*2 samples.
// Returns number of frames actually written.
static size_t rb_write_frames(const int16_t* src_interleaved, size_t frames)
{
  size_t samples_to_write = frames * 2;
  size_t written_samples = rb_write_samples(src_interleaved, samples_to_write);
  return written_samples / 2;
}

// Convenience: read interleaved frames into dst (Frame*), up to 'frames'.
// Returns number of frames actually read.
static size_t rb_read_frames(Frame* dst_frames, size_t frames)
{
  if (!dst_frames || frames == 0) return 0;
  size_t samples_needed = frames * 2;
  // temporary buffer as int16_t to read raw samples
  // We'll read directly into dst_frames memory (same layout), safe because Frame is two int16_t interleaved.
  int16_t* dst_samples = (int16_t*)dst_frames;
  size_t samples_read = rb_read_samples(dst_samples, samples_needed);
  return samples_read / 2;
}

// -------------------------------------------------------------
//  A2DP DATA CALLBACK (Bluetooth lib) - must fill up to frame_count frames.
//  We'll fill with available frames from ring buffer and if not enough
//  we fill the remainder with silence (fast memcpy).
// -------------------------------------------------------------
static int32_t a2dpDataInFrames(Frame *frame, int32_t frame_count)
{
  if (!frame || frame_count <= 0) return 0;

  size_t need_frames = (size_t)frame_count;

  // read available frames
  size_t got = rb_read_frames(frame, need_frames);
  if (got < need_frames) {
    // fill remainder with silence
    size_t remaining = need_frames - got;
    // silenceFrames is frames array of zeros; copy in blocks of SILENCE_FRAMES
    Frame* dst_ptr = frame + got;
    while (remaining > 0) {
      size_t chunk = remaining > SILENCE_FRAMES ? SILENCE_FRAMES : remaining;
      memcpy(dst_ptr, silenceFrames, chunk * sizeof(Frame));
      dst_ptr += chunk;
      remaining -= chunk;
    }
  }

  // Always return the number of frames we provided (frame_count).
  // Some libraries may expect the actual frames consumed; returning full frame_count is safe.
  return frame_count;
}

// -------------------------------------------------------------
//  WEAK OVERRIDE FROM AudioEx.h (producer-side): the MP3 decoder / I2S system will call this
//  with interleaved stereo int16 samples in buff, and len = number of samples (not frames).
//  We copy as many frames as will fit into the ring buffer (non-blocking).
//  If ring is full, we drop samples (producer may decide to backpressure if supported).
// -------------------------------------------------------------
void audio_process_extern(int16_t *buff, uint16_t len, bool *continueI2S)
{
  if (!buff || len == 0) {
    if (continueI2S) *continueI2S = true;
    return;
  }

  // len is number of int16 samples (e.g., 512 samples = 256 frames)
  // write as many samples as possible
  size_t samples = (size_t)len;
  size_t written = rb_write_samples(buff, samples);

  // If not all samples written, we drop the rest (avoid blocking).
  // Option: could set continueI2S=false to pause producer if supported, but many producers don't support it.
  (void)written;

  if (continueI2S) *continueI2S = true;
}

// -------------------------------------------------------------
//  PLUGIN IMPLEMENTATION
// -------------------------------------------------------------
class BluetoothA2DPSourcePlugin : public Plugin
{
public:
    BluetoothA2DPSourcePlugin() {
        registerPlugin();  // REQUIRED for plugin manager
    }

    void initBT() {
        if (bt_initialized) return;

        // init silence buffer to zeros (should be already zeroed in bss, but be explicit)
        memset(silenceFrames, 0, sizeof(silenceFrames));

        btSender.set_auto_reconnect(false);
        btSender.set_data_callback_in_frames(a2dpDataInFrames);
        btSender.set_volume(30);
        // Start with device name; ensure it's a C-string literal (no String)
        btSender.start("LEXON MINO L");
        bt_initialized = true;
    }

    void on_setup() override {
        initBT();
    }

    void on_start_play() override {
        // clear ring buffer on station start
        portENTER_CRITICAL(&rb_lock);
        rb_head = 0;
        rb_tail = 0;
        portEXIT_CRITICAL(&rb_lock);
    }

    void on_stop_play() override {
        // optionally clear ring buffer so old audio doesn't leak to next station
        portENTER_CRITICAL(&rb_lock);
        rb_head = 0;
        rb_tail = 0;
        portEXIT_CRITICAL(&rb_lock);
    }

    void on_ticker() override {
        // every second update volume from player
        uint8_t vol = player.getVolume();
        btSender.set_volume(vol);
    }
};

// Instantiate plugin
static BluetoothA2DPSourcePlugin _BtSourceA2DPsource;
