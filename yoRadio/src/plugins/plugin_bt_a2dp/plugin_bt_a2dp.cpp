#include <Arduino.h>
#include "BluetoothA2DPSource.h"
#include "../../core/player.h"
//#include "../../audioI2S/AudioEx.h"
#include "../../pluginsManager/pluginsManager.h"



// -------------------------------------------------------------
//  GLOBALS
// -------------------------------------------------------------
static BluetoothA2DPSource btSender;
static int16_t *g_buf = nullptr;
static uint16_t g_len = 0;
static size_t g_pos = 0;
static int16_t silence[256*2] = {0};

int32_t a2dpDataInFrames(Frame *frame, int32_t frame_count) {
    if (!g_buf || g_len == 0) {
        // žádný audio blok → ticho
        for (int i = 0; i < frame_count; ++i) {
            frame[i].channel1 = 0;
            frame[i].channel2 = 0;
        }
        return frame_count;
    }

    // převod interleaved stereo int16_t na Frame
    int maxFrames = min((int)frame_count, (int)(g_len - g_pos));
    for (int i = 0; i < maxFrames; ++i) {
        frame[i].channel1 = g_buf[g_pos*2];
        frame[i].channel2 = g_buf[g_pos*2 + 1];
        g_pos++;
    }

    return maxFrames;
}

// -------------------------------------------------------------
//  WEAK OVERRIDE FROM AudioEx.h
// -------------------------------------------------------------
void audio_process_extern(int16_t *buff, uint16_t len, bool *continueI2S)
{
    g_buf = buff;
    g_len = len;
    g_pos = 0;

    *continueI2S = true;
}

// -------------------------------------------------------------
//  PLUGIN IMPLEMENTATION
// -------------------------------------------------------------
class BluetoothA2DPPlugin : public Plugin
{
public:
    BluetoothA2DPPlugin() {
        registerPlugin();  // REQUIRED for plugin manager
    }

    void initBT(){        
        btSender.set_auto_reconnect(false);
        btSender.set_data_callback_in_frames(a2dpDataInFrames);        
        btSender.set_volume(30);
        btSender.start("LEXON MINO L");  
    }

    void on_setup() override {
        initBT();
    }

    void on_start_play() override {
        // reset buffer pointer on station start
        g_buf = nullptr;
        g_len = 0;
        g_pos = 0;
    }

    void on_stop_play() override {
        // optional: stop sending valid data
        g_buf = nullptr;
        g_len = 0;
    }

    void on_ticker() override{
        // every second
        //uint8_t vol = player.getVolume();
        
        //debug
        uint8_t vol = 10; 

        btSender.set_volume(vol);
    }
};

// Instantiate plugin
static BluetoothA2DPPlugin _plugin_instance;
