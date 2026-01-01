#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "audio_driver.h"

extern "C" void app_main(void) {
    i2s_chan_handle_t rx_handle = NULL;
    audio_driver_init(&rx_handle);

    int32_t buffer[I2S_BUFFER_SIZE];
    size_t bytes_read;

    while (1) {
        if (i2s_channel_read(rx_handle, buffer, sizeof(buffer), &bytes_read, 1000) == ESP_OK) {
            //calculate average volume)
            int64_t sum = 0;
            int samples = bytes_read / sizeof(int32_t);
            for (int i = 0; i < samples; ++i) {
                sum += abs(buffer[i]);
            }
            double average = (double)sum / samples;
            if (average < 1.0) average = 1.0; //prevent log(0)
            double db = 20.0 * log10(average / 2147483647.0);
            printf("Volume: %.1f dB |", db);
            
            //0dB is the ceiling
            int bars = (int)(db + 90.0); 
            if (bars < 0) bars = 0;
            if (bars > 80) bars = 80;
            
            for (int i = 0; i < bars; ++i) {printf("#");}
            if (db > -70.0) {printf("  <-- VOICE DETECTED");}
            printf("\n");
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}
