#include <stdio.h>
#include "audio_driver.h"

extern "C" void app_main(void) {
    i2s_chan_handle_t rx_handle = NULL;
    audio_driver_init(&rx_handle);

    int32_t buffer[I2S_BUFFER_SIZE];
    size_t bytes_read;

    while (1) {
        if (i2s_channel_read(rx_handle, buffer, sizeof(buffer), &bytes_read, 1000) == ESP_OK) {
            printf("Read %d bytes: %ld %ld %ld ...\n", (int)bytes_read, (long)buffer[0], (long)buffer[1], (long)buffer[2]);
        }
    }
}
