#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1306.h"
#include "audio_driver.h"
#include "font8x8_basic.h"

#define SDA 5
#define SCL 6
#define W 72
#define H 40
#define THRESH -70.0

esp_lcd_panel_handle_t panel;
uint8_t buf[W * (H / 8)];

void draw(const char *txt, int x, int pg) {
    for (int i = 0; txt[i]; i++, x += 8) {
        if (x + 8 > W) break;
        int idx = txt[i] - 32;
        if (idx >= 0 && idx < 96)
            memcpy(&buf[pg * W + x], font8x8_basic[idx], 8);
    }
}

void init_display() {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = (gpio_num_t)SDA,
        .scl_io_num = (gpio_num_t)SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {.enable_internal_pullup = 1}
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    esp_lcd_panel_io_i2c_config_t io_cfg = {
        .dev_addr = 0x3C,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .scl_speed_hz = 400000
    };
    esp_lcd_panel_io_handle_t io;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(bus, &io_cfg, &io));

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = -1,
        .bits_per_pixel = 1
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io, &panel_cfg, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
    
    memset(buf, 0, sizeof(buf));
}

extern "C" void app_main(void) {
    init_display();
    i2s_chan_handle_t rx;
    audio_driver_init(&rx);

    int32_t data[I2S_BUFFER_SIZE];
    size_t read;
    bool det = false;
    
    draw("Listening", 0, 2);
    esp_lcd_panel_draw_bitmap(panel, 0, 0, W, H, buf);

    while (1) {
        if (i2s_channel_read(rx, data, sizeof(data), &read, 1000) == ESP_OK) {
            int64_t sum = 0;
            int n = read / sizeof(int32_t);
            for (int i = 0; i < n; i++) sum += abs(data[i]);
            
            double avg = (double)sum / n;
            if (avg < 1.0) avg = 1.0;
            double db = 20.0 * log10(avg / 2147483647.0);
            
            bool now = db > THRESH;
            if (now != det) {
                memset(buf, 0, sizeof(buf));
                if (now) {
                    draw("VOICE", 16, 1);
                    draw("DETECTED", 4, 3);
                } else {
                    draw("Listening", 0, 2);
                }
                esp_lcd_panel_draw_bitmap(panel, 0, 0, W, H, buf);
                det = now;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}