#pragma once

#include "esp_err.h"

// Display OLED SSD1306 128x64 da Heltec WiFi LoRa 32 V3
// Texto em fonte 5x7: 21 colunas x 8 linhas

esp_err_t oled_init(void);
void oled_clear(void);
void oled_text(int col, int row, const char *str);   // col 0..20, row 0..7
void oled_printf(int col, int row, const char *fmt, ...);
void oled_hline(int row);                            // linha horizontal no meio da linha de texto
esp_err_t oled_update(void);                         // envia o buffer para o display
