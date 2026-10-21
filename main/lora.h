33433434#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// Radio SX1262 da Heltec WiFi LoRa 32 V3 (LoRa ponto a ponto, sem LoRaWAN).
// Transmissor e receptor precisam usar os MESMOS parametros abaixo.

#define LORA_FREQ_HZ    915000000  // Brasil (Anatel): 902-907,5 e 915-928 MHz
#define LORA_SF         9          // spreading factor 7..12 (maior = mais alcance, mais lento)
#define LORA_BW         0x04       // largura de banda: 0x04 = 125 kHz, 0x05 = 250 kHz, 0x06 = 500 kHz
#define LORA_CR         0x01       // coding rate: 0x01 = 4/5 ... 0x04 = 4/8
#define LORA_TX_DBM     14         // potencia de transmissao: -9..22 dBm
#define LORA_PREAMBULO  8          // simbolos de preambulo

esp_err_t lora_init(void);

// Transmite e espera o fim do envio (bloqueia por ~200 ms com SF9)
esp_err_t lora_send(const uint8_t *data, size_t len);

// Coloca o radio em recepcao continua
esp_err_t lora_start_rx(void);

// Espera um pacote por ate timeout_ms.
// Retorna o tamanho recebido, 0 se nada chegou ou -1 se o pacote veio corrompido.
int lora_receive(uint8_t *buf, size_t max, int *rssi, float *snr, uint32_t timeout_ms);
