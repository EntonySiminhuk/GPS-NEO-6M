#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_timer.h"

#define MODO_RECEPTOR 0   // 0 = transmissor (placa que voa), 1 = receptor (solo)

/* ---------- Pinos ---------- */
#define SDA_PIN   41
#define SCL_PIN   42
#define MPU_ADDR  0x68

#define LORA_NSS   8
#define LORA_SCK   9
#define LORA_MOSI  10
#define LORA_MISO  11
#define LORA_RST   12
#define LORA_BUSY  13
#define LORA_DIO1  14

/* ---------- Parâmetros de rádio (TX e RX devem ser iguais) ---------- */
#define LORA_FREQ_HZ   915000000UL
#define LORA_SF        7
#define LORA_BW_CODE   0x04      // 0x04 = 125 kHz, 0x05 = 250 kHz
#define LORA_CR_CODE   0x01      // 4/5
#define LORA_TX_DBM    17        // confira os limites da ANATEL

/* ---------- Pacote ---------- */
typedef struct __attribute__((packed)) {
    uint16_t seq;
    uint32_t t_ms;
    int16_t  roll_cd;       // centésimos de grau
    int16_t  pitch_cd;
    int16_t  gyro_z_dps10;  // décimos de °/s
} pacote_t;

/* ================= Driver SX1262 ================= */
static spi_device_handle_t lora;

static void wait_busy(void) {
    int n = 100000;
    while (gpio_get_level(LORA_BUSY) && --n) { }
}

static void sx_xfer(const uint8_t *tx, uint8_t *rx, size_t len) {
    wait_busy();
    spi_transaction_t t = { .length = len * 8, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_polling_transmit(lora, &t);
}

static void sx_cmd(uint8_t cmd, const uint8_t *data, size_t n) {
    uint8_t buf[64];
    buf[0] = cmd;
    if (n) memcpy(&buf[1], data, n);
    sx_xfer(buf, NULL, n + 1);
}

static void sx_write_reg(uint16_t addr, uint8_t v0, uint8_t v1) {
    uint8_t d[4] = { addr >> 8, addr & 0xFF, v0, v1 };
    sx_cmd(0x0D, d, 4);
}

static uint16_t sx_irq_status(void) {
    uint8_t tx[4] = { 0x12, 0, 0, 0 }, rx[4];
    sx_xfer(tx, rx, 4);
    return (rx[2] << 8) | rx[3];
}

static void sx_clear_irq(void) {
    uint8_t d[2] = { 0xFF, 0xFF };
    sx_cmd(0x02, d, 2);
}

static void sx_set_packet_params(uint8_t len) {
    uint8_t d[6] = { 0x00, 0x08, 0x00, len, 0x01, 0x00 };  // preâmbulo 8, header explícito, CRC on, IQ normal
    sx_cmd(0x8C, d, 6);
}

static void lora_init(void) {
    gpio_config_t out = { .pin_bit_mask = 1ULL << LORA_RST, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&out);
    gpio_config_t in = { .pin_bit_mask = (1ULL << LORA_BUSY) | (1ULL << LORA_DIO1),
                         .mode = GPIO_MODE_INPUT };
    gpio_config(&in);

    spi_bus_config_t bus = { .mosi_io_num = LORA_MOSI, .miso_io_num = LORA_MISO,
                             .sclk_io_num = LORA_SCK, .quadwp_io_num = -1,
                             .quadhd_io_num = -1, .max_transfer_sz = 64 };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
    spi_device_interface_config_t dev = { .clock_speed_hz = 8000000, .mode = 0,
                                          .spics_io_num = LORA_NSS, .queue_size = 1 };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &lora));

    // Reset
    gpio_set_level(LORA_RST, 0); vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_RST, 1); vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t d[8];
    d[0] = 0x00; sx_cmd(0x80, d, 1);                       // Standby RC
    d[0] = 0x02; d[1] = 0x00; d[2] = 0x01; d[3] = 0x40;    // TCXO 1,8 V no DIO3 (Heltec V3)
    sx_cmd(0x97, d, 4);
    d[0] = 0x7F; sx_cmd(0x89, d, 1);                       // Calibrate tudo
    vTaskDelay(pdMS_TO_TICKS(10));
    d[0] = 0x01; sx_cmd(0x9D, d, 1);                       // DIO2 controla a chave de RF
    d[0] = 0x01; sx_cmd(0x96, d, 1);                       // Regulador DC-DC
    d[0] = 0x01; sx_cmd(0x8A, d, 1);                       // Pacote tipo LoRa

    d[0] = 0xE1; d[1] = 0xE9; sx_cmd(0x98, d, 2);          // Calibra imagem 902-928 MHz
    uint32_t frf = ((uint64_t)LORA_FREQ_HZ << 25) / 32000000ULL;
    d[0] = frf >> 24; d[1] = frf >> 16; d[2] = frf >> 8; d[3] = frf;
    sx_cmd(0x86, d, 4);                                    // Frequência

    d[0] = 0x04; d[1] = 0x07; d[2] = 0x00; d[3] = 0x01;
    sx_cmd(0x95, d, 4);                                    // Config do PA (SX1262)
    d[0] = LORA_TX_DBM; d[1] = 0x04; sx_cmd(0x8E, d, 2);   // Potência e rampa

    d[0] = 0x00; d[1] = 0x00; sx_cmd(0x8F, d, 2);          // Buffer base TX/RX = 0
    d[0] = LORA_SF; d[1] = LORA_BW_CODE; d[2] = LORA_CR_CODE; d[3] = 0x00;
    sx_cmd(0x8B, d, 4);                                    // Modulação
    sx_set_packet_params(sizeof(pacote_t));
    sx_write_reg(0x0740, 0x14, 0x24);                      // Sync word (rede privada)

    // Interrupções no DIO1: TxDone | RxDone | CrcErr | Timeout
    uint8_t irq[8] = { 0x02, 0x43, 0x02, 0x43, 0, 0, 0, 0 };
    sx_cmd(0x08, irq, 8);
    sx_clear_irq();
}

static void lora_start_tx(const void *data, uint8_t len) {
    uint8_t buf[64];
    buf[0] = 0x00;                                         // offset
    memcpy(&buf[1], data, len);
    sx_cmd(0x0E, buf, len + 1);                            // WriteBuffer
    sx_set_packet_params(len);
    uint8_t t[3] = { 0, 0, 0 };
    sx_cmd(0x83, t, 3);                                    // SetTx (sem timeout)
}

static void lora_start_rx(void) {
    uint8_t t[3] = { 0xFF, 0xFF, 0xFF };                   // RX contínuo
    sx_cmd(0x82, t, 3);
}

/* ================= MPU-9250 ================= */
static i2c_master_dev_handle_t mpu;

static esp_err_t mpu_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(mpu, buf, 2, 100);
}

static esp_err_t mpu_read(uint8_t reg, uint8_t *dst, size_t len) {
    return i2c_master_transmit_receive(mpu, &reg, 1, dst, len, 100);
}

static esp_err_t mpu_read_all(float *ax, float *ay, float *az,
                              float *gx, float *gy, float *gz) {
    uint8_t d[14];
    esp_err_t err = mpu_read(0x3B, d, 14);
    if (err != ESP_OK) return err;
    #define S16(i) ((int16_t)((d[i] << 8) | d[i + 1]))
    *ax = S16(0) / 16384.0f;  *ay = S16(2) / 16384.0f;  *az = S16(4) / 16384.0f;
    *gx = S16(8) / 131.0f;    *gy = S16(10) / 131.0f;   *gz = S16(12) / 131.0f;
    return ESP_OK;
}

/* ================= Transmissor ================= */
static void app_transmissor(void) {
    i2c_master_bus_handle_t bus;
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT, .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA_PIN, .scl_io_num = SCL_PIN,
        .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = MPU_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &mpu));

    uint8_t id = 0;
    if (mpu_read(0x75, &id, 1) != ESP_OK) {
        printf("ERRO: MPU nao respondeu. Confira a fiacao.\n");
        return;
    }
    printf("WHO_AM_I = 0x%02X\n", id);
    mpu_write(0x6B, 0x01);                 // acorda o chip
    vTaskDelay(pdMS_TO_TICKS(100));
    mpu_write(0x1B, 0x00);                 // giroscópio ±250 °/s
    mpu_write(0x1C, 0x00);                 // acelerômetro ±2 g

    lora_init();
    printf("LoRa iniciado\n");

    // Calibração do giroscópio (sensor parado)
    float ox = 0, oy = 0, oz = 0;
    const int N = 200;
    for (int i = 0; i < N; i++) {
        float ax, ay, az, gx, gy, gz;
        mpu_read_all(&ax, &ay, &az, &gx, &gy, &gz);
        ox += gx; oy += gy; oz += gz;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    ox /= N; oy /= N; oz /= N;

    float roll = 0, pitch = 0, gz_c = 0;
    const float ALPHA = 0.98f;
    int64_t t_ant = esp_timer_get_time();
    int64_t t_tx = 0;
    bool tx_busy = false;
    uint16_t seq = 0;

    while (1) {
        float ax, ay, az, gx, gy, gz;
        if (mpu_read_all(&ax, &ay, &az, &gx, &gy, &gz) == ESP_OK) {
            int64_t t = esp_timer_get_time();
            float dt = (t - t_ant) / 1e6f;
            t_ant = t;

            float roll_acc  = atan2f(ay, az) * 57.29578f;
            float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;
            roll  = ALPHA * (roll  + (gx - ox) * dt) + (1 - ALPHA) * roll_acc;
            pitch = ALPHA * (pitch + (gy - oy) * dt) + (1 - ALPHA) * pitch_acc;
            gz_c = gz - oz;
        }

        // Terminou a transmissão anterior?
        if (tx_busy && gpio_get_level(LORA_DIO1)) {
            sx_clear_irq();
            tx_busy = false;
        }

        // Envia 2 pacotes por segundo
        int64_t agora = esp_timer_get_time();
        if (!tx_busy && (agora - t_tx) >= 500000) {
            t_tx = agora;
            pacote_t p = {
                .seq = seq++,
                .t_ms = (uint32_t)(agora / 1000),
                .roll_cd = (int16_t)(roll * 100.0f),
                .pitch_cd = (int16_t)(pitch * 100.0f),
                .gyro_z_dps10 = (int16_t)(gz_c * 10.0f),
            };
            lora_start_tx(&p, sizeof(p));
            tx_busy = true;
            printf("TX seq=%u roll=%.2f pitch=%.2f\n", p.seq, roll, pitch);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ================= Receptor ================= */
static void app_receptor(void) {
    lora_init();
    lora_start_rx();
    printf("Receptor aguardando pacotes...\n");
    uint16_t seq_ant = 0;
    bool primeiro = true;

    while (1) {
        if (gpio_get_level(LORA_DIO1)) {
            uint16_t irq = sx_irq_status();
            sx_clear_irq();
            if (irq & 0x0002) {                              // RxDone
                if (irq & 0x0040) {
                    printf("Pacote com CRC invalido\n");
                } else {
                    uint8_t tx[4] = { 0x13, 0, 0, 0 }, rx[4];
                    sx_xfer(tx, rx, 4);                      // GetRxBufferStatus
                    uint8_t len = rx[2], off = rx[3];

                    uint8_t tb[3 + 32] = { 0x1E, off }, rb[3 + 32];
                    if (len > 32) len = 32;
                    sx_xfer(tb, rb, 3 + len);                // ReadBuffer

                    uint8_t ps[5] = { 0x14 }, pr[5];
                    sx_xfer(ps, pr, 5);                      // GetPacketStatus
                    float rssi = -pr[2] / 2.0f;
                    float snr = ((int8_t)pr[3]) / 4.0f;

                    if (len == sizeof(pacote_t)) {
                        pacote_t p;
                        memcpy(&p, &rb[3], sizeof(p));
                        int perdidos = primeiro ? 0 : (uint16_t)(p.seq - seq_ant - 1);
                        seq_ant = p.seq; primeiro = false;
                        printf("seq=%u t=%lu ms roll=%.2f pitch=%.2f gz=%.1f | RSSI=%.0f dBm SNR=%.1f dB perdidos=%d\n",
                               p.seq, (unsigned long)p.t_ms, p.roll_cd / 100.0f,
                               p.pitch_cd / 100.0f, p.gyro_z_dps10 / 10.0f,
                               rssi, snr, perdidos);
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void app_main(void)
{
#if MODO_RECEPTOR
    app_receptor();
#else
    app_transmissor();
#endif
}