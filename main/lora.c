#include <string.h>
#include "lora.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Pinos internos do SX1262 na Heltec V3
#define PIN_NSS     8
#define PIN_SCK     9
#define PIN_MOSI    10
#define PIN_MISO    11
#define PIN_RST     GPIO_NUM_12
#define PIN_BUSY    GPIO_NUM_13
#define PIN_DIO1    GPIO_NUM_14

// Comandos do SX1262 (datasheet, cap. 13)
#define CMD_SET_STANDBY         0x80
#define CMD_SET_TX              0x83
#define CMD_SET_RX              0x82
#define CMD_SET_PACKET_TYPE     0x8A
#define CMD_SET_RF_FREQUENCY    0x86
#define CMD_SET_TX_PARAMS       0x8E
#define CMD_SET_PA_CONFIG       0x95
#define CMD_SET_MODULATION      0x8B
#define CMD_SET_PACKET_PARAMS   0x8C
#define CMD_SET_BUFFER_BASE     0x8F
#define CMD_SET_DIO_IRQ         0x08
#define CMD_GET_IRQ_STATUS      0x12
#define CMD_CLEAR_IRQ           0x02
#define CMD_GET_RX_BUFFER       0x13
#define CMD_GET_PACKET_STATUS   0x14
#define CMD_WRITE_BUFFER        0x0E
#define CMD_READ_BUFFER         0x1E
#define CMD_WRITE_REGISTER      0x0D
#define CMD_READ_REGISTER       0x1D
#define CMD_CALIBRATE           0x89
#define CMD_CALIBRATE_IMAGE     0x98
#define CMD_SET_REGULATOR       0x96
#define CMD_SET_DIO2_RF_SWITCH  0x9D
#define CMD_SET_DIO3_TCXO       0x97
#define CMD_CLEAR_ERRORS        0x07

// Bits de interrupcao
#define IRQ_TX_DONE     (1 << 0)
#define IRQ_RX_DONE     (1 << 1)
#define IRQ_HEADER_ERR  (1 << 5)
#define IRQ_CRC_ERR     (1 << 6)
#define IRQ_TIMEOUT     (1 << 9)

#define REG_SYNC_WORD_MSB   0x0740
#define REG_TX_CLAMP        0x08D8

static spi_device_handle_t spi;
static uint8_t tx_buf[260];
static uint8_t rx_buf[260];

#define CHECK(x) do { esp_err_t e_ = (x); if (e_ != ESP_OK) return e_; } while (0)

// ---------------------------------------------------------------------------
// Acesso SPI de baixo nivel
// ---------------------------------------------------------------------------

// O SX1262 deixa BUSY em nivel alto enquanto processa um comando
static esp_err_t wait_busy(void)
{
    for (int i = 0; i < 1000; i++) {
        if (gpio_get_level(PIN_BUSY) == 0) {
            return ESP_OK;
        }
        esp_rom_delay_us(100);
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t xfer(size_t len)
{
    CHECK(wait_busy());
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };
    return spi_device_polling_transmit(spi, &t);
}

// Comando de escrita: opcode + parametros
static esp_err_t cmd(uint8_t op, const uint8_t *params, size_t n)
{
    tx_buf[0] = op;
    if (n) {
        memcpy(&tx_buf[1], params, n);
    }
    return xfer(n + 1);
}

// Comando de leitura: opcode, status, dados...
static esp_err_t cmd_read(uint8_t op, uint8_t *out, size_t n)
{
    memset(tx_buf, 0, n + 2);
    tx_buf[0] = op;
    CHECK(xfer(n + 2));
    memcpy(out, &rx_buf[2], n);
    return ESP_OK;
}

static esp_err_t write_reg(uint16_t addr, uint8_t val)
{
    const uint8_t p[] = {addr >> 8, addr & 0xFF, val};
    return cmd(CMD_WRITE_REGISTER, p, sizeof(p));
}

static esp_err_t read_reg(uint16_t addr, uint8_t *val)
{
    memset(tx_buf, 0, 5);
    tx_buf[0] = CMD_READ_REGISTER;
    tx_buf[1] = addr >> 8;
    tx_buf[2] = addr & 0xFF;
    CHECK(xfer(5));
    *val = rx_buf[4];
    return ESP_OK;
}

static esp_err_t get_irq(uint16_t *irq)
{
    uint8_t r[2];
    CHECK(cmd_read(CMD_GET_IRQ_STATUS, r, 2));
    *irq = (r[0] << 8) | r[1];
    return ESP_OK;
}

static esp_err_t clear_irq(void)
{
    const uint8_t p[] = {0xFF, 0xFF};
    return cmd(CMD_CLEAR_IRQ, p, sizeof(p));
}

static esp_err_t set_standby(void)
{
    const uint8_t p[] = {0x00};  // STDBY_RC
    return cmd(CMD_SET_STANDBY, p, sizeof(p));
}

static esp_err_t set_packet_params(uint8_t len)
{
    const uint8_t p[] = {
        LORA_PREAMBULO >> 8, LORA_PREAMBULO & 0xFF,
        0x00,   // cabecalho explicito (tamanho vai no pacote)
        len,    // tamanho do payload
        0x01,   // CRC ligado
        0x00,   // IQ normal
    };
    return cmd(CMD_SET_PACKET_PARAMS, p, sizeof(p));
}

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

esp_err_t lora_init(void)
{
    // GPIOs de controle
    gpio_reset_pin(PIN_RST);
    gpio_set_direction(PIN_RST, GPIO_MODE_OUTPUT);
    gpio_reset_pin(PIN_BUSY);
    gpio_set_direction(PIN_BUSY, GPIO_MODE_INPUT);
    gpio_reset_pin(PIN_DIO1);
    gpio_set_direction(PIN_DIO1, GPIO_MODE_INPUT);

    // Reset do radio
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Barramento SPI
    spi_bus_config_t bus = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = sizeof(tx_buf),
    };
    CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev = {
        .clock_speed_hz = 8 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NSS,
        .queue_size = 1,
    };
    CHECK(spi_bus_add_device(SPI2_HOST, &dev, &spi));

    // Confere se o chip responde: o sync word padrao comeca com 0x14
    uint8_t sync;
    CHECK(read_reg(REG_SYNC_WORD_MSB, &sync));
    if (sync != 0x14) {
        return ESP_ERR_NOT_FOUND;
    }

    CHECK(set_standby());

    // TCXO da Heltec V3 alimentado pelo DIO3 em 1,8 V (timeout 5 ms = 320 x 15,625 us)
    const uint8_t tcxo[] = {0x02, 0x00, 0x01, 0x40};
    CHECK(cmd(CMD_SET_DIO3_TCXO, tcxo, sizeof(tcxo)));

    const uint8_t calib[] = {0x7F};  // calibra todos os blocos
    CHECK(cmd(CMD_CALIBRATE, calib, sizeof(calib)));
    vTaskDelay(pdMS_TO_TICKS(10));
    const uint8_t no_err[] = {0x00, 0x00};
    CHECK(cmd(CMD_CLEAR_ERRORS, no_err, sizeof(no_err)));

    const uint8_t dcdc[] = {0x01};
    CHECK(cmd(CMD_SET_REGULATOR, dcdc, sizeof(dcdc)));

    const uint8_t rf_switch[] = {0x01};  // DIO2 controla a chave TX/RX da antena
    CHECK(cmd(CMD_SET_DIO2_RF_SWITCH, rf_switch, sizeof(rf_switch)));

    const uint8_t lora[] = {0x01};
    CHECK(cmd(CMD_SET_PACKET_TYPE, lora, sizeof(lora)));

    // Frequencia: valor = f * 2^25 / 32 MHz
    uint32_t frf = (uint32_t)(((uint64_t)LORA_FREQ_HZ << 25) / 32000000);
    const uint8_t freq[] = {frf >> 24, frf >> 16, frf >> 8, frf};
    CHECK(cmd(CMD_SET_RF_FREQUENCY, freq, sizeof(freq)));

    const uint8_t img[] = {0xE1, 0xE9};  // calibracao de imagem para 902-928 MHz
    CHECK(cmd(CMD_CALIBRATE_IMAGE, img, sizeof(img)));

    // Amplificador de potencia do SX1262 (ate +22 dBm)
    const uint8_t pa[] = {0x04, 0x07, 0x00, 0x01};
    CHECK(cmd(CMD_SET_PA_CONFIG, pa, sizeof(pa)));
    const uint8_t txp[] = {(uint8_t)(int8_t)LORA_TX_DBM, 0x04};  // rampa de 200 us
    CHECK(cmd(CMD_SET_TX_PARAMS, txp, sizeof(txp)));

    // Errata 15.2 do SX1262: melhora a tolerancia a descasamento da antena
    uint8_t clamp;
    CHECK(read_reg(REG_TX_CLAMP, &clamp));
    CHECK(write_reg(REG_TX_CLAMP, clamp | 0x1E));

    const uint8_t base[] = {0x00, 0x00};  // buffer de TX e RX a partir do endereco 0
    CHECK(cmd(CMD_SET_BUFFER_BASE, base, sizeof(base)));

    // Low data rate optimize: obrigatorio quando o simbolo dura mais de 16 ms
    const uint8_t ldro = (LORA_SF >= 11 && LORA_BW == 0x04) ? 1 : 0;
    const uint8_t mod[] = {LORA_SF, LORA_BW, LORA_CR, ldro};
    CHECK(cmd(CMD_SET_MODULATION, mod, sizeof(mod)));

    CHECK(set_packet_params(255));

    // Interrupcoes usadas, todas sinalizadas no DIO1
    const uint16_t mask = IRQ_TX_DONE | IRQ_RX_DONE | IRQ_HEADER_ERR | IRQ_CRC_ERR | IRQ_TIMEOUT;
    const uint8_t irq[] = {mask >> 8, mask & 0xFF, mask >> 8, mask & 0xFF, 0, 0, 0, 0};
    CHECK(cmd(CMD_SET_DIO_IRQ, irq, sizeof(irq)));

    return clear_irq();
}

esp_err_t lora_send(const uint8_t *data, size_t len)
{
    if (len == 0 || len > 255) {
        return ESP_ERR_INVALID_SIZE;
    }

    CHECK(set_standby());

    // Copia o pacote para o buffer interno do radio
    tx_buf[0] = CMD_WRITE_BUFFER;
    tx_buf[1] = 0x00;  // offset
    memcpy(&tx_buf[2], data, len);
    CHECK(xfer(len + 2));

    CHECK(set_packet_params(len));
    CHECK(clear_irq());

    const uint8_t sem_timeout[] = {0x00, 0x00, 0x00};
    CHECK(cmd(CMD_SET_TX, sem_timeout, sizeof(sem_timeout)));

    // Espera o TxDone (ate 3 s)
    for (int i = 0; i < 300; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        uint16_t irq;
        CHECK(get_irq(&irq));
        if (irq & IRQ_TX_DONE) {
            return clear_irq();
        }
    }
    set_standby();
    return ESP_ERR_TIMEOUT;
}

esp_err_t lora_start_rx(void)
{
    CHECK(set_standby());
    CHECK(set_packet_params(255));
    CHECK(clear_irq());
    const uint8_t continuo[] = {0xFF, 0xFF, 0xFF};  // recepcao continua
    return cmd(CMD_SET_RX, continuo, sizeof(continuo));
}

int lora_receive(uint8_t *buf, size_t max, int *rssi, float *snr, uint32_t timeout_ms)
{
    uint16_t irq = 0;
    for (uint32_t t = 0; t < timeout_ms; t += 10) {
        if (get_irq(&irq) != ESP_OK) {
            return -1;
        }
        if (irq & (IRQ_RX_DONE | IRQ_CRC_ERR | IRQ_HEADER_ERR)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!(irq & (IRQ_RX_DONE | IRQ_CRC_ERR | IRQ_HEADER_ERR))) {
        return 0;
    }
    if (irq & (IRQ_CRC_ERR | IRQ_HEADER_ERR)) {
        clear_irq();
        return -1;
    }

    // Tamanho e posicao do pacote no buffer do radio
    uint8_t st[2];
    if (cmd_read(CMD_GET_RX_BUFFER, st, 2) != ESP_OK) {
        return -1;
    }
    size_t len = st[0] < max ? st[0] : max;

    memset(tx_buf, 0, len + 3);
    tx_buf[0] = CMD_READ_BUFFER;
    tx_buf[1] = st[1];  // offset
    if (xfer(len + 3) != ESP_OK) {
        return -1;
    }
    memcpy(buf, &rx_buf[3], len);

    // Qualidade do sinal
    uint8_t ps[3];
    if (cmd_read(CMD_GET_PACKET_STATUS, ps, 3) == ESP_OK) {
        *rssi = -ps[0] / 2;
        *snr = (int8_t)ps[1] / 4.0f;
    }

    clear_irq();
    return (int)len;
}
