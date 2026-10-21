#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "oled.h"
#include "lora.h"

static const char *TAG = "GPS";
static const char *TAG_LORA = "LORA";

// 0 = TRANSMISSOR (placa com GPS: le a posicao e envia pelo LoRa)
// 1 = RECEPTOR    (segunda placa: recebe os pacotes e mostra na tela/terminal)
#define MODO_RECEPTOR       0

// Intervalo entre transmissoes LoRa
#define LORA_INTERVALO_MS   5000

#define GPS_UART    UART_NUM_1
#define GPS_RX_PIN  4    // ligado ao TX do GPS
#define GPS_TX_PIN  5    // ligado ao RX do GPS (necessario para configurar o GPS)
#define GPS_BAUD_PADRAO 9600   // velocidade de fabrica do NEO-6M
#define GPS_BAUD    38400      // velocidade usada depois da configuracao
#define GPS_TAXA_HZ 1          // posicoes por segundo (NEO-6M: 1 a 5)
#define BUF_SIZE    1024

// Heltec WiFi LoRa 32 V3: Vext (GPIO36) alimenta o pino 3V3 "Vext" e o OLED.
// Nivel LOW = ligado. Necessario se o GPS estiver alimentado pelo Vext.
#define VEXT_PIN    GPIO_NUM_36

// Fuso horario em horas em relacao ao UTC (Brasilia = -3)
#define FUSO_HORARIO   (-3)

// 1 = mostra tambem as sentencas NMEA brutas (para depuracao)
#define MOSTRAR_NMEA   0

typedef struct {
    bool   valido;      // true quando o GPS tem posicao (fix)
    double lat;         // graus decimais (sul negativo)
    double lon;         // graus decimais (oeste negativo)
    float  alt;         // altitude em metros (nivel do mar)
    int    sats;        // satelites usados no calculo
    time_t timestamp;   // Unix time (segundos desde 1970, UTC)
    int    ms;          // milissegundos dentro do segundo (0, 200, 400...)
} gps_data_t;

static gps_data_t gps;          // usado so pela gps_task, enquanto monta os dados
static gps_data_t gps_ultimo;   // copia completa mais recente, lida por outras tasks
static portMUX_TYPE gps_lock = portMUX_INITIALIZER_UNLOCKED;

// Pacote enviado pelo LoRa: 21 bytes em binario (em texto seriam ~70)
typedef struct __attribute__((packed)) {
    uint8_t  versao;     // formato do pacote (PACOTE_VERSAO)
    uint8_t  flags;      // bit 0: posicao valida
    uint16_t contador;   // numero sequencial, para detectar pacotes perdidos
    uint32_t timestamp;  // Unix time UTC
    int32_t  lat;        // graus x 10^7
    int32_t  lon;        // graus x 10^7
    int32_t  alt;        // decimetros
    uint8_t  sats;       // satelites usados
} pacote_t;

_Static_assert(sizeof(pacote_t) == 21, "pacote_t deve ter 21 bytes");

#define PACOTE_VERSAO 1

static bool oled_ok;
static int lora_ultimo_enviado = -1;  // contador do ultimo pacote enviado (-1 = nenhum)
static bool lora_erro;

// ---------------------------------------------------------------------------
// FORMATO DE SAIDA: altere esta funcao para mudar o que aparece no
// terminal e no display (21 colunas x 8 linhas)
// ---------------------------------------------------------------------------
static void gps_print(const gps_data_t *g)
{
    // Converte o timestamp UTC para o horario local
    time_t local = g->timestamp + FUSO_HORARIO * 3600;
    struct tm t;
    gmtime_r(&local, &t);

    if (oled_ok) {
        oled_clear();
        if (g->timestamp) {
            oled_printf(0, 0, "%02d/%02d/%04d %02d:%02d:%02d.%d",
                        t.tm_mday, t.tm_mon + 1, t.tm_year + 1900,
                        t.tm_hour, t.tm_min, t.tm_sec, g->ms / 100);
        } else {
            oled_text(0, 0, "GPS NEO-6M");
        }
        oled_hline(1);
        if (g->valido) {
            oled_printf(0, 2, "Lat: %.6f", g->lat);
            oled_printf(0, 3, "Lon: %.6f", g->lon);
            oled_printf(0, 4, "Alt: %.1f m", g->alt);
        } else {
            oled_text(0, 3, "Aguardando posicao...");
        }
        oled_printf(0, 6, "Satelites: %d", g->sats);
        if (lora_erro) {
            oled_text(0, 7, "LoRa: erro");
        } else if (lora_ultimo_enviado >= 0) {
            oled_printf(0, 7, "LoRa: #%d enviado", lora_ultimo_enviado);
        }
        oled_update();
    }

    if (!g->valido) {
        ESP_LOGW(TAG, "Aguardando posicao... (satelites: %d)", g->sats);
        return;
    }

    ESP_LOGI(TAG, "%02d/%02d/%04d %02d:%02d:%02d.%d | Lat: %.6f | Lon: %.6f | Alt: %.1f m",
             t.tm_mday, t.tm_mon + 1, t.tm_year + 1900,
             t.tm_hour, t.tm_min, t.tm_sec, g->ms / 100,
             g->lat, g->lon, g->alt);
}

// ---------------------------------------------------------------------------
// Parser NMEA
// ---------------------------------------------------------------------------

// Confere o checksum "*XX" (XOR de tudo entre '$' e '*')
static bool nmea_checksum_ok(const char *s)
{
    if (*s != '$') {
        return false;
    }
    uint8_t sum = 0;
    for (s++; *s && *s != '*'; s++) {
        sum ^= (uint8_t)*s;
    }
    if (*s != '*') {
        return false;
    }
    return sum == (uint8_t)strtol(s + 1, NULL, 16);
}

// Divide a sentenca nos campos separados por ',' (mantem campos vazios)
static int nmea_split(char *s, char *campos[], int max)
{
    int n = 0;
    campos[n++] = s;
    for (; *s && n < max; s++) {
        if (*s == ',' || *s == '*') {
            *s = '\0';
            campos[n++] = s + 1;
        }
    }
    return n;
}

// "ddmm.mmmm" + hemisferio -> graus decimais
static double nmea_coord(const char *valor, const char *hemi)
{
    double v = atof(valor);
    int graus = (int)(v / 100);
    double d = graus + (v - graus * 100) / 60.0;
    return (hemi[0] == 'S' || hemi[0] == 'W') ? -d : d;
}

// hhmmss.ss + ddmmyy -> Unix time (UTC)
static time_t nmea_time(const char *hora, const char *data)
{
    if (strlen(hora) < 6 || strlen(data) < 6) {
        return 0;
    }
    struct tm t = {0};
    t.tm_hour = (hora[0] - '0') * 10 + (hora[1] - '0');
    t.tm_min  = (hora[2] - '0') * 10 + (hora[3] - '0');
    t.tm_sec  = (hora[4] - '0') * 10 + (hora[5] - '0');
    t.tm_mday = (data[0] - '0') * 10 + (data[1] - '0');
    t.tm_mon  = (data[2] - '0') * 10 + (data[3] - '0') - 1;
    t.tm_year = (data[4] - '0') * 10 + (data[5] - '0') + 100;  // 20yy
    return mktime(&t);  // TZ do ESP32 e UTC por padrao
}

static void nmea_parse(char *linha)
{
    if (!nmea_checksum_ok(linha) || strlen(linha) < 6) {
        return;
    }

    char *f[20];
    int n = nmea_split(linha, f, 20);
    const char *tipo = f[0] + 3;  // ignora "$GP"/"$GN"

    // $GPRMC,hora,status,lat,N/S,lon,E/W,vel,curso,data,...
    if (strcmp(tipo, "RMC") == 0 && n >= 10) {
        gps.valido = (f[2][0] == 'A');
        if (gps.valido) {
            gps.lat = nmea_coord(f[3], f[4]);
            gps.lon = nmea_coord(f[5], f[6]);
        }
        gps.timestamp = nmea_time(f[1], f[9]);
        // fracao do segundo: "183535.20" -> 200 ms
        gps.ms = (strlen(f[1]) > 6) ? (int)(atof(f[1] + 6) * 1000 + 0.5) : 0;
    }
    // $GPGGA,hora,lat,N/S,lon,E/W,qualidade,sats,hdop,alt,M,...
    // GGA chega depois do RMC em cada ciclo: imprime aqui
    else if (strcmp(tipo, "GGA") == 0 && n >= 10) {
        gps.sats = atoi(f[7]);
        gps.alt  = atof(f[9]);

        // Publica a leitura completa para a lora_tx_task
        taskENTER_CRITICAL(&gps_lock);
        gps_ultimo = gps;
        taskEXIT_CRITICAL(&gps_lock);

        gps_print(&gps);
    }
}

// ---------------------------------------------------------------------------
// Configuracao do GPS (protocolo binario UBX da u-blox)
// ---------------------------------------------------------------------------

// Pacote UBX: B5 62 | classe | id | tamanho (2 bytes, LE) | payload | CK_A CK_B
static void ubx_send(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    uint8_t hdr[6] = {0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
    uint8_t ck[2] = {0, 0};

    // Checksum Fletcher de 8 bits, da classe ate o fim do payload
    for (int i = 2; i < 6; i++) {
        ck[0] += hdr[i];
        ck[1] += ck[0];
    }
    for (int i = 0; i < len; i++) {
        ck[0] += payload[i];
        ck[1] += ck[0];
    }

    uart_write_bytes(GPS_UART, hdr, sizeof(hdr));
    uart_write_bytes(GPS_UART, payload, len);
    uart_write_bytes(GPS_UART, ck, sizeof(ck));
    uart_wait_tx_done(GPS_UART, pdMS_TO_TICKS(100));
}

// Espera o UBX-ACK-ACK (B5 62 05 01 02 00 cls id) do comando enviado
static bool ubx_wait_ack(uint8_t cls, uint8_t id)
{
    const uint8_t ack[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, cls, id};
    size_t match = 0;
    TickType_t inicio = xTaskGetTickCount();
    uint8_t b;

    while (xTaskGetTickCount() - inicio < pdMS_TO_TICKS(500)) {
        if (uart_read_bytes(GPS_UART, &b, 1, pdMS_TO_TICKS(10)) != 1) {
            continue;
        }
        if (b == ack[match]) {
            if (++match == sizeof(ack)) {
                return true;
            }
        } else {
            match = (b == ack[0]) ? 1 : 0;
        }
    }
    return false;
}

static bool ubx_cmd(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    ubx_send(cls, id, payload, len);
    return ubx_wait_ack(cls, id);
}

// Muda a velocidade, desliga sentencas nao usadas e ajusta a taxa de
// atualizacao. A configuracao fica na RAM do GPS: ao perder a alimentacao
// ele volta ao padrao (9600 baud, 1 Hz), por isso e refeita a cada boot.
static bool gps_config(void)
{
    // UBX-CFG-PRT: UART1 do GPS em GPS_BAUD, 8N1, entrada UBX+NMEA, saida UBX+NMEA
    const uint8_t cfg_prt[20] = {
        0x01, 0x00, 0x00, 0x00,                          // porta 1, reservado, txReady
        0xD0, 0x08, 0x00, 0x00,                          // modo 8N1
        (uint8_t)(GPS_BAUD & 0xFF), (uint8_t)((GPS_BAUD >> 8) & 0xFF),
        (uint8_t)((GPS_BAUD >> 16) & 0xFF), (uint8_t)(GPS_BAUD >> 24),
        0x03, 0x00,                                      // protocolos de entrada
        0x03, 0x00,                                      // protocolos de saida
        0x00, 0x00, 0x00, 0x00,                          // flags, reservado
    };

    // O GPS pode estar em 9600 (acabou de ligar) ou ja em GPS_BAUD (so o
    // ESP32 reiniciou): envia o comando nas duas velocidades.
    const uint32_t bauds[] = {GPS_BAUD_PADRAO, GPS_BAUD};
    for (size_t i = 0; i < sizeof(bauds) / sizeof(bauds[0]); i++) {
        uart_set_baudrate(GPS_UART, bauds[i]);
        ubx_send(0x06, 0x00, cfg_prt, sizeof(cfg_prt));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    uart_set_baudrate(GPS_UART, GPS_BAUD);
    uart_flush_input(GPS_UART);

    // UBX-CFG-MSG: desliga GLL, GSA, GSV e VTG (so RMC e GGA sao usadas)
    const uint8_t desligar[] = {0x01, 0x02, 0x03, 0x05};
    for (size_t i = 0; i < sizeof(desligar); i++) {
        const uint8_t cfg_msg[3] = {0xF0, desligar[i], 0x00};  // classe NMEA, id, taxa 0
        if (!ubx_cmd(0x06, 0x01, cfg_msg, sizeof(cfg_msg))) {
            return false;
        }
    }

    // UBX-CFG-RATE: intervalo entre medicoes em ms, 1 navegacao por medicao, hora GPS
    const uint16_t periodo = 1000 / GPS_TAXA_HZ;
    const uint8_t cfg_rate[6] = {
        (uint8_t)(periodo & 0xFF), (uint8_t)(periodo >> 8),
        0x01, 0x00,
        0x01, 0x00,
    };
    return ubx_cmd(0x06, 0x08, cfg_rate, sizeof(cfg_rate));
}

// ---------------------------------------------------------------------------

static void vext_on(void)
{
    gpio_reset_pin(VEXT_PIN);
    gpio_set_direction(VEXT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(VEXT_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void gps_task(void *arg)
{
    // static: fora da pilha da task
    static uint8_t rx[BUF_SIZE];
    static char line[128];
    size_t pos = 0;

    while (1) {
        int len = uart_read_bytes(GPS_UART, rx, sizeof(rx), pdMS_TO_TICKS(100));
        for (int i = 0; i < len; i++) {
            char c = (char)rx[i];
            if (c == '$') {// inicio de sentenca NMEA
                pos = 0;
            }
            if (c == '\r' || c == '\n') {
                if (pos > 0) {
                    line[pos] = '\0';
#if MOSTRAR_NMEA
                    ESP_LOGI(TAG, "%s", line);
#endif
                    nmea_parse(line);
                    pos = 0;
                }
            } else if (pos < sizeof(line) - 1) {
                line[pos++] = c;
            } else {
                pos = 0;// linha longa demais: descarta
            }
        }
    }
}

// ---------------------------------------------------------------------------
// LoRa
// ---------------------------------------------------------------------------

// Arredonda para o inteiro mais proximo (sem precisar da libm)
static int32_t arredonda(double v)
{
    return (int32_t)(v >= 0 ? v + 0.5 : v - 0.5);
}

// TRANSMISSOR: a cada LORA_INTERVALO_MS empacota a ultima leitura e envia
static void lora_tx_task(void *arg)
{
    uint16_t contador = 0;
    TickType_t ultimo_envio = xTaskGetTickCount();

    while (1) {
        xTaskDelayUntil(&ultimo_envio, pdMS_TO_TICKS(LORA_INTERVALO_MS));

        gps_data_t g;
        taskENTER_CRITICAL(&gps_lock);
        g = gps_ultimo;
        taskEXIT_CRITICAL(&gps_lock);

        pacote_t p = {
            .versao    = PACOTE_VERSAO,
            .flags     = g.valido ? 0x01 : 0x00,
            .contador  = contador,
            .timestamp = (uint32_t)g.timestamp,
            .lat       = arredonda(g.lat * 1e7),
            .lon       = arredonda(g.lon * 1e7),
            .alt       = arredonda(g.alt * 10),
            .sats      = (uint8_t)g.sats,
        };

        esp_err_t err = lora_send((const uint8_t *)&p, sizeof(p));
        if (err == ESP_OK) {
            lora_ultimo_enviado = contador;
            ESP_LOGI(TAG_LORA, "Pacote #%u enviado (%u bytes, %s)", contador,
                     (unsigned)sizeof(p), g.valido ? "com posicao" : "sem posicao");
        } else {
            ESP_LOGW(TAG_LORA, "Falha ao enviar pacote #%u: %s", contador, esp_err_to_name(err));
        }
        contador++;
    }
}

// RECEPTOR: espera pacotes, decodifica e mostra
static void lora_rx_task(void *arg)
{
    uint8_t buf[64];
    int rssi = 0;
    float snr = 0;
    bool primeiro = true;
    uint16_t esperado = 0;
    uint32_t recebidos = 0, perdidos = 0;

    lora_start_rx();

    while (1) {
        int n = lora_receive(buf, sizeof(buf), &rssi, &snr, 1000);
        if (n == 0) {
            continue;  // nada chegou ainda
        }
        if (n < 0) {
            ESP_LOGW(TAG_LORA, "Pacote corrompido (erro de CRC)");
            continue;
        }

        pacote_t p;
        if (n != sizeof(p)) {
            ESP_LOGW(TAG_LORA, "Pacote desconhecido (%d bytes)", n);
            continue;
        }
        memcpy(&p, buf, sizeof(p));
        if (p.versao != PACOTE_VERSAO) {
            ESP_LOGW(TAG_LORA, "Versao de pacote desconhecida: %u", p.versao);
            continue;
        }

        // Pacotes perdidos: saltos no contador
        if (!primeiro && p.contador != esperado) {
            perdidos += (uint16_t)(p.contador - esperado);
        }
        primeiro = false;
        esperado = p.contador + 1;
        recebidos++;

        double lat = p.lat / 1e7;
        double lon = p.lon / 1e7;
        double alt = p.alt / 10.0;
        time_t local = (time_t)p.timestamp + FUSO_HORARIO * 3600;
        struct tm t;
        gmtime_r(&local, &t);

        if (p.flags & 0x01) {
            ESP_LOGI(TAG_LORA, "#%u | %02d/%02d/%04d %02d:%02d:%02d | Lat: %.6f | Lon: %.6f | "
                               "Alt: %.1f m | Sats: %u | RSSI: %d dBm | SNR: %.1f dB | Perdidos: %lu",
                     p.contador, t.tm_mday, t.tm_mon + 1, t.tm_year + 1900,
                     t.tm_hour, t.tm_min, t.tm_sec, lat, lon, alt, p.sats,
                     rssi, snr, (unsigned long)perdidos);
        } else {
            ESP_LOGW(TAG_LORA, "#%u | transmissor sem posicao (sats: %u) | RSSI: %d dBm | SNR: %.1f dB",
                     p.contador, p.sats, rssi, snr);
        }

        if (oled_ok) {
            oled_clear();
            oled_printf(0, 0, "RX #%u  Perd: %lu", p.contador, (unsigned long)perdidos);
            oled_hline(1);
            if (p.flags & 0x01) {
                oled_printf(0, 2, "Lat: %.6f", lat);
                oled_printf(0, 3, "Lon: %.6f", lon);
                oled_printf(0, 4, "Alt: %.1f m", alt);
                oled_printf(0, 5, "%02d/%02d %02d:%02d:%02d S:%u",
                            t.tm_mday, t.tm_mon + 1, t.tm_hour, t.tm_min, t.tm_sec, p.sats);
            } else {
                oled_text(0, 3, "TX sem posicao");
            }
            oled_printf(0, 7, "RSSI:%d SNR:%.1f", rssi, snr);
            oled_update();
        }
    }
}

// ---------------------------------------------------------------------------

static void iniciar_receptor(void)
{
    esp_err_t err = lora_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_LORA, "Falha ao iniciar o radio: %s", esp_err_to_name(err));
        if (oled_ok) {
            oled_text(0, 7, "LoRa: erro");
            oled_update();
        }
        return;
    }
    ESP_LOGI(TAG_LORA, "Receptor pronto: %.1f MHz, SF%d", LORA_FREQ_HZ / 1e6, LORA_SF);
    if (oled_ok) {
        oled_clear();
        oled_text(0, 0, "Receptor LoRa");
        oled_hline(1);
        oled_text(0, 3, "Aguardando pacotes...");
        oled_update();
    }
    xTaskCreate(lora_rx_task, "lora_rx_task", 4096, NULL, 5, NULL);
}

void app_main(void)
{
    vext_on();

    esp_err_t err = oled_init();
    oled_ok = (err == ESP_OK);
    if (oled_ok) {
        oled_text(0, 0, MODO_RECEPTOR ? "Receptor LoRa" : "GPS NEO-6M");
        oled_text(0, 3, "Iniciando...");
        oled_update();
    } else {
        ESP_LOGE(TAG, "Falha ao iniciar o display: %s", esp_err_to_name(err));
    }

    if (MODO_RECEPTOR) {
        iniciar_receptor();
        return;
    }

    const uart_config_t cfg = {
        .baud_rate  = GPS_BAUD_PADRAO,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(GPS_UART, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART, GPS_TX_PIN, GPS_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    if (gps_config()) {
        ESP_LOGI(TAG, "GPS configurado: UART%d (RX=%d, TX=%d), %d baud, %d Hz",
                 GPS_UART, GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD, GPS_TAXA_HZ);
    } else {
        // Sem resposta (ex.: fio GPIO5 -> RX do GPS desligado): volta ao padrao
        uart_set_baudrate(GPS_UART, GPS_BAUD_PADRAO);
        ESP_LOGW(TAG, "GPS nao confirmou a configuracao (verifique GPIO%d -> RX do GPS). "
                      "Usando o padrao: %d baud, 1 Hz",
                 GPS_TX_PIN, GPS_BAUD_PADRAO);
    }

    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);

    err = lora_init();
    if (err == ESP_OK) {
        ESP_LOGI(TAG_LORA, "Transmissor pronto: %.1f MHz, SF%d, %d dBm, a cada %d ms",
                 LORA_FREQ_HZ / 1e6, LORA_SF, LORA_TX_DBM, LORA_INTERVALO_MS);
        xTaskCreate(lora_tx_task, "lora_tx_task", 4096, NULL, 4, NULL);
    } else {
        lora_erro = true;
        ESP_LOGE(TAG_LORA, "Falha ao iniciar o radio: %s", esp_err_to_name(err));
    }
}
