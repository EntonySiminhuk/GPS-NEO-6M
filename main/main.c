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

static const char *TAG = "GPS";

#define GPS_UART    UART_NUM_1
#define GPS_RX_PIN  4    // ligado ao TX do GPS
#define GPS_TX_PIN  5    // ligado ao RX do GPS
#define GPS_BAUD    9600 // padrao do NEO-6M
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
} gps_data_t;

static gps_data_t gps;

static bool oled_ok;

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
            oled_printf(0, 0, "%02d/%02d/%04d %02d:%02d:%02d",
                        t.tm_mday, t.tm_mon + 1, t.tm_year + 1900,
                        t.tm_hour, t.tm_min, t.tm_sec);
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
        oled_update();
    }

    if (!g->valido) {
        ESP_LOGW(TAG, "Aguardando posicao... (satelites: %d)", g->sats);
        return;
    }

    ESP_LOGI(TAG, "%02d/%02d/%04d %02d:%02d:%02d | Lat: %.6f | Lon: %.6f | Alt: %.1f m",
             t.tm_mday, t.tm_mon + 1, t.tm_year + 1900,
             t.tm_hour, t.tm_min, t.tm_sec,
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
    }
    // $GPGGA,hora,lat,N/S,lon,E/W,qualidade,sats,hdop,alt,M,...
    // GGA chega depois do RMC em cada ciclo: imprime aqui
    else if (strcmp(tipo, "GGA") == 0 && n >= 10) {
        gps.sats = atoi(f[7]);
        gps.alt  = atof(f[9]);
        gps_print(&gps);
    }
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
            if (c == '$') {           // inicio de sentenca NMEA
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

void app_main(void)
{
    vext_on();

    esp_err_t err = oled_init();
    oled_ok = (err == ESP_OK);
    if (oled_ok) {
        oled_text(0, 0, "GPS NEO-6M");
        oled_text(0, 3, "Iniciando...");
        oled_update();
    } else {
        ESP_LOGE(TAG, "Falha ao iniciar o display: %s", esp_err_to_name(err));
    }

    const uart_config_t cfg = {
        .baud_rate  = GPS_BAUD,
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

    ESP_LOGI(TAG, "UART%d pronta (RX=%d, TX=%d, %d baud)",
             GPS_UART, GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);

    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
}
