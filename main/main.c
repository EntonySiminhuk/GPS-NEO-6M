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
#include "gps_filtro.h"

static const char *TAG = "GPS";
static const char *TAG_LORA = "LORA";

// Intervalo entre transmissoes LoRa
#define LORA_INTERVALO_MS   5000

#define GPS_UART    UART_NUM_1
#define GPS_RX_PIN  4    // ligado ao TX do GPS
#define GPS_TX_PIN  5    // ligado ao RX do GPS (necessario para configurar o GPS)
#define GPS_BAUD_PADRAO 9600   // velocidade de fabrica do NEO-6M
#define GPS_BAUD    38400      // velocidade usada depois da configuracao
#define GPS_TAXA_HZ 5          // posicoes por segundo desejadas (NEO-6M: 1 a 5)
#define BUF_SIZE    1024

// Heltec WiFi LoRa 32 V3: Vext (GPIO36) alimenta o pino 3V3 "Vext" e o OLED.
// Nivel LOW = ligado. Necessario se o GPS estiver alimentado pelo Vext.
#define VEXT_PIN    GPIO_NUM_36

// Fuso horario em horas em relacao ao UTC (Brasilia = -3)
#define FUSO_HORARIO   (-3)

// 1 = mostra tambem as sentencas NMEA brutas (para depuracao)
#define MOSTRAR_NMEA   0

// 1 = imprime TODAS as leituras (5 Hz) em linhas "CSV,..." para analise no PC.
// Capture o monitor serial em arquivo e filtre as linhas que comecam com "CSV,".
#define SAIDA_CSV      0

// Modelo dinamico do filtro interno do GPS (UBX-CFG-NAV5):
// 0 portatil, 2 parado, 3 pedestre, 4 automotivo, 5 maritimo,
// 6 aereo < 1g, 7 aereo < 2g, 8 aereo < 4g
#define GPS_MODELO_DINAMICO 6

// Sentencas que formam uma epoca (uma medicao completa)
#define SENT_RMC  (1 << 0)
#define SENT_GGA  (1 << 1)
#define SENT_GSA  (1 << 2)
#define SENT_GST  (1 << 3)

// Epoca publicada para as outras tasks
typedef struct {
    gps_leitura_t l;
    gps_motivo_t  motivo;  // GPS_OK = aprovada pelo filtro
    TickType_t    tick;    // instante em que foi publicada
} gps_estado_t;

static gps_leitura_t gps;            // epoca sendo montada (so a gps_task usa)
static gps_validador_t validador;    // estado da camada 1 (so a gps_task usa)
static gps_estado_t gps_ultimo;      // ultima epoca completa, aprovada ou nao
static gps_estado_t gps_aceito;      // ultima epoca aprovada
static bool gps_tem_aceito;
static portMUX_TYPE gps_lock = portMUX_INITIALIZER_UNLOCKED;

// Montagem da epoca: RMC, GGA e GST trazem a hora; a GSA nao
static uint8_t epoca_esperadas = SENT_RMC | SENT_GGA | SENT_GSA;  // GST entra se o GPS aceitar
static uint8_t epoca_recebidas;
static bool epoca_publicada;
static char epoca_hora[16];

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
static const char *fix_str(int fix)
{
    switch (fix) {
    case 3:  return "3D";
    case 2:  return "2D";
    case 1:  return "--";
    default: return "?";
    }
}

// Valor opcional (negativo = desconhecido) como texto: "1.2" ou "--"
static const char *opcional(char *buf, size_t n, const char *fmt, float v)
{
    if (v < 0) {
        return "--";
    }
    snprintf(buf, n, fmt, v);
    return buf;
}

static void gps_print(const gps_estado_t *e)
{
    const gps_leitura_t *g = &e->l;
    const bool aceito = (e->motivo == GPS_OK);

    // Converte o timestamp UTC para o horario local
    time_t local = g->timestamp + FUSO_HORARIO * 3600;
    struct tm t;
    gmtime_r(&local, &t);

    char buf_hdop[8], buf_erro[8];
    const char *hdop = opcional(buf_hdop, sizeof(buf_hdop), "%.1f", g->hdop);
    const char *erro = opcional(buf_erro, sizeof(buf_erro), "%.1f m", gps_erro_horizontal(g));

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
        if (aceito) {
            oled_printf(0, 2, "Lat: %.6f", g->lat);
            oled_printf(0, 3, "Lon: %.6f", g->lon);
            oled_printf(0, 4, "Alt: %.1f m", g->alt);
        } else if (e->motivo == GPS_REJ_SEM_FIX) {
            oled_text(0, 3, "Aguardando posicao...");
        } else {
            oled_text(0, 2, "Posicao rejeitada:");
            oled_printf(0, 3, "  %s", gps_motivo_str(e->motivo));
        }
        // Erro estimado arredondado para caber nas 21 colunas
        char buf_e[8];
        oled_printf(0, 5, "%s HDOP:%s e:%s", fix_str(g->fix), hdop,
                    opcional(buf_e, sizeof(buf_e), "%.0fm", gps_erro_horizontal(g)));
        oled_printf(0, 6, "Satelites: %d", g->sats);
        if (lora_erro) {
            oled_text(0, 7, "LoRa: erro");
        } else if (lora_ultimo_enviado >= 0) {
            oled_printf(0, 7, "LoRa: #%d enviado", lora_ultimo_enviado);
        }
        oled_update();
    }

    // Terminal: no maximo 1 linha por segundo, com a taxa medida (leituras/s)
    static TickType_t ultimo_log;
    static int leituras;
    leituras++;
    TickType_t agora = xTaskGetTickCount();
    if (agora - ultimo_log < pdMS_TO_TICKS(1000)) {
        return;
    }
    int taxa = leituras;
    leituras = 0;
    ultimo_log = agora;

    if (e->motivo == GPS_REJ_SEM_FIX) {
        ESP_LOGW(TAG, "Aguardando posicao... (satelites: %d) | %d Hz", g->sats, taxa);
        return;
    }
    if (!aceito) {
        ESP_LOGW(TAG, "Rejeitada (%s) | Fix: %s | Sats: %d | HDOP: %s | Erro: %s | %d Hz",
                 gps_motivo_str(e->motivo), fix_str(g->fix), g->sats, hdop, erro, taxa);
        return;
    }

    ESP_LOGI(TAG, "%02d/%02d/%04d %02d:%02d:%02d.%d | Lat: %.6f | Lon: %.6f | Alt: %.1f m | "
                  "Sats: %d | HDOP: %s | Erro: %s | %d Hz",
             t.tm_mday, t.tm_mon + 1, t.tm_year + 1900,
             t.tm_hour, t.tm_min, t.tm_sec, g->ms / 100,
             g->lat, g->lon, g->alt, g->sats, hdop, erro, taxa);
}

#if SAIDA_CSV
// Uma linha por epoca, aprovada ou nao, para reprocessar no PC
static void gps_csv(const gps_estado_t *e)
{
    static bool cabecalho;
    if (!cabecalho) {
        printf("CSV,tick_ms,unix,ms,lat,lon,alt,vel_ms,curso,qualidade,fix,sats,"
               "hdop,pdop,vdop,erro_lat,erro_lon,erro_alt,motivo\n");
        cabecalho = true;
    }
    const gps_leitura_t *g = &e->l;
    printf("CSV,%lu,%lld,%d,%.7f,%.7f,%.1f,%.2f,%.1f,%d,%d,%d,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%s\n",
           (unsigned long)pdTICKS_TO_MS(e->tick), (long long)g->timestamp, g->ms,
           g->lat, g->lon, g->alt, g->vel_ms, g->curso, g->qualidade, g->fix, g->sats,
           g->hdop, g->pdop, g->vdop, g->erro_lat, g->erro_lon, g->erro_alt,
           gps_motivo_str(e->motivo));
}
#endif

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

// Campo numerico opcional: vazio -> -1 (desconhecido)
static float nmea_float(const char *s)
{
    return (*s) ? (float)atof(s) : -1.0f;
}

static int nmea_int(const char *s)
{
    return (*s) ? atoi(s) : 0;
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

// Marca todos os campos como "nao recebido" no inicio de cada epoca, para
// que uma sentenca perdida nao deixe valores da medicao anterior
static void epoca_limpar(void)
{
    gps = (gps_leitura_t){
        .vel_ms = -1, .curso = -1,
        .hdop = -1, .pdop = -1, .vdop = -1,
        .erro_lat = -1, .erro_lon = -1, .erro_alt = -1,
    };
}

// Passa a epoca montada pela camada 1 e publica para as outras tasks
static void epoca_publicar(void)
{
    gps_estado_t e = {
        .l      = gps,
        .motivo = gps_validar(&validador, &gps),
        .tick   = xTaskGetTickCount(),
    };

    taskENTER_CRITICAL(&gps_lock);
    gps_ultimo = e;
    if (e.motivo == GPS_OK) {
        gps_aceito = e;
        gps_tem_aceito = true;
    }
    taskEXIT_CRITICAL(&gps_lock);

    epoca_publicada = true;
#if SAIDA_CSV
    gps_csv(&e);
#endif
    gps_print(&e);
}

// Uma nova epoca comeca quando a hora muda ou quando chega de novo um tipo
// de sentenca que ja foi recebido. A epoca anterior e publicada mesmo
// incompleta (ex.: GST perdida) desde que tenha RMC ou GGA.
static void epoca_sentenca(uint8_t sent, const char *hora)
{
    bool nova = (epoca_recebidas & sent) != 0;
    if (hora && strcmp(hora, epoca_hora) != 0) {
        nova = true;
    }
    if (nova) {
        if (!epoca_publicada && (epoca_recebidas & (SENT_RMC | SENT_GGA))) {
            epoca_publicar();
        }
        epoca_limpar();
        epoca_recebidas = 0;
        epoca_publicada = false;
    }
    if (hora) {
        strlcpy(epoca_hora, hora, sizeof(epoca_hora));
    }
}

// Chamada depois de interpretar a sentenca: publica assim que a epoca fica completa
static void epoca_recebida(uint8_t sent)
{
    epoca_recebidas |= sent;
    if (!epoca_publicada && (epoca_recebidas & epoca_esperadas) == epoca_esperadas) {
        epoca_publicar();
    }
}

static void nmea_parse(char *linha)
{
    if (!nmea_checksum_ok(linha) || strlen(linha) < 6) {
        return;
    }

    char *f[20];
    int n = nmea_split(linha, f, 20);
    const char *tipo = f[0] + 3;  // ignora "$GP"/"$GN"

    // $GPRMC,hora,status,lat,N/S,lon,E/W,vel(nos),curso,data,...
    if (strcmp(tipo, "RMC") == 0 && n >= 10) {
        epoca_sentenca(SENT_RMC, f[1]);
        gps.rmc_ativo = (f[2][0] == 'A');
        if (gps.rmc_ativo) {
            gps.lat = nmea_coord(f[3], f[4]);
            gps.lon = nmea_coord(f[5], f[6]);
        }
        float nos = nmea_float(f[7]);
        gps.vel_ms = (nos >= 0) ? nos * 0.514444f : -1;
        gps.curso = nmea_float(f[8]);
        gps.timestamp = nmea_time(f[1], f[9]);
        // fracao do segundo: "183535.20" -> 200 ms
        gps.ms = (strlen(f[1]) > 6) ? (int)(atof(f[1] + 6) * 1000 + 0.5) : 0;
        epoca_recebida(SENT_RMC);
    }
    // $GPGGA,hora,lat,N/S,lon,E/W,qualidade,sats,hdop,alt,M,...
    else if (strcmp(tipo, "GGA") == 0 && n >= 10) {
        epoca_sentenca(SENT_GGA, f[1]);
        gps.qualidade = nmea_int(f[6]);
        gps.sats = nmea_int(f[7]);
        gps.hdop = nmea_float(f[8]);
        gps.alt  = atof(f[9]);
        epoca_recebida(SENT_GGA);
    }
    // $GPGSA,modo,fix,sat1..sat12,pdop,hdop,vdop  (sem hora)
    else if (strcmp(tipo, "GSA") == 0 && n >= 18) {
        epoca_sentenca(SENT_GSA, NULL);
        gps.fix  = nmea_int(f[2]);
        gps.pdop = nmea_float(f[15]);
        gps.vdop = nmea_float(f[17]);
        epoca_recebida(SENT_GSA);
    }
    // $GPGST,hora,rms,semieixo_maior,semieixo_menor,orientacao,erro_lat,erro_lon,erro_alt
    else if (strcmp(tipo, "GST") == 0 && n >= 9) {
        epoca_sentenca(SENT_GST, f[1]);
        gps.erro_lat = nmea_float(f[6]);
        gps.erro_lon = nmea_float(f[7]);
        gps.erro_alt = nmea_float(f[8]);
        epoca_recebida(SENT_GST);
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
// Retorna a taxa aceita em Hz, ou 0 se o GPS nao respondeu em GPS_BAUD
// (nesse caso ele continua em GPS_BAUD_PADRAO).
static int gps_config(void)
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

    // UBX-CFG-MSG: RMC e GGA ja vem ligadas de fabrica. Liga GSA (tipo de fix
    // e DOP) e GST (erro estimado em metros), desliga GLL, GSV e VTG.
    // Se o primeiro ACK chegar, o GPS esta mesmo em GPS_BAUD.
    static const struct { uint8_t id; uint8_t taxa; uint8_t sent; const char *nome; } msgs[] = {
        {0x01, 0, 0,        "GLL"},
        {0x02, 1, SENT_GSA, "GSA"},
        {0x03, 0, 0,        "GSV"},
        {0x05, 0, 0,        "VTG"},
        {0x07, 1, SENT_GST, "GST"},
    };
    for (size_t i = 0; i < sizeof(msgs) / sizeof(msgs[0]); i++) {
        const uint8_t cfg_msg[3] = {0xF0, msgs[i].id, msgs[i].taxa};  // classe NMEA, id, taxa
        if (ubx_cmd(0x06, 0x01, cfg_msg, sizeof(cfg_msg))) {
            epoca_esperadas |= msgs[i].sent;
        } else if (i == 0) {
            return 0;
        } else {
            ESP_LOGW(TAG, "GPS nao aceitou configurar a sentenca %s", msgs[i].nome);
        }
    }

    // UBX-CFG-NAV5: modelo dinamico e fix somente 3D. Com 3 satelites o
    // NEO-6M faz fix 2D supondo a altitude, e a posicao pode errar centenas
    // de metros: assim ele passa a informar "sem fix" nesses casos.
    uint8_t cfg_nav5[36] = {0};
    cfg_nav5[0] = 0x05;                 // mascara: aplica dynModel (bit 0) e fixMode (bit 2)
    cfg_nav5[2] = GPS_MODELO_DINAMICO;
    cfg_nav5[3] = 2;                    // fixMode: 1 = so 2D, 2 = so 3D, 3 = automatico
    if (!ubx_cmd(0x06, 0x24, cfg_nav5, sizeof(cfg_nav5))) {
        ESP_LOGW(TAG, "GPS nao aceitou o CFG-NAV5 (modelo dinamico / fix 3D)");
    }

    // UBX-CFG-RATE: intervalo entre medicoes em ms, 1 navegacao por medicao, hora GPS.
    // Alguns modulos (clones ou firmwares antigos) recusam taxas altas:
    // tenta GPS_TAXA_HZ e vai descendo ate 1 Hz. O baud continua GPS_BAUD.
    for (int taxa = GPS_TAXA_HZ; taxa > 1; taxa--) {
        const uint16_t periodo = 1000 / taxa;
        const uint8_t cfg_rate[6] = {
            (uint8_t)(periodo & 0xFF), (uint8_t)(periodo >> 8),
            0x01, 0x00,
            0x01, 0x00,
        };
        if (ubx_cmd(0x06, 0x08, cfg_rate, sizeof(cfg_rate))) {
            return taxa;
        }
        ESP_LOGW(TAG, "GPS recusou %d Hz, tentando uma taxa menor", taxa);
    }
    return 1;  // padrao de fabrica do GPS
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

        gps_estado_t ultimo, aceito;
        bool tem_aceito;
        taskENTER_CRITICAL(&gps_lock);
        ultimo = gps_ultimo;
        aceito = gps_aceito;
        tem_aceito = gps_tem_aceito;
        taskEXIT_CRITICAL(&gps_lock);

        // So transmite posicao aprovada pelo filtro e recente
        const TickType_t agora = xTaskGetTickCount();
        const TickType_t idade_max = pdMS_TO_TICKS(GPS_MAX_IDADE_MS);
        const bool com_posicao = tem_aceito && (agora - aceito.tick) <= idade_max;

        // Motivo exibido no log quando o pacote vai sem posicao
        const char *estado = "com posicao";
        if (!com_posicao) {
            if (ultimo.tick == 0 || agora - ultimo.tick > idade_max) {
                estado = "GPS sem dados";
            } else if (ultimo.motivo != GPS_OK) {
                estado = gps_motivo_str(ultimo.motivo);
            } else {
                estado = "leitura antiga";
            }
        }

        const gps_leitura_t *g = com_posicao ? &aceito.l : &ultimo.l;
        pacote_t p = {
            .versao    = PACOTE_VERSAO,
            .flags     = com_posicao ? 0x01 : 0x00,
            .contador  = contador,
            .timestamp = (uint32_t)g->timestamp,
            .lat       = com_posicao ? arredonda(g->lat * 1e7) : 0,
            .lon       = com_posicao ? arredonda(g->lon * 1e7) : 0,
            .alt       = com_posicao ? arredonda(g->alt * 10) : 0,
            .sats      = (uint8_t)g->sats,
        };

        esp_err_t err = lora_send((const uint8_t *)&p, sizeof(p));
        if (err == ESP_OK) {
            lora_ultimo_enviado = contador;
            ESP_LOGI(TAG_LORA, "Pacote #%u enviado (%u bytes, %s%s)", contador, (unsigned)sizeof(p),
                     com_posicao ? "" : "sem posicao: ", estado);
        } else {
            ESP_LOGW(TAG_LORA, "Falha ao enviar pacote #%u: %s", contador, esp_err_to_name(err));
        }
        contador++;
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

    int taxa = gps_config();
    if (taxa > 0) {
        ESP_LOGI(TAG, "GPS configurado: UART%d (RX=%d, TX=%d), %d baud, %d Hz",
                 GPS_UART, GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD, taxa);
    } else {
        // Sem resposta (ex.: fio GPIO5 -> RX do GPS desligado): volta ao padrao
        uart_set_baudrate(GPS_UART, GPS_BAUD_PADRAO);
        ESP_LOGW(TAG, "GPS nao confirmou a configuracao (verifique GPIO%d -> RX do GPS). "
                      "Usando o padrao: %d baud, 1 Hz",
                 GPS_TX_PIN, GPS_BAUD_PADRAO);
    }

    gps_validador_init(&validador);
    epoca_limpar();
    xTaskCreate(gps_task, "gps_task", 6144, NULL, 5, NULL);  // printf de float usa bastante pilha

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
