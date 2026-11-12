#pragma once

// Filtro das leituras do GPS, em camadas:
//   1. validacao da qualidade (este arquivo, por enquanto)
//   2. rejeicao de outliers (a fazer)
//   3. suavizacao (a fazer)
//
// C puro, sem dependencia do ESP-IDF: pode ser compilado no PC para
// reprocessar logs gravados com a saida CSV.

#include <stdbool.h>
#include <time.h>

// ---------------------------------------------------------------------------
// CRITERIOS DE QUALIDADE (camada 1). Sao pontos de partida experimentais:
// ajustar com base em testes. Passar nos criterios nao garante precisao.
// ---------------------------------------------------------------------------
#define GPS_MIN_SATS            5      // satelites usados no calculo (fix 3D exige 4)
#define GPS_MAX_HDOP            2.5f   // diluicao horizontal da precisao
#define GPS_MAX_ERRO_M          10.0f  // erro horizontal estimado pela GST (m); 0 = nao usar
#define GPS_LEITURAS_ESTAVEIS   5      // leituras boas seguidas antes de aceitar (5 = 1 s a 5 Hz)
#define GPS_MAX_IDADE_MS        1000   // posicao mais antiga que isso nao e transmitida

// Uma epoca de navegacao completa (RMC + GGA + GSA + GST do mesmo instante).
// Valores negativos (ou 0 em `fix`) indicam campo nao recebido.
typedef struct {
    // RMC
    bool   rmc_ativo;   // status 'A' (o receptor diz que a posicao e valida)
    double lat;         // graus decimais (sul negativo)
    double lon;         // graus decimais (oeste negativo)
    float  vel_ms;      // velocidade sobre o solo (m/s)
    float  curso;       // direcao do movimento (graus, 0 = norte)
    time_t timestamp;   // Unix time (UTC)
    int    ms;          // milissegundos dentro do segundo
    // GGA
    int    qualidade;   // 0 = sem fix, 1 = GPS, 2 = DGPS, 6 = estimado
    int    sats;        // satelites usados no calculo
    float  alt;         // altitude (m, nivel do mar)
    float  hdop;
    // GSA
    int    fix;         // 1 = sem fix, 2 = 2D, 3 = 3D
    float  pdop;
    float  vdop;
    // GST: desvio-padrao estimado do erro, em metros
    float  erro_lat;
    float  erro_lon;
    float  erro_alt;
} gps_leitura_t;

typedef enum {
    GPS_OK = 0,
    GPS_REJ_SEM_FIX,        // RMC 'V' ou GGA qualidade 0/estimada
    GPS_REJ_FIX_2D,         // so 2D: a altitude e suposta e a posicao pode errar centenas de metros
    GPS_REJ_POUCOS_SATS,
    GPS_REJ_HDOP,
    GPS_REJ_ERRO_ALTO,      // erro estimado pela GST acima do limite
    GPS_REJ_ESTABILIZANDO,  // leitura boa, mas ainda nao houve GPS_LEITURAS_ESTAVEIS seguidas
} gps_motivo_t;

typedef struct {
    int seguidas;  // leituras boas seguidas
} gps_validador_t;

void gps_validador_init(gps_validador_t *v);

// Camada 1: avalia uma epoca. Retorna GPS_OK se ela pode ser usada.
gps_motivo_t gps_validar(gps_validador_t *v, const gps_leitura_t *l);

// Erro horizontal estimado (m) a partir da GST, ou -1 se desconhecido
float gps_erro_horizontal(const gps_leitura_t *l);

// Texto curto (ate 11 caracteres) para o display e o log
const char *gps_motivo_str(gps_motivo_t m);
