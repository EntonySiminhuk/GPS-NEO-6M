#include <math.h>
#include "gps_filtro.h"

void gps_validador_init(gps_validador_t *v)
{
    v->seguidas = 0;
}

float gps_erro_horizontal(const gps_leitura_t *l)
{
    if (l->erro_lat < 0 || l->erro_lon < 0) {
        return -1;
    }
    return sqrtf(l->erro_lat * l->erro_lat + l->erro_lon * l->erro_lon);
}

// Criterios que dependem so da leitura atual
static gps_motivo_t avaliar(const gps_leitura_t *l)
{
    if (!l->rmc_ativo || (l->qualidade != 1 && l->qualidade != 2)) {
        return GPS_REJ_SEM_FIX;
    }
    // Sem GSA (fix = 0) o tipo de fix e desconhecido: o minimo de satelites cobre
    if (l->fix != 0 && l->fix < 3) {
        return GPS_REJ_FIX_2D;
    }
    if (l->sats < GPS_MIN_SATS) {
        return GPS_REJ_POUCOS_SATS;
    }
    if (l->hdop < 0 || l->hdop > GPS_MAX_HDOP) {
        return GPS_REJ_HDOP;
    }
    // Sem GST (alguns clones nao enviam) o criterio e ignorado
    float erro = gps_erro_horizontal(l);
    if (GPS_MAX_ERRO_M > 0 && erro >= 0 && erro > GPS_MAX_ERRO_M) {
        return GPS_REJ_ERRO_ALTO;
    }
    return GPS_OK;
}

gps_motivo_t gps_validar(gps_validador_t *v, const gps_leitura_t *l)
{
    gps_motivo_t m = avaliar(l);
    if (m != GPS_OK) {
        v->seguidas = 0;
        return m;
    }
    // Logo apos ganhar (ou recuperar) o fix a solucao ainda esta convergindo
    if (v->seguidas < GPS_LEITURAS_ESTAVEIS) {
        v->seguidas++;
    }
    return (v->seguidas < GPS_LEITURAS_ESTAVEIS) ? GPS_REJ_ESTABILIZANDO : GPS_OK;
}

const char *gps_motivo_str(gps_motivo_t m)
{
    switch (m) {
    case GPS_OK:                return "ok";
    case GPS_REJ_SEM_FIX:       return "sem fix";
    case GPS_REJ_FIX_2D:        return "fix 2D";
    case GPS_REJ_POUCOS_SATS:   return "poucos sats";
    case GPS_REJ_HDOP:          return "HDOP alto";
    case GPS_REJ_ERRO_ALTO:     return "erro alto";
    case GPS_REJ_ESTABILIZANDO: return "estabiliz.";
    }
    return "?";
}
