/** @file filtres.h  Filtres simples et utilitaires d'angles (code pur, sans matériel). */
#ifndef FILTRES_H
#define FILTRES_H

#include <stdbool.h>

/** Médiane de 3 valeurs : supprime un pic isolé (parasite de mesure). */
float filt_median3(float a, float b, float c);

/** Filtre passe-bas exponentiel : y += alpha * (x - y). alpha dans ]0,1]. */
typedef struct {
    float alpha;
    float y;
    bool  init;
} ema_t;

void  ema_init(ema_t *e, float alpha);
void  ema_reset(ema_t *e);
/** Les entrées NaN/inf sont ignorées (la sortie précédente est conservée). */
float ema_update(ema_t *e, float x);
/** alpha équivalent à une fréquence de coupure fc (Hz) pour une période dt (s). */
float ema_alpha_from_fc(float fc_hz, float dt_s);

/** Ramène un angle en degrés dans [-180, 180[ ou [0, 360[. */
float wrap180(float deg);
float wrap360(float deg);

#endif
