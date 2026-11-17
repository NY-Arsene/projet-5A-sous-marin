#include "profondeur.h"

#define G              9.80665f   /* gravité, m/s² */
#define RHO_EAU_DOUCE  997.0f     /* masse volumique, kg/m³ */

float profondeur_m(float p_mbar, float p_surface_mbar)
{
    /* P = rho * g * h  donc  h = P / (rho * g) ; le x100 convertit les mbar en Pa */
    return (p_mbar - p_surface_mbar) * 100.0f / (RHO_EAU_DOUCE * G);
}