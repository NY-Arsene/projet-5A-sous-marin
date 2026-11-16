#include "types_capteurs.h"

const char *sens_status_str(sens_status_t s)
{
    switch (s) {
    case SENS_OK:            return "OK";
    case SENS_ERR_BUS:       return "ERR_BUS";
    case SENS_ERR_ID:        return "ERR_ID";
    case SENS_ERR_CRC:       return "ERR_CRC";
    case SENS_ERR_PARAM:     return "ERR_PARAM";
    case SENS_ERR_NOT_READY: return "NOT_READY";
    case SENS_ERR_RANGE:     return "ERR_RANGE";
    case SENS_ERR_TIMEOUT:   return "ERR_TIMEOUT";
    default:                 return "?";
    }
}
