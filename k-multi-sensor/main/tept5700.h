#ifndef TEPT5700_H
#define TEPT5700_H

#include "inttypes.h"

/**
 * @brief tept5700_init
 * @param[in] Vce Collector tension (volts)
 * @param[in] Rl : Load resistor (ohms)
 */
void tept5700_init(float Vce, float Rl);

/**
 * @brief tept5700_v_to_lux
 * @param[in] Vout: measured tension on Rl
 * @param[in] temp: circuit ambiant temperature (Not implemented, concidered to be 24°C)
 * @return
 */
float tept5700_v_to_lux(float Vrl, float temp);

#endif // TEPT5700_H
