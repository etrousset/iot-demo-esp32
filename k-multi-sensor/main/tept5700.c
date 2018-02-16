#include "tept5700.h"
#include "math.h"

#include "esp_log.h"

#define TAG "TEPT5700"

static float _Vce = 0;
static float _Rl  = 0;

// - Vce = 5v -
// afine line parameters in log plan when Vce = 5v
#define m 1.0f
#define k 1.3333333333333333f

void tept5700_init(float Vce, float Rl)
{
    _Vce = Vce;
    _Rl  = Rl;
}

float tept5700_v_to_lux(float Vrl, float temp)
{
    float i_ua = Vrl / _Rl * 1000000.f;
    float lux  = k * pow(i_ua, m);

//    ESP_LOGD(TAG, "Vrl = %fv, I = %fuA, L = %f lux", Vrl, i_ua, lux);
    return lux;
}
