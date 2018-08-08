#include "PID_ASYNC.h"

#ifndef PID_PARAM_CONFIG
  #define PID_PARAM_CONFIG

Coefficient k_param[9];

void init_params() {
  k_param[0].Kp = 0.3;
  k_param[0].Ki = 0.0001;
  k_param[0].Kd = 0;

  k_param[1].Kp = 0.8;
  k_param[1].Ki = 0.0001;
  k_param[1].Kd = 0;

  k_param[2].Kp = 1;
  k_param[2].Ki = 0.0001;
  k_param[2].Kd = 0;

  k_param[3].Kp = 1.5;
  k_param[3].Ki = 0.0002;
  k_param[3].Kd = 0;

  k_param[4].Kp = 2;
  k_param[4].Ki = 0.0002;
  k_param[4].Kd = 0;

  k_param[5].Kp = 3;
  k_param[5].Ki = 0.0002;
  k_param[5].Kd = 0;

  k_param[6].Kp = 3;
  k_param[6].Ki = 0.001;
  k_param[6].Kd = 0;

  k_param[7].Kp = 3;
  k_param[7].Ki = 0.001;
  k_param[7].Kd = 0;

  k_param[8].Kp = 7;
  k_param[8].Ki = 0.002;
  k_param[8].Kd = 0;
}

CoefficientPtr search() {
  double _val = min(Setpoint, Input);
  uint8_t index = 0;

  if (_val >= 550) {
    index = 0;
  } else if (_val >= 500 && _val < 550) {
    index = 1;
  } else if (_val >= 450 && _val < 500) {
    index = 2;
  } else if (_val >= 400 && _val < 450) {
    index = 3;
  } else if (_val >= 350 && _val < 400) {
    index = 4;
  } else if (_val >= 250 && _val < 350) {
    index = 5;
  } else if (_val >= 200 && _val < 250) {
    index = 6;
  } else if (_val >= 150 && _val < 200) {
    index = 7;
  } else if (_val <= 150) {
    index = 8;
  }
  // Serial.println(index);
  return &k_param[index];
}

#endif