
#include "include/kalman_filter.hpp"

KalmanFilter::KalmanFilter() {}
void KalmanFilter::init(float initial_x, float initial_P, float process_noise,
                        float measurement_noise) {
  x = initial_x;
  P = init_P = initial_P;
  Q = process_noise;
  R = measurement_noise;
}

void KalmanFilter::predict(float u) {
  // 予測ステップ
  x = x + u * 0.001;
  P = P + Q;
}

void KalmanFilter::update(float z) {
  // 更新ステップ
  float K = P / (P + R);
  x = x + K * (z - x);
  P = (1 - K) * P;
}

float KalmanFilter::get_state() {
  return x; //
}

void KalmanFilter::reset(float reset_val) {
  x = reset_val;
  P = init_P;
}