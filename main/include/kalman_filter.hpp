class KalmanFilter {
private:
  float x; // 状態の推定値
  float P; // 状態の誤差共分散行列
  float Q; // プロセスノイズの共分散
  float R; // 観測ノイズの共分散

  float init_P;

public:
  KalmanFilter();
  void init(float initial_x, float initial_P, float process_noise,
            float measurement_noise);
  void predict(float u);

  void update(float z);

  float get_state();

  void reset(float reset_val);
};