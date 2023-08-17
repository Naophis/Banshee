# 調整

## 基本機能確認
* UI(ボタン)確認
  * GPIO
* UI(LED)確認
  * I2C
* UI(ブザー)確認
  * PWM(LED PWM)
* ADCポート対応確認
  * バッテリー
  * LEDセンサー
* 駆動モーター左右PWM確認
  * Motor PWM
* 駆動モーターCW/CCW確認
  * GPIO
* 吸引モーター確認
  * Motor PWM

## 単体機能の調整
* センサーLED発光時間調整
  * `led_light_delay_cnt`をセンサー値が上がりきるまで長くする
  * mode:14
* センサー曲線フィッティング
  * mode:15 + ./csv/sensor.sh
* 駆動モーター PWM周波数調整
  * mode:13, mode:2, mode:3
* 吸引モーター　PWM周波数調整
  * mode:11

## ゲイン調整
* 速度制御
  * mode:13, 2
* 角速度制御
  * mode:13, 3
* LEDセンサー制御
  * mode:2 (斜めは後で)

## 単体機能調整
* LEDセンサーと壁との距離オフセット調整
  * mode:2 
* タイヤ径調整
  * mode:2
  * できるだけ長い距離でテスト
* 前壁制御調整: search.exist
  * mode:7
  * 反応距離閾値：`front_ctrl_th`
  * ref距離：`front_ctrl`
  * 角度オフセット: `kireme_r` 負の値で左にオフセットする
  * 前壁制御DutyMAX値： `offset_l`
  * [終了条件]前壁と偏差許容値: `kireme_l`
  * [終了条件]前壁との傾き許容偏差: `offset_r`
* 探索時壁切れ考慮位置調整
  * mode:2 (片側の壁制御を切りながら)
  * `clear_dist_ragne_from`
  * `clear_dist_ragne_to`
* 探索前壁調整：
  * mode:5
  * `front_dist_offset`
* 角速度ゲイン調整
  * mode:3
* FF調整
  * mode:2, 3
* 壁切れ調整
  * 
* 探索スラローム
  * 左右壁オフセットref: `sla_wall_ref_l`/`sla_wall_ref_r`

  * 探索前壁位置調整: 

## テスト



### 探索テスト

### 