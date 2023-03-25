#include "include/ui.hpp"

void UserInterface::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_sensing_result) {
  sensing_result = _sensing_result;
}

void UserInterface::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
}

bool UserInterface::button_state() { return !gpio_get_level(SW1); }

bool UserInterface::button_state_hold() {
  if (!gpio_get_level(SW1)) {
    while (!gpio_get_level(SW1))
      ;
    return true;
  } else {
    return false;
  }
}

int UserInterface::encoder_operation() {
  float v_r = sensing_result->ego.v_r;
  if (v_r > ENC_OPE_V_R_TH) {
    music_sync(MUSIC::G6_, 75);
    return 1;
  }
  if (v_r < -ENC_OPE_V_R_TH) {
    music_sync(MUSIC::C6_, 75);
    return -1;
  }
  return 0;
}
void UserInterface::music_async(MUSIC m, int time) {
  tgt_val->buzzer.hz = (int)m;
  tgt_val->buzzer.time = time;
  int buzzer_timestamp = tgt_val->buzzer.timstamp;
  tgt_val->buzzer.timstamp = ++buzzer_timestamp;
}
void UserInterface::music_sync(MUSIC m, int time) {
  music_async(m, time);
  vTaskDelay(time / portTICK_PERIOD_MS);
}
void UserInterface::motion_check() {
  int c = 0;
  tgt_val->nmr.motion_type = MotionType::READY;
  tgt_val->nmr.timstamp++;
  xQueueReset(*qh);
  xQueueSendToFront(*qh, &tgt_val, 1);
  vTaskDelay(1.0 / portTICK_PERIOD_MS);
  while (1) {
    c++;
    if (c % 2 == 0) {
      LED_on_all();
    } else {
      LED_off_all();
    }
    if (button_state_hold()) {
      break;
    }
    if (sensing_result->ego.left90_mid_dist < 60 &&
        sensing_result->ego.right90_mid_dist < 60 &&
        sensing_result->ego.left90_mid_dist > 10 &&
        sensing_result->ego.right90_mid_dist > 10) {
      LED_off_all();
      for (int i = 0; i < 2; i++) {
        music_sync(MUSIC::C6_, 100);
        vTaskDelay(tgt_val->buzzer.time / portTICK_PERIOD_MS);
      }
      break;
    }
    vTaskDelay(50.0 / portTICK_PERIOD_MS);
  }
}

void UserInterface::coin(int time) {
  music_sync(MUSIC::B5_, time);
  music_sync(MUSIC::E6_, 2 * time);
}

void UserInterface::hello_exia() {
  int time = 120;
  music_sync(MUSIC::A6_, time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  music_sync(MUSIC::A6_, time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  music_sync(MUSIC::A6_, time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  music_sync(MUSIC::A6_, 3 * time);
  vTaskDelay(time / portTICK_PERIOD_MS);
  music_sync(MUSIC::A6_, time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  music_sync(MUSIC::A6_, time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  music_sync(MUSIC::A6_, time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  music_sync(MUSIC::A6_, time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  music_sync(MUSIC::C7_, 1.5 * time);
  vTaskDelay(time / 3 / portTICK_PERIOD_MS);
  music_sync(MUSIC::G6_, 2 * time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  music_sync(MUSIC::F6_, 2 * time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  music_sync(MUSIC::G6_, 2 * time);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
}

void UserInterface::LED_on_off(gpio_num_t gpio_num, int state) {
  // gpio_set_level(gpio_num, state);
  const int num = (int)gpio_num;
  if (num < 32) {
    if (state) {
      GPIO.out_w1ts = BIT(num);
    } else {
      GPIO.out_w1tc = BIT(num);
    }
  } else {
    if (state) {
      GPIO.out1_w1ts.val = BIT(num - 32);
    } else {
      GPIO.out1_w1tc.val = BIT(num - 32);
    }
  }
}

void UserInterface::LED_bit(int b0, int b1, int b2, int b3, int b4) {
  LED_on_off(LED1, (b0 == 1));
  LED_on_off(LED4, (b1 == 1));
  LED_on_off(LED2, (b2 == 1));
  LED_on_off(LED3, (b3 == 1));
  LED_on_off(LED5, (b4 == 1));
}
void UserInterface::LED_otherwise(int byte, int state) {}
void UserInterface::LED(int byte, int state) {}
void UserInterface::LED_on(int byte) {}

void UserInterface::LED_off(int byte) {}
void UserInterface::LED_off_all() {
  const int state = 0;
  LED_on_off(LED1, state);
  LED_on_off(LED2, state);
  LED_on_off(LED3, state);
  LED_on_off(LED4, state);
  LED_on_off(LED5, state);
}
void UserInterface::LED_on_all() {
  const int state = 1;
  LED_on_off(LED1, state);
  LED_on_off(LED2, state);
  LED_on_off(LED3, state);
  LED_on_off(LED4, state);
  LED_on_off(LED5, state);
}

TurnDirection UserInterface::select_direction() {
  TurnDirection td = TurnDirection::None;
  bool b = true;
  while (1) {
    if (sensing_result->ego.v_r > ENC_OPE_V_R_TH) {
      music_sync(MUSIC::G6_, 75);
      td = TurnDirection::Right;
      LED_bit(1, 0, 0, 0, 0);
    }
    if (sensing_result->ego.v_l > ENC_OPE_V_R_TH) {
      music_sync(MUSIC::C6_, 75);
      td = TurnDirection::Left;
      LED_bit(0, 0, 0, 0, 1);
    }

    if (td != TurnDirection::None) {
      if (button_state_hold()) {
        coin(80);
        return td;
      }
    } else {
      LED_bit((int)b, 0, 0, 0, (int)b);
      b = b ? false : true;
    }
    vTaskDelay(25.0 / portTICK_PERIOD_MS);
  }
}

TurnDirection UserInterface::select_direction2() {
  return select_direction(); //
}
void UserInterface::error() {
  int time = 120;
  for (int i = 0; i < 4; i++)
    music_sync(MUSIC::C4_, time);
}