#include "robot/Encoder.h"

namespace robot {

Encoder::Encoder(EncoderDef enc_def) :
    htim_(enc_def.htim), resolution_(enc_def.resolution), tim_reload_(enc_def.tim_reload)
{
}

void Encoder::init() {
  if (HAL_TIM_Encoder_Start(&htim_, TIM_CHANNEL_1) != HAL_OK) {
    Init_Error_Handler();
  }
  if (HAL_TIM_Encoder_Start(&htim_, TIM_CHANNEL_2) != HAL_OK) {
    Init_Error_Handler();
  }

  resetTimerCount();
}

void Encoder::update(float dt_s) {
  int count = getTimerCount();
  resetTimerCount();

  dticks_ = count - tim_reload_;
  ticks_ += dticks_;
  dticks_s_ = dticks_ / dt_s;
}

inline uint32_t Encoder::getTimerCount() {
  return htim_.Instance->CNT;
}

inline void Encoder::resetTimerCount() {
  __HAL_TIM_SET_COUNTER(&htim_, tim_reload_);
}

} /* namespace robot */
