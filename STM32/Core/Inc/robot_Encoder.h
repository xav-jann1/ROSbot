#ifndef ROBOT_ENCODER_H_
#define ROBOT_ENCODER_H_

#include "robot.h"
#include "stm32f4xx_hal.h"

namespace robot {

typedef struct {
  TIM_HandleTypeDef &htim;
  uint16_t resolution;
  uint16_t tim_reload;
} EncoderDef;

class Encoder {
public:
  Encoder(EncoderDef);

  void init();
  void update(float);

  // Getters :
  int getTicks() const {
    return ticks_;
  }
  int getDticks() const {
    return dticks_;
  }
  float getPosition() const {
    return M_2PI * ticks_ / resolution_;  // rad
  }
  float getVelocity() const {
    return M_2PI * dticks_s_ / resolution_;  // rad/s
  }

private:
  // Paramètres:
  TIM_HandleTypeDef &htim_;
  int resolution_;  // ticks / tour
  uint16_t tim_reload_;

  // Encoder:
  int ticks_;
  int dticks_;
  float dticks_s_;

  // Timer:
  uint32_t getTimerCount();
  void resetTimerCount();
};

} /* namespace robot */

#endif /* ROBOT_ENCODER_H_ */
