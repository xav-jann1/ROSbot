#ifndef ROBOT_ENCODER_H_
#define ROBOT_ENCODER_H_

#include "utils.h"
#include "stm32f4xx_hal.h"

#include <string>
#include "ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Empty.h"

namespace robot {

typedef struct {
  TIM_HandleTypeDef &htim;
  uint16_t resolution;
  uint16_t tim_reload;
} EncoderDef;

class Encoder {
public:
  Encoder(EncoderDef, std::string = "encoder");

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

  // Publisher:
  void addPublishers(ros::NodeHandle& nh);
  void publishData();

  // Subscriber
  void addSubscriber(ros::NodeHandle& nh);

private:
  // Paramètres:
  TIM_HandleTypeDef &htim_;
  int resolution_;  // ticks / tour
  uint16_t tim_reload_;

  // Noms des topics des publishers:
  // (besoin d'être sauvegardé dans l'objet pour fonctionner)
  // (le nom du topic ne peut pas être écrit directement lors de la création des subscribers)
  std::string pos_name_;
  std::string vel_name_;
  std::string ticks_name_;
  std::string dticks_name_;

  // Publishers:
  ros::Publisher pos_pub_;
  ros::Publisher vel_pub_;
  ros::Publisher ticks_pub_;
  ros::Publisher dticks_pub_;

  // Publishers' message:
  std_msgs::Float32 pos_msg_;
  std_msgs::Float32 vel_msg_;
  std_msgs::Int32 ticks_msg_;
  std_msgs::Int32 dticks_msg_;

  // Subscriber:
  std::string reset_name_;
  ros::Subscriber<std_msgs::Empty, robot::Encoder> reset_sub_;
  void reset_cb(const std_msgs::Empty &msg) { ticks_ = dticks_ = dticks_s_ = 0; }

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
