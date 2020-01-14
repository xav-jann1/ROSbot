/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifndef ROBOT_PID_H
#define ROBOT_PID_H

#include <string>
#include "ros.h"
#include "std_msgs/Float32.h"

namespace robot {

typedef struct {
  float p, i, d, i_max, i_min;
} PidDef;

/***************************************************/
/***************************************************/

class Pid
{
public:

  Pid(double P = 0.0, double I = 0.0, double D = 0.0, double I1 = 0.0, double I2 = -0.0, std::string name = "pid");

  Pid(PidDef pid_def, std::string name = "pid");

  ~Pid();

  double updatePid(double p_error, float dt);

  void initPid(double P, double I, double D, double I1, double I2);
  
  bool initParam(const std::string& prefix);

  void addSubscribers(ros::NodeHandle&);

  void addPublishers(ros::NodeHandle&);

  void publishPidValues();

  void reset();

  void setCurrentCmd(double cmd);

  double getCurrentCmd();

  void getCurrentPIDErrors(double *pe, double *ie, double *de);

  void setGains(double P, double I, double D, double i_max, double i_min);

  void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

  double updatePid(double error, double error_dot, float dt);

  Pid &operator =(const Pid& p)
  {
    if (this == &p)
      return *this;

    p_gain_ = p.p_gain_;
    i_gain_ = p.i_gain_;
    d_gain_ = p.d_gain_;
    i_max_ = p.i_max_;
    i_min_ = p.i_min_;

    p_error_last_ = p_error_ = i_error_ = d_error_ = cmd_ = 0.0;
    return *this;
  }

private:
  double p_gain_;
  double i_gain_;
  double d_gain_;
  double i_max_;
  double i_min_;

  double p_error_last_; 
  double p_error_; 
  double d_error_; 
  double i_error_; 
  double cmd_;

  // Subscribers' name:
  // (besoin d'être sauvegardé dans l'objet pour fonctionner)
  // (le nom du topic ne doit pas être écrit directement lors de la création des topics)
  std::string set_p_name_;
  std::string set_i_name_;
  std::string set_d_name_;
  std::string set_i_max_name_;
  std::string set_i_min_name_;

  // Subscribers:
  ros::Subscriber<std_msgs::Float32, robot::Pid> set_p_sub_;
  ros::Subscriber<std_msgs::Float32, robot::Pid> set_i_sub_;
  ros::Subscriber<std_msgs::Float32, robot::Pid> set_d_sub_;
  ros::Subscriber<std_msgs::Float32, robot::Pid> set_i_max_sub_;
  ros::Subscriber<std_msgs::Float32, robot::Pid> set_i_min_sub_;

  // Subscribers' Callback:
  void set_p_cb(const std_msgs::Float32 &msg) { p_gain_ = msg.data; }
  void set_i_cb(const std_msgs::Float32 &msg) { i_gain_ = msg.data; }
  void set_d_cb(const std_msgs::Float32 &msg) { d_gain_ = msg.data; }
  void set_i_max_cb(const std_msgs::Float32 &msg) { i_max_ = msg.data; }
  void set_i_min_cb(const std_msgs::Float32 &msg) { i_min_ = msg.data; }

  // Publishers" name:
  std::string get_p_name_;
  std::string get_i_name_;
  std::string get_d_name_;
  std::string get_i_max_name_;
  std::string get_i_min_name_;

  // Publishers:
  ros::Publisher get_p_pub_;
  ros::Publisher get_i_pub_;
  ros::Publisher get_d_pub_;
  ros::Publisher get_i_max_pub_;
  ros::Publisher get_i_min_pub_;

  // Publishers' message:
  std_msgs::Float32 get_p_msg_;
  std_msgs::Float32 get_i_msg_;
  std_msgs::Float32 get_d_msg_;
  std_msgs::Float32 get_i_max_msg_;
  std_msgs::Float32 get_i_min_msg_;
};

}

#endif