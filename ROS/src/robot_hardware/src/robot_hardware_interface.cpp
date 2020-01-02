#include <robot_hardware/robot_hardware_interface.h>
#include <sstream>

using namespace hardware_interface;

namespace robot_hardware_interface {
RobotHardwareInterface::RobotHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
  init();

  // Paramètre "simulate_joints":
  nh_.param("/robot/hardware_interface/simulate_joints", simulate_joints_, false);
  if (simulate_joints_) ROS_INFO("! Simulation des joints actif");

  // Paramètre "write_joints":
  nh_.param("/robot/hardware_interface/write_joints", write_joints_, true);
  write_joints_service_ = nh_.advertiseService("/robot/controller/write_joints", &RobotHardwareInterface::write_joints_srv, this);
  if (!write_joints_) ROS_INFO("! Envois des commandes aux joints desactivees");

  controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
  nh_.param("/robot/hardware_interface/loop_hz", loop_hz_, 0.1);
  ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);
  non_realtime_loop_ = nh_.createTimer(update_freq, &RobotHardwareInterface::update, this);
}

RobotHardwareInterface::~RobotHardwareInterface() {}

void RobotHardwareInterface::init() {
  ROS_INFO("Ajout d'hardware interfaces pour les joints...");

  // Crée le subscriber de tous les joints:
  std::string sub_name = "/robot/controller/joints_data";
  joints_data_sub_ = nh_.subscribe(sub_name, 1, &RobotHardwareInterface::joints_data_callback, this);
  ROS_INFO("> ajout du subscriber: '%s'", sub_name.c_str());

  // Crée le publisher de tous les joints:
  std::string pub_name = "/robot/controller/joints_command";
  joints_command_publisher_ = nh_.advertise<std_msgs::Float64MultiArray>(pub_name, 1);
  ROS_INFO("> ajout du publisher: '%s'", pub_name.c_str());

  // Récupère les noms des joints:
  nh_.getParam("/robot/hardware_interface/joints", joint_names_);
  num_joints_ = joint_names_.size() + 1;  // '+ 1' car DiffDriveController
  ROS_INFO("- %d joints a ajouter:", num_joints_);

  // Dimensionne les vectors pour contenir les valeurs de chaque joint:
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  joint_command_.resize(num_joints_);
  joint_velocity_command_.resize(num_joints_);
  joint_effort_command_.resize(num_joints_);
  joint_type_.resize(num_joints_);
  joint_command_publisher_.resize(num_joints_);

  // Liste des types de joint:
  joint_types_ = {"position_controllers/JointPositionController", "velocity_controllers/JointVelocityController", "effort_controllers/JointPositionController"};
  bool registerInterfaces[3];  // Pour connaître les interfaces à ajouter: [position, velocity, effort]
  int i_joints = 0;            // Compteur des joints ajoutés

  // Initialise les controllers pour chaque joint:
  for (int i = 0; i < joint_names_.size(); ++i) {
    ROS_INFO(" - Controller %d:", i);
    ROS_INFO("  - name: '%s'", joint_names_[i].c_str());

    // Récupère le type du controller:
    std::string controller_type;
    std::string controller_path = "/robot/controller/" + joint_names_[i];
    nh_.getParam(controller_path + "/type", controller_type);
    ROS_INFO("  - type: '%s'", controller_type.c_str());

    /** Ajoute le(s) joint(s) du controller en fonction de son type **/
    std::string joint_name;
    int i_type = -1;

    // Si DiffDriveController:
    if (controller_type == "diff_drive_controller/DiffDriveController") {
      // Ajoute la roue gauche:
      nh_.getParam(controller_path + "/left_wheel", joint_name);
      addJoint(i_joints++, joint_name, "velocity_controllers/JointVelocityController");

      // Ajoute la roue droite:
      nh_.getParam(controller_path + "/right_wheel", joint_name);
      i_type = addJoint(i_joints, joint_name, "velocity_controllers/JointVelocityController");
    }
    // Sinon:
    else {
      // Ajoute directement le controller:
      nh_.getParam(controller_path + "/joint", joint_name);
      i_type = addJoint(i_joints, joint_name, controller_type);
    }

    // Enregistre pour que l'interface soit utilisée:
    registerInterfaces[i_type] = true;
    i_type = -1;

    i_joints++;
  }

  // Enregistre les interfaces utilisées:
  registerInterface(&joint_state_interface_);
  if (registerInterfaces[0]) registerInterface(&position_joint_interface_);
  if (registerInterfaces[1]) registerInterface(&velocity_joint_interface_);
  if (registerInterfaces[2]) registerInterface(&effort_joint_interface_);
}

int RobotHardwareInterface::addJoint(int i, std::string j_name, std::string joint_type) {
  const char* joint_name = j_name.c_str();

  // Crée l'interface joint state:
  JointStateHandle jointStateHandle(joint_name, &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
  joint_state_interface_.registerHandle(jointStateHandle);

  // Crée l'interface de commande:
  JointHandle jointCommandHandle(jointStateHandle, &joint_command_[i]);

  /** Relie l'interface de commande à la bonne interface de contrôle **/
  // Cherche l'indice de l'interface du joint ajouté:
  int i_type = -1;
  for (int i = 0; i < 3; i++)
    if (joint_type == joint_types_[i]) i_type = i;

  // Ajoute l'interface:
  if (i_type == 0)
    position_joint_interface_.registerHandle(jointCommandHandle);
  else if (i_type == 1)
    velocity_joint_interface_.registerHandle(jointCommandHandle);
  else if (i_type == 2)
    effort_joint_interface_.registerHandle(jointCommandHandle);

  ROS_INFO("  > ajout du joint '%s' dans interface '%s'", joint_name, joint_types_[i_type].c_str());

  // Enregistre l'interface utilisée par le joint:
  joint_type_[i] = i_type;

  // Crée le publisher du joint:
  const std::string pub_name = "/robot/controller/" + j_name + "/command";
  joint_command_publisher_[i] = nh_.advertise<std_msgs::Float64>(pub_name, 1);
  ROS_INFO("  > ajout du publisher: '%s'", pub_name.c_str());

  return i_type;
}

void RobotHardwareInterface::update(const ros::TimerEvent& e) {
  elapsed_time_ = ros::Duration(e.current_real - e.last_real);
  read();
  controller_manager_->update(ros::Time::now(), elapsed_time_);
  if (write_joints_) write(elapsed_time_);
  ros::spinOnce();
}

void RobotHardwareInterface::read() {
  /**
   * Read effectué par la fonction joints_data_callback() qui récupère les messages
   * reçus pour le topic "/robot/controller/joints_data"
   */

  // Si la simulation des joints est activés:
  if (simulate_joints_) {
    // Simule chaque joint en boucle fermée à retour unitaire:
    for (int i = 0; i < num_joints_; i++) {
      // Position:
      if (joint_type_[i] == 0) {
        double p = joint_command_[i];
        joint_velocity_[i] = joint_position_[i] - p;
        joint_position_[i] = p;
      }
      // Vitesse:
      else if (joint_type_[i] == 1) {
        double v = joint_command_[i];
        joint_position_[i] += v / loop_hz_;
        joint_velocity_[i] = v;
      }
    }
  }
}

void RobotHardwareInterface::write(ros::Duration elapsed_time) {
  /**
   * 2 méthodes possibles pour l'envoi des commandes:
   *  - un message par joint : "/robot/controller/<joint_name>/command"
   *  - un message pour tous les joints : "/robot/controller/joints_command"
   *
   * Tests avec loop_hz_ = 50 Hz, pour 2 joints:
   *  - Méthode 1 :  800 B/s
   *  - Méthode 2 : 2060 B/s
   */

  /** Méthode 1 : un message par joint **/
  for (int i = 0; i < num_joints_; i++) {
    joint_command_publish(i, joint_command_[i]);
  }

  /** Méthode 2 : un message pour tous les joints **/

  // Crée le message avec les commandes de tous les joints:
  std_msgs::Float64MultiArray msg;
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim[0].size = joint_command_.size();
  msg.layout.dim[0].stride = 1;
  msg.layout.dim[0].label = "i";

  // Ajoute les commandes:
  msg.data.clear();
  msg.data.insert(msg.data.end(), joint_command_.begin(), joint_command_.end());

  // Publie les commandes:
  joints_command_publisher_.publish(msg);
}

// Envoie une commande à un joint:
void RobotHardwareInterface::joint_command_publish(int i_joint, float cmd) {
  // Crée le message:
  std_msgs::Float64 msg;
  msg.data = cmd;

  // Publie la commande:
  joint_command_publisher_[i_joint].publish(msg);
}

// Callback du subscriber des données des joints:
void RobotHardwareInterface::joints_data_callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  // Récupère les données de chaque joint:
  for (int i = 0; i < num_joints_; i++) {
    joint_position_[i] = msg->data[i * 3];
    joint_velocity_[i] = msg->data[i * 3 + 1];
    joint_effort_[i] = msg->data[i * 3 + 2];
  }
}

// Callback du service pour modifier le paramètre "write_joints":
bool RobotHardwareInterface::write_joints_srv(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
  write_joints_ = req.data;
  res.success = true;
  res.message = req.data ? "true" : "false";
  return true;
}
}  // namespace robot_hardware_interface