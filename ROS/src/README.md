# Robot

Packages `ROS` pour interfacer et contrôler le robot :
- [robot_description](./robot_description) : Description du robot (`URDF` par `xacro`).
- [robot_hardware](./robot_hardware) : Interface avec la `STM32` pour récupérer les informations des *capteurs* et agir sur les *actionneurs*.
- [robot_control](./robot_control) : `DiffDriveController` `ros_control`