# Robot

Packages `ROS` pour interfacer et contrôler le robot :
- [robot_bringup](./robot_bringup) : *launch files* pour démarrer le projet.
- [robot_control](./robot_control) : Outils pour envoyer des commandes de vitesse.
- [robot_description](./robot_description) : Description du robot (`URDF` par `xacro`).
- [robot_hardware](./robot_hardware) : Interface avec la `STM32` pour récupérer les informations des *capteurs* et agir sur les *actionneurs*.
- [robot_navigation](./robot_navigation) : Utilise la `Navigation Stack` de *ROS* pour déplacer le robot sur une *map*.
