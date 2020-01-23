# Navigation

Ce package permet d'utiliser la [`Navigation Stack`](http://wiki.ros.org/navigation) de *ROS* pour déplacer le robot sur une map en évitant des obstacles.

## Utilisation

Etapes pour utiliser la navigation pour ce projet :

- Lancer l'`hardware_interface` :
  - Avec le *robot réel* (sur la *Raspberry*) :
    ```sh
    $ roslaunch robot_hardware robot_hardware.launch
    ```
  - En *simulation* :
    ```sh
    $ roslaunch robot_hardware robot_hardware_simulation.launch
    ```

- Lancer la navigation (sur la *Raspberry* pour le *robot réel*):
  ```sh
  $ roslaunch robot_navigation move_base.launch
  ```

- Si avec le *robot réel*, se connecter au *Master ROS* de la *Raspberry* :
  ```sh
  $ export ROS_MASTER_URI=http://10.42.0.1:11311
  ```

- Ouvrir `RViz` (pas sur la *Raspberry*):
  ```sh
  $ rviz -d config/navigation.rviz
  ```

- Utilser l'outil `2D Nav Goal` de `RViz` pour demander de déplacer le robot dans une zone de la map.
- Observer le chemin généré, et les actions effectuées par le robot.


## Configuration

Toutes les configurations se trouvent dans les fichiers du dossier [`config`](./config).


## Ressources

- Tutoriel pour configurer la `Navigation Stack` : [Setup and Configuration of the Navigation Stack on a Robot](http://wiki.ros.org/navigation/Tutorials/RobotSetup).