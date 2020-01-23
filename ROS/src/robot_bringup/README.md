# Bring up

Ce package contient des *launch files* pour tester le projet *avec* ou *sans* le robot.

## Utilisation

Pour lancer l'`hardware_interface`, la *navigation*, `RViz` et le *contrôle avec la souris*, exécuter l'une des commandes suivantes :
- Avec le robot :
  ```sh
  $ roslaunch robot_bringup robot_bringup.launch
  ```
- Sans le robot :
  ```sh
  $ roslaunch robot_bringup simulation_bringup.launch
  ```

`RViz` s'ouvre ainsi pour visualiser les déplacements du robot.

Il est possible d'**envoyer des commandes en vitesse** grâce à une autre fenêtre : **cliquer et glisser** dans une direction pour faire **avancer et tourner** le robot.

Enfin, utilser l'outil `2D Nav Goal` de `RViz` pour demander de déplacer le robot dans une zone de la map.
