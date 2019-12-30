# Description

Description du robot pour pouvoir le visualiser avec `RViz`.

Le *launch file* `description.launch` permet d'afficher le modèle du robot dans `RViz`:
```sh
$ roslaunch robot_description description.launch
```

A l'inverse, le *launch file* `robot_description.launch` contient uniquement les commandes pour charger le modèle du robot (pour éviter l'exécution de plusieurs `joint_state_publisher`).