# pj_ROS
projet ROS2 M2 ENSAM
FABRE Sébastient
DRÉNO Grégoire


# Utilisation
## Épreuve du couloir
### épreuve normal
```> ros2 run pj_ROS couloir.launch.py```
Lance la simulation gazebo et le noeuds de pilotage en mode couloir. Les autres noeuds de pilotage sont éteint.
### épreuve à l'envert
```> ros2 run pj_ROS couloir_reverse.launch.py```
Lance la simulation gazebo et le noeuds de pilotage en mode couloir. Le robot part de la fin pour plus de fun. Les autres noeuds de pilotages sont éteint.

## Épreuve de la ligne
```> ros2 run pj_ROS ligne.launch.py```
Lance la simulation gazebo et le noeuds de pilotage en mode ligne. La position du robot est modifier pour démarrer directement devant la ligne. Les autres noeuds de pilotage sont éteint.
## Épreuve de la balle
```> ros2 run pj_ROS balle.launch.py```
TODO

## Trois pour le prix d'un
```> ros2 run pj_ROS total.launch.py```
Lance la simulation gazebo avec le robot au début de l'épreuve 1. Tout les noeuds sont actifs et le robot execute toutes les épreuves dans l'ordre.
