# **Projet ROS**
projet ROS2 M2 ENSAM
FABRE Sébastient
DRÉNO Grégoire

# Épreuve du couloir
## Fonctionnement
Le Noeud (`Couloir.py`) est abonné au lidar. Chaque message du lidar contient 360 valeurs, corespondant aux distances des objets autours du robot (1 distance / degré).
L'idée de l'algorithme est que le robot doit chercher à se diriger vers la direction ou il y a le plus de distance (donc là ou il n'y a pas de mur), et à s'éloigner des directions où la distance est faible (éviter les murs)). On n'ultilisera que les 180 rayons qui pointent vers l'avant du robot. Seul la situation vers laquel on se dirige n'a d'importance (on s'éloigne de ce qu'il y a derrière donc on ne risque pas de rentrer dedans).
On considère chaque rayon du lidar comme un ressort. Une raideur et une longeur à vide est donc défini. Si la distance est plus grande que la longueur à vide, la force tire le robot dans cette direction (*attiré par ce qui est loin*), si la distance est plus courte il est repoussé (*repoussé par ce qui est proche*). On convertie ensuite cette force linéaire en couple en projetant la force sur l'axe y (axe gauche droite du robot). On a donc des forces qui s'appliquent sur notre robot, on calcule le couple résultant (on note que la physique ne marche pas comme ça dans la vrais vie, les forces sont toutes dans l'axe du robot donc le couple est vide. Mais l'idée de l'algo est de faire cette projection qui n'a aucun sens physique car elle a un sens dans notre cas).
On simule ensuite une inertie et un frottement (proportionelle au carré de la vitesse), ce qui agit comme un filtre passe bas et lisse les déplacements du robot.
Le problème de cette méthode est que le robot est infiniment attiré par les rayons infini. On vient réduire artificiellement la taille des rayons infini pour résoudre ce problème.
On viens aussi bloquer la vitesse de rotation maximum du robot afin de limiter les risques d'emballement du système.

Originalement, le même principe était utiliser pour moduler la vitesse linéaire du robot (on projette sur x et on a la force qui accélère/freine le robot). Nous nous sommes cependant aperçu que même à la vitesse maximum, le robot réussissait l'épreuve.
## Utilisation
### épreuve normal
```> ros2 run pj_ROS couloir.launch.py```
Lance la simulation gazebo et le noeuds de pilotage en mode couloir. Les autres noeuds de pilotage sont éteint.
### épreuve à l'envert
```> ros2 run pj_ROS couloir_reverse.launch.py```
Lance la simulation gazebo et le noeuds de pilotage en mode couloir. Le robot part de la fin pour plus de fun. Les autres noeuds de pilotages sont éteint.

# Épreuve de la ligne
## Utilisation
```> ros2 run pj_ROS ligne.launch.py```
Lance la simulation gazebo et le noeuds de pilotage en mode ligne. La position du robot est modifier pour démarrer directement devant la ligne. Les autres noeuds de pilotage sont éteint.

# Épreuve de la balle
```> ros2 run pj_ROS balle.launch.py```
TODO

# Trois pour le prix d'un
```> ros2 run pj_ROS total.launch.py```
Lance la simulation gazebo avec le robot au début de l'épreuve 1. Tout les noeuds sont actifs et le robot execute toutes les épreuves dans l'ordre.
