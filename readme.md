READ ME

auteurs : DRENO Grégoire / FABRE Sébastien

# Fonctionnement
## couloir normal
```> ros2 run pj_ROS couloir.launch.py```
Lance la simulation gazebo et le noeuds de pilotage en mode couloir. Les autres noeuds de pilotage sont éteint.

## couloir à l'envert
```> ros2 run pj_ROS couloir_reverse.launch.py```
Lance la simulation gazebo et le noeuds de pilotage en mode couloir. Le robot part de la fin pour plus de fun. Les autres noeuds de pilotages sont éteint.

## Ligne
```> ros2 run pj_ROS Ligne.launch.py```
Lance la simulation gazebo et le noeuds de pilotage en mode ligne. La position du robot est modifier pour démarrer directement devant la ligne. Les autres noeuds de pilotage sont éteint.

## Balle
```> ros2 run pj_ROS Balle.launch.py```

## Tout
```> ros2 run pj_ROS Tout.launch.py```
Lance la simulation gazebo avec le robot au début de l'épreuve 1. Tout les noeuds sont actifs et le robot execute toutes les épreuves dans l'ordre.

# Description
Ce package est basé sur le challenge décrit dans le package challenge_robot et ne peut être utilisé sans. Le package challenge_robot n’a pas été modifié par rapport à sa version disponible sur Savoir.

Ubuntu 22.04 LTS et ROS 2 Humble
L’arborescence du package pj_ROS est le suivant :
.
├── LICENSE
├── README.md
├── launch
│   ├── Balle.launch.py
│   ├── Ligne.launch.py
│   ├── Tout.launch.py
│   ├── couloir.launch.py
│   ├── couloir_reverse.launch.py
│   ├── detecCible.launch.py
│   └── project.launch.py
├── package.xml
├── pj_ROS
│   ├── Balle.py
│   ├── Cible.py
│   ├── Couloir.py
│   ├── Ligne.py
│   ├── __init__.py
│   ├── __pycache__
│   │   ├── Arbitre.cpython-310.pyc
│   │   ├── Balle.cpython-310.pyc
│   │   ├── Cible.cpython-310.pyc
│   │   ├── Couloir.cpython-310.pyc
│   │   ├── Ligne.cpython-310.pyc
│   │   ├── Mapper.cpython-310.pyc
│   │   ├── Testgreg.cpython-310.pyc
│   │   ├── __init__.cpython-310.pyc
│   │   ├── cv_plot.cpython-310.pyc
│   │   ├── estimate_pose.cpython-310.pyc
│   │   └── smallest_enclosing_circle.cpython-310.pyc
│   ├── estimate_pose.py
│   └── smallest_enclosing_circle.py
├── resource
│   └── pj_ROS
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py

Chaque launch file permet de lancer une résolution d’un challenge précis mais Tout.launch.py permet de lancer une simulation Gazebo où le robot arrive à résoudre de façon robuste le parcours du labyrinthe + le suivi de ligne + la mise en veille si un obstacle se trouve sur la ligne.


## Description des nœuds Python

Ce package contient plusieurs nœuds ROS 2 autonomes, chacun dédié à un sous-challenge du projet.

### `Cible.py` — Suivi et localisation de cible bleue

Ce nœud reçoit le flux caméra (`/image_raw/compressed`) et la calibration caméra (`/camera_info`), puis détecte une cible bleue et estime sa position en coordonnées 3D. Il publie la position relative cible-robot sur un topic `Vector3` (distance, angle, confiance) via `/Cible_loc`.

Points clés :
- Détection HSV + fermeture morphologique
- Conversion pixel → 3D via estimatePose
- Publication continue sur `/Cible_loc`

### `Couloir.py` — Navigation dans l'épreuve 1

Ce nœud s'appuie sur les données LIDAR (`/scan`) pour faire suivre au robot un couloir par un comportement de répulsion inertielle.

Points clés :
- Transformation LIDAR → moments de rotation
- Intégration dynamique `I.w. = Mz - f.w²`

Le Noeud (`Couloir.py`) est abonné au lidar. Chaque message du lidar contient 360 valeurs, corespondant aux distances des objets autours du robot (1 distance / degré).
L'idée de l'algorithme est que le robot doit chercher à se diriger vers la direction ou il y a le plus de distance (donc là ou il n'y a pas de mur), et à s'éloigner des directions où la distance est faible (éviter les murs)). On n'ultilisera que les 180 rayons qui pointent vers l'avant du robot. Seul la situation vers laquel on se dirige n'a d'importance (on s'éloigne de ce qu'il y a derrière donc on ne risque pas de rentrer dedans).
On considère chaque rayon du lidar comme un ressort. Une raideur et une longeur à vide est donc défini. Si la distance est plus grande que la longueur à vide, la force tire le robot dans cette direction (*attiré par ce qui est loin*), si la distance est plus courte il est repoussé (*repoussé par ce qui est proche*). On convertie ensuite cette force linéaire en couple en projetant la force sur l'axe y (axe gauche droite du robot). On a donc des forces qui s'appliquent sur notre robot, on calcule le couple résultant (on note que la physique ne marche pas comme ça dans la vrais vie, les forces sont toutes dans l'axe du robot donc le couple est vide. Mais l'idée de l'algo est de faire cette projection qui n'a aucun sens physique car elle a un sens dans notre cas).
On simule ensuite une inertie et un frottement (proportionelle au carré de la vitesse), ce qui agit comme un filtre passe bas et lisse les déplacements du robot.
Le problème de cette méthode est que le robot est infiniment attiré par les rayons infini. On vient réduire artificiellement la taille des rayons infini pour résoudre ce problème.
On viens aussi bloquer la vitesse de rotation maximum du robot afin de limiter les risques d'emballement du système.

Originalement, le même principe était utiliser pour moduler la vitesse linéaire du robot (on projette sur x et on a la force qui accélère/freine le robot). Nous nous sommes cependant aperçu que même à la vitesse maximum, le robot réussissait l'épreuve.


### `Ligne.py` — Suivi de ligne verte

Ce nœud lit l'image caméra (`/image_raw/compressed`) et détecte une ligne verte. Il en calcule le centroïde, centre la direction du robot à l’aide d’un correcteur proportionnel et publie la commande `Twist` sur `/cmd_vel`.

### `Balle.py` — Destruction de la balle jaune

Ce nœud détecte une balle jaune dans l’image. Il utilise la position 3D pré-calculée de la cible (via `/Cible_loc`) pour ne chercher la balle que si la cible est déjà identifiée.

Points clés :
- Détection couleur HSV avec cercle englobant
- Estimation 3D via `estimatePose`

### `Tout.launch.py` — Exécution enchainee

Ce fichier de lancement coordonne automatiquement les différentes étapes :
1. Navigation dans le couloir
2. Suivi de ligne
3. Détection de cible
4. **Destruction** de la balle

Chaque nœud s’éteint en publiant une commande sur `/vision_trigger`, permettant une transition fluide entre modules.

## Utilitaires : Vision et géométrie

Deux fichiers fournissent des fonctions auxiliaires utilisées dans les nœuds de vision :

### `estimate_pose.py` — Projection 2D → 3D

Fonction : `estimatePose(uvs, KRTi, camPos, height)`

But : à partir d’un ensemble de coordonnées image `uvs`, cette fonction reconstruit leur position 3D supposée sur le sol (z = 0), en utilisant la pseudo-inverse de la matrice de projection calibrée `KRTi` et la position caméra `camPos`.

Principe :
- Multiplication par la pseudo-inverse KRTi pour obtenir des vecteurs directeurs
- Intersection avec le plan z = height

Dépend aussi de :
- `tilde(P)` : homogénéisation
- `matvec(A, B)` : application de la matrice `A` à chaque vecteur `b` de `B`

### `smallest_enclosing_circle.py` — Cercle minimum englobant

Fonction : `make_circle(points)`

But : retourne le plus petit cercle (centre, rayon) englobant un ensemble de points 2D. Utilisé pour approximer la position d’une cible ou balle dans l’image, en réduisant l’influence des points aberrants.

Algorithme : implémentation directe du **Welzl's algorithm**.

Sortie : `(x, y, r)` = centre + rayon du cercle minimal.
