# Modelisation d'un bras robotique

Projet ROS 2 pour modeliser un bras robotique industriel en URDF/Xacro, le
visualiser dans RViz 2 et fournir une base de configuration MoveIt 2.

## Structure du depot

- `urdf/` : description Xacro du robot (`arm.urdf.xacro`), macros et materiaux.
- `meshes/` : meshes STL references par les visuels.
- `launch/` : lancement RViz 2 pour la description du robot.
- `rviz/` : configuration RViz de visualisation.
- `config/` : fichiers de configuration simples pour le package de description.
- `modelisation_bras_robotique_moveit_config/` : package MoveIt 2.

## Choix URDF vs Xacro

Le projet utilise une architecture Xacro modulaire plutot qu'un unique fichier
URDF. Le critere principal est la repetition : chaque lien possede un visuel,
une collision simplifiee et une inertie estimee. Les macros permettent donc de
factoriser les materiaux, les inerties et les chemins de meshes.

Avantages :

- description plus lisible et plus facile a maintenir ;
- reutilisation des macros pour les inerties et les visuels ;
- possibilite d'ajouter des variantes du robot sans dupliquer tout le XML.

Inconvenients :

- necessite l'outil `xacro` avant l'utilisation directe par certains outils ;
- une erreur de macro peut etre moins evidente qu'une erreur URDF simple.

Un fichier URDF unique aurait ete suffisant pour un modele tres court, sans
repetition et sans besoin d'evolution. Ici, Xacro est plus adapte au livrable.

## Description du robot

Le robot est un bras manipulateur a 3 degres de liberte avec une pince a 1
degre de liberte. Le doigt gauche mime le doigt droit avec un multiplicateur
negatif, comme demande dans l'enonce.

Chaine cinematique :

```text
world -> base_link -> base_plate -> forward_drive_arm -> horizontal_arm -> claw_support -> gripper_right
                                                                                         -> gripper_left
```

Le robot contient les liens suivants :

- `world`
- `base_link`
- `base_plate`
- `forward_drive_arm`
- `horizontal_arm`
- `claw_support`
- `gripper_right`
- `gripper_left`

Les joints principaux sont :

- `joint1` : rotation de la base autour de `z`.
- `joint2` : rotation de l'epaule autour de `x`.
- `joint3` : rotation du coude autour de `x`.
- `joint_claw` : liaison fixe entre le bras horizontal et le support de pince.
- `joint4` : rotation du doigt droit de la pince autour de `z`.
- `joint5` : rotation du doigt gauche, avec `<mimic joint="joint4" multiplier="-1"/>`.

Les origines des visuels reprennent les valeurs fournies dans l'enonce et les
meshes sont affiches avec `scale="0.01 0.01 0.01"`.

## Hypotheses

Les meshes STL fournis par le TP sont references avec des URI
`package://modelisation_bras_robotique/meshes/<nom>.STL`. Les noms conservent
l'extension `.STL` en majuscules pour rester compatibles avec Linux, ou la casse
des fichiers est significative.

Les collisions sont simplifiees avec des boites et cylindres. Les masses et les
inerties sont estimees avec des dimensions rectangulaires coherentes. Les
masses restent volontairement dans une plage pedagogique de `0.1 kg` a `1.0 kg`
:

- base lourde pour stabiliser le robot ;
- bras plus leger que la base ;
- pince legere avec deux doigts couples par mimic.

Les limites articulaires sont choisies pour eviter des mouvements excessifs :

- `joint1` : `[-pi, pi]`
- `joint2` et `joint3` : `[-pi/2, pi/2]`
- `joint4` : `[-0.5, 0] rad`
- `joint5` : `[0, 0.5] rad`, calcule par mimic depuis `joint4`

Les origines de joints sont des estimations initiales. Des commentaires `TODO
RViz` sont conserves dans le Xacro pour signaler les translations a affiner en
comparant les emboitements des meshes dans RViz 2.

## Installation

Depuis un workspace ROS 2 :

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/razafiarisonialy/modelisation_bras_robotique.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Dependances principales :

- ROS 2 Humble ou Jazzy
- `xacro`
- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`
- MoveIt 2

## Visualisation RViz 2

```bash
ros2 launch modelisation_bras_robotique display.launch.py
```

Le launch accepte aussi un chemin de modele explicite :

```bash
ros2 launch modelisation_bras_robotique display.launch.py model:=$(ros2 pkg prefix modelisation_bras_robotique)/share/modelisation_bras_robotique/urdf/arm.urdf.xacro
```

Dans RViz, activer/desactiver `Collision Enabled` dans le display
`RobotModel` pour faire une capture des collisions.

## MoveIt 2

La configuration MoveIt 2 se trouve dans
`modelisation_bras_robotique_moveit_config/`.

Lancement de la demo :

```bash
ros2 launch modelisation_bras_robotique_moveit_config demo.launch.py
```

Le groupe cinematique principal est `arm`, de `base_link` a `claw_support`.
Le groupe `gripper` pilote `joint4`; `joint5` est declare passif dans le SRDF
car il suit `joint4` via la balise `mimic` de l'URDF.

Etapes manuelles recommandees avec MoveIt Setup Assistant :

1. Lancer `ros2 launch moveit_setup_assistant setup_assistant.launch.py`.
2. Charger `urdf/arm.urdf.xacro` depuis le package de description.
3. Generer la matrice de collisions et verifier les collisions adjacentes.
4. Creer le groupe `arm` avec une chaine de `base_link` a `claw_support` et le solveur KDL.
5. Creer le groupe `gripper` avec `joint4`, puis declarer l'end-effector sur `claw_support`.
6. Ajouter les poses `home`, `ready`, `open` et `close`.
7. Generer le package `modelisation_bras_robotique_moveit_config` et tester avec `demo.launch.py`.

## Captures a ajouter

Les captures demandees par l'enonce peuvent etre placees dans
`docs/images/` :

- modele complet dans RViz 2 ;
- affichage des collisions ;
- demo MoveIt 2 avec une trajectoire planifiee.

## Difficultes et solutions

- Les URI de meshes peuvent echouer sous Linux si la casse ne correspond pas.
  Solution : referencer les fichiers fournis avec leur extension exacte `.STL`.
- Les formes de collision detaillees seraient trop couteuses pour la
  planification. Solution : collisions primitives simplifiees.
- Les origines de joints ne sont pas donnees dans le sujet. Solution : poser
  des valeurs initiales commentees, puis les ajuster dans RViz 2 avec
  `joint_state_publisher_gui`.
- La pince doit rester symetrique. Solution : utiliser un joint revolute mimic
  sur `joint5`, couple a `joint4`.
