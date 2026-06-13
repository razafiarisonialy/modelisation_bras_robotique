# Modélisation d'un bras robotique

**Technologies :** ROS 2 (Jazzy) · URDF / Xacro · `ros2_control` · MoveIt 2 · RViz 2

Projet ROS 2 qui modélise un bras robotique industriel à 3 degrés de liberté
(+ une pince à 1 degré de liberté commandé) en Xacro, le visualise dans
RViz 2, et fournit une configuration MoveIt 2 complète (planification OMPL,
simulation `ros2_control` avec hardware factice).

## Aperçu rapide

```bash
# Visualisation seule (RViz 2 + joint_state_publisher_gui)
ros2 launch modelisation_bras_robotique display.launch.py

# Démo complète MoveIt 2 (RViz 2 + move_group + ros2_control + RViz MotionPlanning)
ros2 launch modelisation_bras_robotique_moveit_config demo.launch.py
```

## 1. Structure du dépôt

Le dépôt contient deux packages ROS 2 ament_cmake placés côte à côte :

```text
modelisation_bras_robotique/            # package de description du robot
├── urdf/
│   ├── arm.urdf.xacro                  # description complète du bras (links, joints, visuels, collisions, inerties)
│   ├── bras_robotique.urdf.xacro       # point d'entrée, inclus par le package MoveIt
│   ├── arm_macro.xacro                 # macros visuel / collision
│   ├── inertials.xacro                 # macros d'inertie (boîte)
│   └── materials.xacro                 # couleurs des matériaux
├── meshes/                              # géométries .STL fournies pour le TP
├── launch/
│   └── display.launch.py               # RViz 2 + robot_state_publisher + joint_state_publisher_gui
├── rviz/
│   └── view_robot.rviz                 # configuration RViz pour la visualisation
└── config/
    └── joint_names.yaml                # liste des joints actionnés (référence)

modelisation_bras_robotique_moveit_config/   # package généré par MoveIt Setup Assistant
├── config/
│   ├── bras_robotique.urdf.xacro       # inclut le robot + macro ros2_control
│   ├── bras_robotique.ros2_control.xacro
│   ├── bras_robotique.srdf             # groupes, états nommés, collisions désactivées
│   ├── joint_limits.yaml
│   ├── ompl_planning.yaml
│   ├── kinematics.yaml
│   ├── moveit_controllers.yaml / ros2_controllers.yaml
│   ├── pilz_cartesian_limits.yaml / sensors_3d.yaml / trajectory_execution.yaml
│   └── initial_positions.yaml
└── launch/
    ├── demo.launch.py                  # démo complète (RViz + MoveIt + ros2_control)
    ├── move_group.launch.py
    ├── moveit_rviz.launch.py
    ├── rsp.launch.py
    ├── spawn_controllers.launch.py
    ├── static_virtual_joint_tfs.launch.py
    ├── warehouse_db.launch.py
    └── setup_assistant.launch.py       # ré-ouvre le Setup Assistant sur cette config
```

## 2. Choix URDF vs Xacro

Le projet utilise une **architecture Xacro modulaire** plutôt qu'un unique
fichier URDF.

- **Critère décisif : la répétition.** Chaque lien possède la même structure
  (un `<visual>` avec mesh + matériau, une `<collision>` primitive, une
  `<inertial>` calculée). Les macros `mesh_visual`, `box_collision`,
  `cylinder_collision` (`arm_macro.xacro`) et `box_link_inertial`
  (`inertials.xacro`) factorisent ce code répétitif pour les 7 liens mobiles.
- **Avantages** :
  - description plus courte et plus lisible (une ligne par mesh/collision/inertie
    au lieu d'un bloc XML complet) ;
  - un seul endroit où corriger un bug de macro (ex. le calcul de l'inertie
    d'une boîte) au lieu de le répéter 7 fois ;
  - facilite l'ajout de variantes (autre échelle, autre hardware
    `ros2_control`) sans dupliquer tout le XML.
- **Inconvénients** :
  - nécessite l'outil `xacro` pour générer l'URDF final (les outils qui
    n'acceptent que de l'URDF brut doivent d'abord exécuter
    `xacro arm.urdf.xacro`) ;
  - une erreur dans une macro (paramètre manquant, typo dans `${...}`) est
    parfois moins explicite qu'une erreur de syntaxe URDF directe.
- **Cas où un URDF unique aurait suffi** : un robot très simple (1‑2 liens,
  aucune répétition de visuel/collision/inertie) où l'effort de factorisation
  via macros coûterait plus cher que la duplication elle‑même.

## 3. Description du robot

Bras manipulateur à 3 degrés de liberté (rotation de base, épaule, coude) +
une pince à 2 doigts dont le doigt gauche est asservi au doigt droit via
`<mimic joint="joint4" multiplier="-1"/>`.

### Chaîne cinématique

```text
world -> base_link -> base_plate -> forward_drive_arm -> horizontal_arm -> claw_support -> gripper_right
                                                                                          -> gripper_left
```

| Joint | Type | Axe | Parent → Enfant | Rôle |
|---|---|---|---|---|
| `virtual_joint` | fixed | – | `world` → `base_link` | ancrage au monde |
| `joint1` | revolute | Z | `base_link` → `base_plate` | rotation de la base |
| `joint2` | revolute | X | `base_plate` → `forward_drive_arm` | épaule |
| `joint3` | revolute | X | `forward_drive_arm` → `horizontal_arm` | coude |
| `joint_claw` | fixed | – | `horizontal_arm` → `claw_support` | support de pince |
| `joint4` | revolute | Z | `claw_support` → `gripper_right` | doigt droit (actionné) |
| `joint5` | revolute (mimic `joint4`) | Z | `claw_support` → `gripper_left` | doigt gauche (passif) |

Les origines des visuels reprennent les valeurs fournies dans l'énoncé du TP
et tous les meshes sont affichés avec `scale="0.01 0.01 0.01"`.

### Collisions

Chaque lien possède une géométrie de collision **primitive** (boîte ou
cylindre), volontairement plus simple que le mesh visuel, afin de garder la
vérification de collision rapide pour MoveIt :

- `base_link` / `base_plate` : cylindres ;
- `forward_drive_arm`, `horizontal_arm`, `claw_support`, `gripper_right`,
  `gripper_left` : boîtes englobantes.

> 📷 *Capture d'écran à ajouter ici : RViz 2 avec `RobotModel → Collision Enabled`
> activé, montrant les volumes de collision simplifiés.*

## 4. Hypothèses

### Meshes

Les meshes STL fournis sont référencés via
`package://modelisation_bras_robotique/meshes/<nom>.STL`, en conservant
l'extension `.STL` en majuscules (les noms de fichiers fournis), car la casse
est significative sous Linux.

### Masses et inerties

Les masses et inerties ne sont pas fournies dans l'énoncé : elles sont donc
**estimées** à partir de boîtes/cylindres englobants de dimensions cohérentes
avec les meshes, dans une plage pédagogique de `0.1 kg` à `1.0 kg` :

- base lourde (`base_link` 1.0 kg, `base_plate` 0.8 kg) pour stabiliser le robot ;
- bras de plus en plus léger en s'éloignant de la base (`forward_drive_arm`
  0.6 kg, `horizontal_arm` 0.5 kg, `claw_support` 0.25 kg) ;
- pince légère, deux doigts identiques de 0.1 kg couplés par `mimic`.

Le tenseur d'inertie de chaque lien est calculé par la macro `box_inertia`
(`inertials.xacro`) à partir de la masse et des dimensions de la boîte
englobante.

### Limites articulaires

Choisies pour rester dans une amplitude de mouvement réaliste sans collision
entre liens adjacents :

- `joint1` : `[-π, π]` rad — rotation complète de la base ;
- `joint2`, `joint3` : `[-π/2, π/2]` rad — épaule/coude ;
- `joint4` : `[-0.5, 0]` rad — fermeture du doigt droit ;
- `joint5` : `[0, 0.5]` rad — calculé par `mimic` depuis `joint4` (`multiplier="-1"`).

Toutes les limites de vitesse/effort (`velocity`, `effort`) sont des valeurs
indicatives raisonnables pour un bras de cette taille.

### Origines des joints

Les origines de joints ne sont pas données explicitement dans les fichiers
sources : elles sont déduites des cotes fournies sur les schémas, multipliées
par 10 pour rester cohérentes avec l'échelle visuelle `0.01` appliquée aux
meshes.

| Repères (image) | Cote mesurée | Joint | Axe retenu | Valeur xacro |
|---|---|---|---|---|
| `base_link` / `base_plate` | 3.07 cm | `joint1` | Z | `0.307` |
| `base_plate` / `forward_drive_arm` | 3.5 cm + 2 mm | `joint2` | Z + X | `0.36` / `-0.018` |
| `forward_drive_arm` / `horizontal_arm` | 8 cm | `joint3` | Z | `0.78` |
| `horizontal_arm` / `claw_support` | 8.2 cm | `joint_claw` | Y | `0.82` |
| `claw_support` / `gripper_right` & `gripper_left` | 1 cm, 2.2 cm | `joint4` / `joint5` | Z, X | `-0.1` / `±0.11` |

L'axe (X/Y/Z et signe) retenu pour chaque joint a été vérifié visuellement
dans RViz 2 avec `joint_state_publisher_gui` : les meshes s'emboîtent
correctement sur toute la course des joints.

## 5. Installation

Depuis un workspace ROS 2 existant :

```bash
cd ~/ros2_ws/src
git clone https://github.com/razafiarisonialy/modelisation_bras_robotique.git
cd ~/ros2_ws
```

### Dépendances

```bash
sudo apt update
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```

Cette commande installe toutes les dépendances déclarées dans les
`package.xml` des deux packages : `xacro`, `robot_state_publisher`,
`joint_state_publisher_gui`, `rviz2`, ainsi que les paquets MoveIt 2
(`moveit_ros_move_group`, `moveit_ros_visualization`, `moveit_planners`,
`moveit_kinematics`, `moveit_configs_utils`, `moveit_setup_assistant`) et
`ros2_control`/`controller_manager`.

> Si `rosdep` échoue à résoudre les paquets MoveIt 2 sur votre distribution,
> suivez le guide d'installation binaire officiel :
> https://moveit.ai/install-moveit2/binary/

### Build

```bash
colcon build --packages-select modelisation_bras_robotique modelisation_bras_robotique_moveit_config
source install/setup.bash
```

## 6. Lancer RViz 2 (description seule)

```bash
ros2 launch modelisation_bras_robotique display.launch.py
```

Arguments disponibles :

- `model` (défaut : `urdf/arm.urdf.xacro` du package) — chemin vers un autre
  fichier Xacro/URDF à charger ;
- `use_gui` (défaut : `true`) — démarre `joint_state_publisher_gui` pour
  bouger chaque articulation. Mettre `use_gui:=false` pour un lancement sans
  interface de contrôle des joints (utile pour une capture d'écran propre).

> 📷 *Capture d'écran à ajouter ici : modèle complet du bras dans RViz 2,
> arbre TF sans erreur, avec `joint_state_publisher_gui` ouvert.*

## 7. Lancer MoveIt 2

```bash
ros2 launch modelisation_bras_robotique_moveit_config demo.launch.py
```

Ce launch démarre : `robot_state_publisher`, `move_group`, RViz 2 (avec le
plugin **MotionPlanning**), `ros2_control_node` (hardware factice
`mock_components/GenericSystem`) et les contrôleurs `arm_controller`,
`gripper_controller`, `joint_state_broadcaster`.

Groupes définis dans le SRDF :

- `arm` : chaîne de `base_link` à `claw_support`, solveur **KDL**, planificateur **OMPL** ;
- `gripper` : pilote `joint4` (end‑effector sur `claw_support`), `joint5` est
  déclaré `passive_joint` car il suit `joint4` via `mimic`.

États nommés disponibles dans l'onglet **Stored States** de RViz : `home`,
`ready` (groupe `arm`), `open`, `close` (groupe `gripper`).

> 📷 *Capture d'écran / GIF à ajouter ici : RViz 2 avec le plugin MotionPlanning,
> une planification entre `home` et `ready` exécutée avec succès (chemin vert,
> pas d'erreur dans le panneau Status).*

### Génération du package MoveIt

Le package `modelisation_bras_robotique_moveit_config` a été généré avec le
**MoveIt Setup Assistant** à partir de `urdf/bras_robotique.urdf.xacro` :

1. `ros2 launch moveit_setup_assistant setup_assistant.launch.py`
2. Chargement de l'URDF/Xacro du package de description.
3. Génération de la matrice de collisions (collisions adjacentes désactivées
   automatiquement, voir `bras_robotique.srdf`).
4. Création du groupe `arm` (chaîne `base_link` → `claw_support`, solveur KDL)
   et du groupe `gripper` (`joint4`, end‑effector sur `claw_support`).
5. Ajout des états nommés `home`, `ready`, `open`, `close`.
6. Configuration `ros2_control` avec hardware factice (`mock_components/GenericSystem`).
7. Génération du package complet.

Pour modifier la configuration générée, relancer le Setup Assistant sur cette
config existante :

```bash
ros2 launch modelisation_bras_robotique_moveit_config setup_assistant.launch.py
```

## 8. Difficultés rencontrées et solutions

- **URI de meshes sous Linux** : les noms de fichiers fournis utilisent
  l'extension `.STL` en majuscules ; comme la casse est significative sous
  Linux, les références `package://.../meshes/<nom>.STL` doivent reprendre
  exactement la casse des fichiers présents dans `meshes/`.
- **Coût des collisions détaillées** : utiliser les meshes complets comme
  géométrie de collision serait trop coûteux pour la planification. →
  collisions remplacées par des primitives (boîtes/cylindres) englobantes.
- **Origines de joints non fournies** : déduites des cotes des schémas (voir
  tableau §4), puis vérifiées/ajustées dans RViz 2 avec
  `joint_state_publisher_gui` pour confirmer l'emboîtement correct des
  meshes sur toute la course des joints.
- **Symétrie de la pince** : un second joint `revolute` indépendant aurait pu
  désynchroniser les deux doigts. → `joint5` utilise
  `<mimic joint="joint4" multiplier="-1"/>` et est déclaré `passive_joint`
  dans le SRDF.
- **`move_group` plantait au démarrage** (`InvalidParameterTypeException` sur
  `robot_description_planning.joint_limits.*.max_velocity`) : les valeurs
  numériques de `joint_limits.yaml` étaient écrites comme entiers (`2`, `0`)
  alors que MoveIt attend des `double`. → écrites en `2.0` / `0.0`.
- **`move_group` plantait avec *"Planning plugin name is empty or not defined
  in namespace 'ompl'"*** : `ompl_planning.yaml` utilisait l'ancien format
  `planning_plugin:` (chaîne unique) au lieu du format attendu par MoveIt 2 sur
  Jazzy, `planning_plugins:` (liste). Les adaptateurs de réponse
  (`AddTimeOptimalParameterization`, `ValidateSolution`, `DisplayMotionPath`)
  doivent également être déclarés sous `response_adapters` et non
  `request_adapters`. → fichier mis à jour avec le format actuel.
- **`package 'controller_manager' not found`** au lancement de `demo.launch.py` :
  les paquets `ros-jazzy-ros2-control` / `ros-jazzy-ros2-controllers` (qui
  fournissent `controller_manager`) n'étaient pas installés. → installés via
  `rosdep install` (voir §5).
- **Erreurs `occupancy_map_monitor/DepthImageOctomapUpdater` introuvable** :
  `sensors_3d.yaml` référençait un plugin de capteur de profondeur inexistant
  alors que le robot n'a pas de caméra 3D. → `sensors: []`.
