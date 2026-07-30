# Trailer Safety Nodes - Guide d'Utilisation

## Vue d'ensemble

Ce package contient les nœuds de sécurité et de contrôle pour la navigation autonome d'un robot différentiel avec remorque passive, adaptés depuis le projet Husky.

## Nœuds Implémentés

### 1. cmd_vel_safety_node.py
**Fonction**: Protection anti-jackknife pour les commandes de vitesse

**Caractéristiques**:
- Limite la vitesse linéaire (max_forward_speed, max_reverse_speed)
- Limite la vitesse angulaire selon la cinématique de la remorque
- Réduction adaptive de ω_max selon l'angle β
- Publication du statut de sécurité (0=safe, 1=warning, 2=danger)

**Topics**:
- Sub: `/cmd_vel_raw` (Twist) - Commande brute
- Sub: `/trailer/beta` (Float64) - Angle d'attelage
- Pub: `/cmd_vel` (Twist) - Commande sécurisée
- Pub: `/cmd_vel_safety/status` (Float64) - Statut de sécurité

**Paramètres** (trailer_params.yaml):
```yaml
max_forward_speed: 0.26
max_reverse_speed: 0.15
max_stationary_omega: 1.0
hitch_distance: 0.50
beta_limit_deg: 45.0
```

### 2. trailer_aware_controller_node.py
**Fonction**: Transforme les commandes Nav2 en commandes sécurisées pour la remorque

**Caractéristiques**:
- Cinématique inverse pour contrôler la remorque
- Correction proportionnelle pour réduire β
- Réduction adaptive de la vitesse selon β
- Prédiction de l'état stationnaire de β

**Topics**:
- Sub: `/cmd_vel_nav2` (Twist) - Commande Nav2
- Sub: `/trailer/beta` (Float64) - Angle d'attelage
- Pub: `/cmd_vel_raw` (Twist) - Commande transformée

**Paramètres** (trailer_params.yaml):
```yaml
hitch_distance: 0.50
beta_limit_deg: 45.0
max_beta_deg: 35.0
beta_lookahead_time: 1.0
use_inverse_kinematics: true
adaptive_velocity_reduction: true
```

### 3. trailer_dashboard_node.py
**Fonction**: Interface graphique de monitoring en temps réel

**Caractéristiques**:
- Affichage des commandes /cmd_vel
- Affichage de l'odométrie /odom
- Affichage de l'angle β et de la marge
- Affichage des métriques de mouvement
- Barre de progression pour l'utilisation de β
- Indicateur de risque

**Topics**:
- Sub: `/cmd_vel` (Twist)
- Sub: `/odom` (Odometry)
- Sub: `/trailer/beta` (Float64)
- Sub: `/trailer/metrics` (Float64MultiArray)
- Sub: `/trailer/risk` (String)

**Paramètres**:
```yaml
beta_limit_deg: 45.0
hitch_distance: 0.50
```

### 4. trailer_control_node.py
**Fonction**: Interface de téléopération manuelle avec GUI

**Caractéristiques**:
- Boutons pour avancer, reculer, tourner
- Mode clic ou maintien
- Ajustement des vitesses linéaire et angulaire
- Saturation automatique de ω selon β
- Affichage de l'état et de β

**Topics**:
- Sub: `/trailer/beta` (Float64)
- Pub: `/cmd_vel` (Twist)

**Paramètres**:
```yaml
cmd_vel_topic: /cmd_vel
hitch_distance: 0.50
beta_limit_deg: 45.0
max_stationary_omega: 1.0
```

### 5. dynamic_footprint_node.py
**Fonction**: Modèle dynamique du footprint du convoi

**Caractéristiques**:
- Approximation par disques du robot et de la remorque
- Mise à jour selon l'angle β
- Publication sur `/local_costmap/published_footprint`

**Topics**:
- Sub: `/odom` (Odometry)
- Sub: `/trailer/beta` (Float64)
- Pub: `/local_costmap/published_footprint` (Polygon)

**Paramètres** (trailer_params.yaml):
```yaml
robot_length: 0.6
robot_width: 0.4
trailer_length: 0.8
trailer_width: 0.5
hitch_distance: 0.50
hitch_offset: 0.0
disk_radius: 0.12
```

## Architecture du Système

```
Nav2 (MPPI/Smac)
    ↓
trailer_aware_controller_node
    ↓
cmd_vel_safety_node
    ↓
/cmd_vel → Robot
```

## Fichiers de Configuration

### trailer_params.yaml
Contient tous les paramètres géométriques et de contrôle pour la remorque:
- Dimensions du robot et de la remorque
- Paramètres de l'attelage (d, hitch_offset)
- Limites cinématiques (β_lim, v_max, ω_max)
- Paramètres de contrôle

### nav2_params.yaml
Configuration Nav2 adaptée pour la remorque:
- MPPI Controller avec PreferForwardCritic (évite marche arrière)
- Smac Planner avec minimum_turning_radius=0.80m
- reverse_penalty=5.0 (forte pénalité)

## Commandes de Lancement

### 1. Lancer avec Gazebo et Dashboard uniquement (sans Nav2)
```bash
cd /home/rabeb/ros2_diff_drive_robot
source install/setup.bash
ros2 launch diff_robot robot_remorque.launch.py
```

Puis lancer le dashboard:
```bash
ros2 run diff_robot trailer_dashboard_node.py
```

Ou le contrôle manuel:
```bash
ros2 run diff_robot trailer_control_node.py
```

### 2. Lancer avec Nav2 et sécurité
```bash
cd /home/rabeb/ros2_diff_drive_robot
source install/setup.bash
ros2 launch diff_robot nav2_trailer_safety.launch.py \
    map_yaml:=/path/to/your/map.yaml \
    use_nav2:=true \
    launch_dashboard:=true
```

### 3. Lancer uniquement les nœuds de sécurité (avec robot déjà lancé)
```bash
ros2 run diff_robot trailer_aware_controller_node.py
ros2 run diff_robot cmd_vel_safety_node.py
ros2 run diff_robot dynamic_footprint_node.py
```

## Adaptation pour Robot Différentiel

### Différences principales avec le projet Husky:

1. **Type de robot**: Différentiel (pas skid-steer)
   - Pas de glissement latéral
   - Virages plus précis
   - Modèle cinématique plus simple

2. **Paramètres géométriques**:
   - d = 0.50 m (vs 0.80 m pour Husky)
   - β_lim = 45° (vs 70° pour Husky)
   - R_min = 0.707 m (vs 0.851 m pour Husky)

3. **Limites de vitesse**:
   - v_max = 0.26 m/s (vs 0.40 m/s pour Husky)
   - ω_max_stationnaire = 1.0 rad/s (vs 0.47 rad/s pour Husky)

4. **Nav2 Configuration**:
   - Motion model: "DiffDrive" (pas "ACKERMANN")
   - MPPI Controller au lieu de RegulatedPurePursuit
   - Smac Planner avec primitives différentielles

## Documentation Supplémentaire

- **CINEMATIQUE_DIFFERENTIEL.md**: Détails sur le modèle cinématique adapté
- **trailer_params.yaml**: Paramètres configurables
- **nav2_params.yaml**: Configuration Nav2

## Dépannage

### Problème: Le robot ne bouge pas
- Vérifiez que les nœuds sont lancés: `ros2 node list`
- Vérifiez les topics: `ros2 topic list`
- Vérifiez que /trailer/beta est publié par beta_ekf_node

### Problème: β atteint la limite rapidement
- Réduisez la vitesse dans nav2_params.yaml
- Augmentez β_lim dans trailer_params.yaml
- Vérifiez que trailer_aware_controller_node fonctionne

### Problème: Le footprint ne se met pas à jour
- Vérifiez que dynamic_footprint_node est lancé
- Vérifiez que /odom est publié
- Vérifiez que /trailer/beta est publié

## Compilation

```bash
cd /home/rabeb/ros2_diff_drive_robot
colcon build --packages-select diff_robot
source install/setup.bash
```

## Notes de Sécurité

- Le système est conçu pour éviter le jackknife (β > 45°)
- La marche arrière est fortement découragée (penalité dans Nav2)
- Le safety node a toujours la priorité sur les commandes
- Le dashboard permet de surveiller β en temps réel
