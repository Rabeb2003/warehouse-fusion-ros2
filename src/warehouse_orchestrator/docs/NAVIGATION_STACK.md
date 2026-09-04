# Documentation Navigation — Warehouse Fusion (ROS 2 Humble)

**Date :** 2026-08-03  
**Objectif mission :** trailer → couloir entre 2 murs → dock table Panda `(-0.24, 2.80)` → `/trailer/docking_status` → pick & place  
**Résultat dernier run :** **navigation ÉCHOUÉE** (`Nav finished status=6` = ABORTED)

Ce document décrit **exactement** ce qui est utilisé aujourd’hui, pour pouvoir chercher / comparer d’autres solutions.

---

## 1. Vue d’ensemble

| Couche | Solution actuelle | Package / plugin |
|--------|-------------------|------------------|
| Middleware | ROS 2 Humble + Gazebo Fortress (Ignition) | `ros_gz_*` |
| Stack navigation | **Nav2** | `nav2_*` |
| Localisation globale | **EKF map + GPS** (pas AMCL) | `robot_localization` |
| Localisation locale | **EKF odom** (roues + IMU) | `robot_localization` |
| Planificateur global | **NavfnPlanner (A\*)** | `nav2_navfn_planner/NavfnPlanner` |
| Contrôleur local | **Regulated Pure Pursuit (RPP)** | `nav2_regulated_pure_pursuit_controller` |
| Behavior Tree | XML custom trailer (nav_to_pose seulement) | `navigate_to_pose_trailer_recovery.xml` |
| Carte | Occupancy grid statique | `diff_robot/map/my_map.yaml` + `.pgm` |
| Sécurité remorque | Pipeline custom après Nav2 | `trailer_aware_controller` + `cmd_vel_safety_node` |
| Mission | Actions Nav2 | `NavigateThroughPoses` puis fallback `NavigateToPose` |

**Launch mission complète :**
```bash
ros2 launch warehouse_orchestrator warehouse_mission.launch.py
```
→ inclut `full_mission.launch.py` avec `enable_nav2:=true`, `enable_ekf:=true`, `enable_mission:=true`.

---

## 2. Fichiers de configuration (source de vérité)

| Rôle | Chemin |
|------|--------|
| Params Nav2 (planner, controller, costmaps, BT params) | `src/diff_robot/config/nav2_params.yaml` |
| Behavior Tree `navigate_to_pose` | `src/diff_robot/config/navigate_to_pose_trailer_recovery.xml` |
| EKF local + EKF map + navsat | `src/diff_robot/config/gps_ekf.yaml` |
| Launch localisation GPS | `src/diff_robot/launch/gps_localization.launch.py` |
| Params sécurité remorque | `src/diff_robot/config/trailer_params.yaml` |
| Diff drive (odom TF **off**) | `src/diff_robot/config/diff_drive_controller.yaml` (`enable_odom_tf: false`) |
| Capteurs GPS/IMU Gazebo | `src/diff_robot/urdf/sensors_gps_imu.xacro` |
| Carte | `src/diff_robot/map/my_map.yaml` |
| Orchestration Nav2 + mission | `src/warehouse_orchestrator/launch/full_mission.launch.py` |
| Mission auto dock→PnP | `src/warehouse_orchestrator/warehouse_orchestrator/mission_orchestrator.py` |
| Launch dédié mission | `src/warehouse_orchestrator/launch/warehouse_mission.launch.py` |
| Variante GPS sans static map (non utilisée dans mission) | `src/diff_robot/config/nav2_gps_params.yaml` |

---

## 3. Chaîne TF (localisation)

```
world ──(static identité)──► map ──(EKF map)──► odom ──(EKF local)──► base_footprint
                                      ▲                    ▲
                               /odometry/gps          /diff_drive_controller/odom
                               + twist odom + IMU      + /imu
```

### 3.1 EKF local — `ekf_filter_node_odom`
- **Fichier :** `gps_ekf.yaml` section `ekf_filter_node_odom`
- **Publie TF :** `odom → base_footprint` (`publish_tf: true`, `world_frame: odom`)
- **Sortie odom :** `/odometry/local` (remapping `odometry/filtered`)
- **Entrées :**
  - `odom` → remappé `/diff_drive_controller/odom` — **twist only** (vx, vy, ωz)
  - `imu` → yaw + ωz + ax
- **Démarrage :** t ≈ 40 s (`gps_localization.launch.py`)

### 3.2 navsat_transform — GPS → cartésien
- **Entrées :** `/gps/fix`, `/imu`, `/odometry/local`
- **Sortie :** `/odometry/gps`
- **Datum fixe :** `[49.0, 3.0, 0.0]` (`wait_for_datum: true`)
- **Bruit GPS sim :** stddev horizontal `0.15` m (`sensors_gps_imu.xacro`)
- **Démarrage :** t ≈ 42 s

### 3.3 EKF global — `ekf_filter_node_map`
- **Publie TF :** `map → odom`
- **Sortie :** `/odometry/global`
- **Fuse :** twist odom + IMU yaw + **pose XY de `/odometry/gps`**
- **Démarrage :** t ≈ 45 s
- **Pas d’AMCL** — commentaire explicite dans `nav2_params.yaml`

### 3.4 Pont Panda ↔ Nav2
- Static TF `world → map` (identité) dans `full_mission.launch.py`

### 3.5 Points critiques connus
- `enable_odom_tf: false` sur le diff_drive → **seul l’EKF** publie `odom→base`
- Bootstrap `map→odom` statique **désactivé** quand EKF est ON (sinon combat TF)
- Logs observés : `Failed to meet update rate` sur EKF (RTF bas sous double robot)

---

## 4. Stack Nav2 — détail plugins

### 4.1 Planificateur global (`planner_server`)

| Paramètre | Valeur actuelle |
|-----------|-----------------|
| Plugin ID | `GridBased` |
| Type | `nav2_navfn_planner/NavfnPlanner` |
| Algorithme | A\* (`use_astar: true`) |
| tolerance | 0.50 m |
| allow_unknown | true |
| expected_planner_frequency | 1.0 Hz |
| planner_patience | 15.0 s |
| max_planning_retries | 3 |

**Historique :** le header du YAML mentionne encore SMAC Lattice, mais le plugin actif est **Navfn**. SMAC a été abandonné car jugé trop complexe pour le couloir.

**Recherche alternatives (mots-clés) :**
- `nav2_smac_planner` / `SmacPlanner2D` / `SmacPlannerHybrid` / `SmacPlannerLattice`
- `nav2_theta_star_planner`
- `nav2_constrained_smoother` + lattice
- State Lattice / Hybrid A\* trailer / tractor-trailer path planning
- Dubins / Reeds-Shepp + hitch angle constraints

### 4.2 Contrôleur local (`controller_server`)

| Paramètre | Valeur actuelle |
|-----------|-----------------|
| Plugin ID | `FollowPath` |
| Type | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` |
| Fréquence | 20 Hz |
| desired_linear_vel | 0.35 m/s |
| lookahead | 0.45–1.40 m (scaled) |
| use_rotate_to_heading | true |
| use_collision_detection | **false** (désactivé en sim) |
| allow_reversing | false |
| xy_goal_tolerance | 0.50 m |
| yaw_goal_tolerance | 0.80 rad |
| odom utilisé par BT | `/odometry/local` |

**Progress checker :** `SimpleProgressChecker` (0.10 m / 90 s)  
**Goal checker :** `SimpleGoalChecker`

**Recherche alternatives :**
- `nav2_mppi_controller` / MPPI PreferForwardCritic (déjà évoqué dans `README_TRAILER_SAFETY.md`)
- `dwb_core` / DWB local planner
- `teb_local_planner` (ROS1 legacy ; ports ROS2)
- Custom pure pursuit avec contrainte β (angle hitch) — partiellement dans `cmd_vel_safety_node`
- Model Predictive Control tractor-trailer

### 4.3 Smoother
- `nav2_smoother::SimpleSmoother`

### 4.4 Behaviors (recovery)
Plugins : `Spin`, `BackUp`, `Wait`, `DriveOnHeading`  
Ordre voulu dans le BT custom : Clear costmaps → Wait → BackUp → Spin

### 4.5 Velocity smoother Nav2
- max_velocity : `[0.55, 0.0, 1.60]`
- Topic chaîne : voir §6

### 4.6 Behavior Tree

**Injecté pour `navigate_to_pose` :**
`diff_robot/config/navigate_to_pose_trailer_recovery.xml`

Contenu clé :
- Replan à **0.2 Hz** (`RateController`)
- `ComputePathToPose` + `FollowPath`
- RecoveryNode 6 retries : clear costmaps / backup −0.25 m / spin 0.60 rad

**Bug observé (critique) :**
```
[bt_navigator] Behavior tree threw exception: Empty Tree
[navigate_through_poses] Aborting handle
```
→ `default_nav_through_poses_bt_xml` est **vide** (`""` dans `nav2_params.yaml`)  
→ `NavigateThroughPoses` **ne peut pas marcher** tant qu’aucun BT through-poses n’est fourni  
→ le mission orchestrator tombe en fallback `NavigateToPose` direct vers le dock

---

## 5. Costmaps

### Local (`global_frame: odom`)
- Rolling window 10×10 m, résolution 0.05 m
- Layers : `obstacle_layer` (`/scan`) + `inflation_layer`
- inflation_radius : **0.30 m**, cost_scaling_factor : 5.5
- Footprint rectangle approx : `[[0.28,0.26],[0.28,-0.26],[-1.10,-0.26],[-1.10,0.26]]`
  (tractor + remorque allongée ; un footprint dynamique peut écraser via `/local_costmap/footprint`)

### Global (`global_frame: map`)
- Static map + obstacles scan + inflation 0.30 m
- Même footprint
- Carte : `my_map.yaml` — résolution 0.05, origin `[-7.79, -4.94, 0]`, mode trinary

**Risque couloir :** footprint long (~1.4 m) + inflation 0.30 peut **bloquer** le passage entre deux murs si le gap map/monde est trop étroit ou mal aligné GPS/map.

---

## 6. Pipeline cmd_vel (après Nav2)

```
controller_server / behavior_server
        │  cmd_vel → remappé cmd_vel_nav2_raw
        ▼
velocity_smoother
        │  cmd_vel_nav2
        ▼
trailer_aware_controller_node.py   (input /cmd_vel_nav2 → /cmd_vel_raw)
        ▼
cmd_vel_safety_node.py             (limite v/ω selon angle hitch β)
        │  /cmd_vel
        ▼
cmd_vel_relay                      (/cmd_vel → /diff_drive_controller/cmd_vel_unstamped)
        ▼
diff_drive_controller
```

**Params sécurité (`trailer_params.yaml`) :**
- `beta_limit_deg: 55`, `max_beta_deg: 48`
- `max_forward_speed: 0.55`
- `use_inverse_kinematics: true`

**Observation dernier run :** `v=0.00` avec `omega≈±0.07` et `beta≈41°` → le robot **tourne sur place** / n’avance plus (souvent coincé en recovery ou butée sécurité remorque).

---

## 7. Orchestration mission (goals)

Fichier : `mission_orchestrator.py`

| Étape | Action | Coordonnées (frame `map`) |
|-------|--------|---------------------------|
| 1a | `NavigateThroughPoses` (si BT OK) | `(1.50, -1.50, yaw=π/2)` → `(1.50, 1.20, π/2)` → dock |
| 1b | Fallback `NavigateToPose` | dock `x=-0.24, y=2.80, yaw=1.57` |
| 2 | Vérif TF optionnelle | drop vs `(-0.5, 3.2)`, tol 0.60 m |
| 3 | Publish | `/trailer/docking_status = true` |
| 4 | Attend | `/panda/task_completed` |

Spawn trailer typique : `x=-0.14, y=-2.07`

Démarrage mission : Timer ~70 s + délai interne 5 s.

---

## 8. Capteurs utilisés par la navigation

| Capteur | Topic ROS | Usage |
|---------|-----------|--------|
| LiDAR | `/scan` | costmaps obstacle |
| IMU | `/imu` | EKF local + map + navsat yaw |
| GPS | `/gps/fix` | → navsat → `/odometry/gps` → EKF map |
| Odom roues | `/diff_drive_controller/odom` | EKF (twist) |
| Odom filtrée | `/odometry/local` | Nav2 `odom_topic`, BT |
| Angle hitch β | via `beta_ekf_node` / joint | `cmd_vel_safety` |

**Non utilisé :** AMCL, slam_toolbox (présent dans le repo mais pas dans `warehouse_mission`).

---

## 9. Échecs observés (run 2026-08-03)

| Symptôme | Log / évidence | Interprétation |
|----------|----------------|----------------|
| ThroughPoses KO | `Empty Tree` sur `navigate_through_poses` | BT through-poses non configuré |
| NavToPose abort | `Nav finished status=6` puis `Navigation failed — mission abort` | Goal dock non atteint |
| Planner timeout | `Timed out ... compute_path_to_pose` | planner_server saturé / RTF bas |
| EKF lent | `Failed to meet update rate` | sim trop lourde → odom/TF irréguliers |
| Controller lent | `Control loop missed ... 20 Hz` | même cause RTF |
| Sur place | `v=0.00 omega=±0.07 beta≈41°` | bloqué en rotation / couloir / sécurité |

---

## 10. Alternatives à explorer (checklist recherche)

### A. Localisation (si odom/GPS instable dans le couloir)
1. **AMCL** sur carte `my_map` + LiDAR (classique Nav2) — désactiver EKF map GPS  
2. GPS **seulement** pour init pose, puis odom+IMU seul (`publish_tf` map figé / static)  
3. **slam_toolbox** localisation mode (fichiers déjà dans `diff_robot/launch/`)  
4. Réduire bruit / poids GPS (`odom1` covariance, `pose_rejection_threshold`)  
5. Datum / alignement map↔GPS : vérifier que `my_map` et le monde SDF sont cohérents

### B. Planification globale
1. **SmacPlannerHybrid** ou **SmacPlannerLattice** avec `minimum_turning_radius` adapté remorque  
2. **Theta\*** pour chemins plus droits dans le couloir  
3. Waypoints manuels simples via **`/navigate_to_pose` en série** (sans ThroughPoses) — 3 goals successifs dans l’orchestrator  
4. Path offline (A\* custom `planning/a_star_path_finding.py`, RRT `planning/rrt.py`) + FollowPath  
5. Élargir le gap monde / réduire footprint / inflation

### C. Contrôle local
1. **MPPI** (recommandé dans docs internes trailer safety)  
2. RPP avec collision_detection ON + footprint dynamique fiable  
3. Autoriser `allow_reversing` pour sortir d’un coin  
4. Baisser vitesse encore (0.15–0.25 m/s) dans le couloir  
5. Désactiver temporairement `cmd_vel_safety` pour isoler si β bloque la mission

### D. Behavior Tree / mission
1. Fournir un **BT `navigate_through_poses`** Nav2 standard (copie install `nav2_bt_navigator`)  
2. Remplacer ThroughPoses par **3× NavigateToPose** séquentiels  
3. Goal intermédiaire **au centre du gap** mesuré dans Gazebo (pas approximé 1.5)

### E. Performance sim
1. Headless Gazebo / Physics step plus lent / moins de capteurs  
2. Baisser fréquences EKF / controller (10 Hz) pour coller au RTF

---

## 11. Commandes de diagnostic utiles

```bash
# TF
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_footprint

# Odom / GPS
ros2 topic hz /odometry/local /odometry/gps /gps/fix /imu /scan
ros2 topic echo /odometry/local --once

# Nav2
ros2 action list | grep navigate
ros2 topic echo /plan --once
ros2 topic echo /local_costmap/costmap --once

# Cmd pipeline
ros2 topic echo /cmd_vel_nav2 --once
ros2 topic echo /cmd_vel --once
```

Goal manuel test (sans mission) :
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: -0.24, y: 2.80, z: 0.0},
   orientation: {z: 0.707, w: 0.707}}}}"
```

---

## 12. Versions / packages ROS attendus

- `nav2_bringup`, `nav2_planner`, `nav2_controller`, `nav2_bt_navigator`, `nav2_behaviors`, `nav2_smoother`, `nav2_velocity_smoother`, `nav2_map_server`, `nav2_waypoint_follower`
- `nav2_navfn_planner`
- `nav2_regulated_pure_pursuit_controller`
- `robot_localization` (`ekf_node`, `navsat_transform_node`)
- Custom : `diff_robot` scripts safety + `warehouse_orchestrator` mission

---

## 13. Résumé (mis à jour 2026-08-03)

**Corrections appliquées :**
1. Mission = **3× NavigateToPose** séquentiels (plus de ThroughPoses / Empty Tree).
2. RPP : `use_rotate_to_heading: false` + controller 10 Hz (anti spin v=0).
3. `cmd_vel_safety` : si β>25° et v≈0 → force avance 0.12 m/s (anti jackknife spin).
4. GPS : EKF map fuse `/odometry/gps` explicitement ; goals en frame `map` GPS-corrigée.

**Stack :** Nav2 + Navfn A* + RPP + dual-EKF+GPS + sécurité remorque.
