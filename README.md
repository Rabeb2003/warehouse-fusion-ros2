# Warehouse Fusion — README

## Contexte

Fusion de deux projets ROS2 Humble / Gazebo distincts et fonctionnels en un
seul environnement de simulation d'entrepôt (warehouse).

**Projet 1 — Navigation trailer (package `diff_robot`)**
Robot différentiel (Husky-like) tirant une remorque passive, navigation
autonome via Nav2 (SmacPlannerLattice + RegulatedPurePursuitController),
localisation AMCL + EKF (robot_localization), GPS intégré. Contrôle
anti-jackknife personnalisé basé sur IS-MPC.
État stable, commit : `7dc6b43` (tag `etat-stable-trailer-v1`).

**Projet 2 — Bras robotique panda (`panda_gz_moveit2`)**
Bras Panda avec pick-and-place fonctionnel via MoveIt2.
État stable, tag : `etat-stable-panda-v1`.

## But de la fusion

Faire cohabiter les deux robots dans le même monde Gazebo, pour simuler un
scénario d'entrepôt où le robot trailer navigue de manière autonome vers une
zone, et le bras panda effectue un pick-and-place à cet endroit (ou toute
autre interaction entre les deux systèmes).

## Contrainte principale

**Zéro risque pour les deux projets originaux.** Toute la fusion se fait
dans un workspace séparé, avec des commits Git réguliers à chaque étape,
pour pouvoir revenir en arrière à tout moment sans jamais perdre les états
stables initiaux.

---

## Architecture de sécurité à 3 niveaux

| Niveau | Ce qu'il protège | Comment revenir en arrière |
|---|---|---|
| Dossiers originaux jamais touchés (`panda_gz_moveit2-master/`, `ros2_diff_drive_robot/`) | Les deux projets qui marchent déjà | Ils n'ont jamais changé, rien à faire |
| Tag `debut-fusion` dans le nouveau workspace | Le point de départ de la fusion elle-même | `git reset --hard debut-fusion` |
| Commits réguliers + branches | Chaque étape intermédiaire de la fusion | `git reset --hard <hash>` ou `git checkout main` |

Pire scénario possible : supprimer tout `warehouse_fusion_ws` et recopier les
deux sources originales — 30 secondes de travail perdu, jamais plus.

---

## Procédure complète

### 1. Sécuriser l'état actuel de chaque projet source

```bash
cd ~/Downloads/panda_gz_moveit2-master
git add -A
git commit -m "État stable : pick-and-place fonctionnel avant fusion trailer"
git tag etat-stable-panda-v1

cd ~/ros2_diff_drive_robot
git add -A
git commit -m "État stable : navigation trailer fonctionnelle avant fusion"
git tag etat-stable-trailer-v1
```

Le tag est une bouée de sauvetage : peu importe ce qui se passe après,
`git checkout etat-stable-panda-v1` (ou `7dc6b43` directement) restaure
exactement cet état.

### 2. Créer le workspace de fusion (par copie, jamais par déplacement)

```bash
mkdir -p ~/warehouse_fusion_ws/src
cp -r ~/Downloads/panda_gz_moveit2-master/src/* ~/warehouse_fusion_ws/src/
cp -r ~/ros2_diff_drive_robot/src/* ~/warehouse_fusion_ws/src/
```

`cp -r` copie, ne déplace pas — les dossiers originaux restent intacts.

### 3. Initialiser le dépôt Git du workspace de fusion

```bash
cd ~/warehouse_fusion_ws
git init
git add -A
git commit -m "Point de départ : copie des deux projets avant fusion"
git tag debut-fusion
```

### 4. Committer à chaque petite étape

```bash
git add -A
git commit -m "Étape 1 : fusion des deux worlds Gazebo"
# ...
git add -A
git commit -m "Étape 2 : ajout static_transform_publisher world->map"
```

Annuler la dernière étape si elle casse quelque chose :
```bash
git reset --hard HEAD~1
```

Revenir à un point précis plus loin dans l'historique :
```bash
git log --oneline
git reset --hard <hash>
```

### 5. Utiliser des branches pour les tentatives risquées

```bash
git checkout -b test-fusion-worlds
# ... expérimentation ...
git checkout main        # retour instantané si ça casse, rien n'est perdu
git merge test-fusion-worlds   # si ça marche, on intègre
```

---

## Structure de dossiers

```
~/warehouse_fusion_ws/
├── README.md              (ce fichier)
└── src/
    ├── panda_description/
    ├── panda_moveit_config/
    ├── diff_robot/
    ├── trailer_kinematics/
    └── warehouse_orchestrator/   (créé plus tard, pendant la fusion)
```

Ne crée `warehouse_orchestrator/` que quand tu commences à écrire la logique
qui coordonne les deux robots ensemble.

---

## Étapes prévues de la fusion

1. Fusion des deux worlds Gazebo en un seul monde d'entrepôt
2. Ajout des transformations statiques nécessaires (ex: `world -> map`) pour
   que les deux robots partagent un repère cohérent
3. Vérification qu'il n'y a pas de conflit de noms (topics, TF frames,
   packages) entre les deux projets une fois copiés ensemble
4. Création du package `warehouse_orchestrator` qui déclenche le
   pick-and-place une fois que le robot trailer a atteint sa position

---

## Où j'en suis maintenant

_(à compléter au fur et à mesure)_

---

## Outillage

Travail fait localement (terminal + IDE), pas via un agent cloud sandboxé
(type Devin) — ce projet demande d'observer Gazebo en direct et d'itérer
vite sur le comportement de la simulation, ce qui va mieux avec un agent
ayant un accès terminal/filesystem local (ex: Antigravity) qu'avec un
sandbox cloud isolé sans affichage graphique.

## Script de setup rapide

```bash
#!/bin/bash
set -e

PANDA_SRC=~/Downloads/panda_gz_moveit2-master
TRAILER_SRC=~/ros2_diff_drive_robot
FUSION_WS=~/warehouse_fusion_ws

mkdir -p "$FUSION_WS/src"
cp -r "$PANDA_SRC/src/"* "$FUSION_WS/src/"
cp -r "$TRAILER_SRC/src/"* "$FUSION_WS/src/"

cd "$FUSION_WS"
git init
git add -A
git commit -m "Point de départ : copie des deux projets avant fusion"
git tag debut-fusion

echo "Workspace prêt dans : $FUSION_WS"
echo "Pour revenir au point de départ : git reset --hard debut-fusion"
```
