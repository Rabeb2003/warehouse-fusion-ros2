"""
Script de Pick & Place automatique pour le robot Panda dans Robosuite.
Le robot lit la position de l'objet rouge (canette) et calcule les mouvements
nécessaires pour l'attraper et le déposer dans le bac cible (le "trailer").

Ce script utilise une logique de machine à états (state machine) :
  1. MOVE_ABOVE  : Se positionner au-dessus de l'objet
  2. DESCEND     : Descendre vers l'objet
  3. GRASP       : Fermer la pince
  4. LIFT        : Soulever l'objet
  5. MOVE_TO_BIN : Se déplacer au-dessus du bac cible
  6. LOWER       : Descendre dans le bac
  7. RELEASE     : Ouvrir la pince et lâcher l'objet
  8. DONE        : Tâche terminée !
"""

import time
import numpy as np
import robosuite as suite


def main():
    print("=" * 60)
    print("  ROBOSUITE - Pick & Place Automatique (Robot Panda)")
    print("  Objet : Canette rouge  →  Destination : Bac cible")
    print("=" * 60)

    # ── 1. Créer l'environnement ──────────────────────────────
    env = suite.make(
        env_name="PickPlaceCan",       # Tâche simplifiée : 1 seul objet (canette rouge)
        robots="Panda",
        has_renderer=True,
        has_offscreen_renderer=False,
        use_camera_obs=False,
        use_object_obs=True,           # IMPORTANT : donne la position de l'objet
        reward_shaping=True,           # Récompense progressive
        control_freq=20,
        horizon=2000,                  # 2000 pas max par épisode
        ignore_done=True,
    )

    obs = env.reset()
    env.viewer.set_camera(camera_id=0)

    print("\n[INFO] Environnement initialisé avec succès !")
    print("[INFO] Observation keys disponibles :")
    for key in sorted(obs.keys()):
        if isinstance(obs[key], np.ndarray):
            print(f"  - {key}: shape={obs[key].shape}")
        else:
            print(f"  - {key}: {obs[key]}")

    # ── 2. Identifier les clés d'observation ──────────────────
    # Chercher les bonnes clés pour la position de l'objet et du robot
    eef_key = None
    obj_pos_key = None
    gripper_key = None

    for key in obs.keys():
        if "eef_pos" in key and eef_key is None:
            eef_key = key
        if "Can_pos" in key and obj_pos_key is None:
            obj_pos_key = key
        if "gripper" in key and "qpos" in key and gripper_key is None:
            gripper_key = key

    if eef_key is None or obj_pos_key is None:
        print("\n[ERREUR] Impossible de trouver les clés d'observation.")
        print("  Clés trouvées :", list(obs.keys()))
        env.close()
        return

    print(f"\n[INFO] Clé position effecteur (pince) : {eef_key}")
    print(f"[INFO] Clé position objet (canette)  : {obj_pos_key}")
    if gripper_key:
        print(f"[INFO] Clé état pince               : {gripper_key}")

    # ── 3. Paramètres de la machine à états ───────────────────
    # La position du bac cible (bin2) est codée dans l'environnement
    bin2_pos = np.array(env.bin2_pos)
    print(f"[INFO] Position du bac cible (trailer) : {bin2_pos}")

    # Hauteurs de travail
    hover_height = 0.15     # Hauteur au-dessus de l'objet
    grasp_height = -0.01    # Hauteur de saisie (légèrement en dessous)
    lift_height = 0.20      # Hauteur de levage

    # Gains de contrôle (vitesse de déplacement)
    pos_gain = 10.0         # Gain pour le déplacement XYZ
    gripper_open = -1.0     # Commande : ouvrir la pince
    gripper_close = 1.0     # Commande : fermer la pince

    # ── 4. Machine à états ────────────────────────────────────
    state = "MOVE_ABOVE"
    grasp_counter = 0       # Compteur pour laisser le temps à la pince de se fermer
    step_count = 0
    success = False

    print("\n[START] Début du Pick & Place automatique !\n")

    while step_count < 2000:
        start_time = time.time()

        # Lire les positions actuelles
        eef_pos = obs[eef_key]        # Position de la pince [x, y, z]
        obj_pos = obs[obj_pos_key]    # Position de la canette [x, y, z]

        # Calculer l'action (delta position + gripper)
        action_dim = env.action_dim
        action = np.zeros(action_dim)

        # ── État 1 : Se positionner au-dessus de l'objet ──
        if state == "MOVE_ABOVE":
            target = obj_pos.copy()
            target[2] += hover_height  # Au-dessus de l'objet

            delta = target - eef_pos
            action[:3] = delta * pos_gain
            action[-1] = gripper_open  # Pince ouverte

            if np.linalg.norm(delta[:2]) < 0.02 and abs(delta[2]) < 0.02:
                state = "DESCEND"
                print(f"  [Étape {step_count:4d}] ✓ Au-dessus de l'objet → Descente...")

        # ── État 2 : Descendre vers l'objet ──
        elif state == "DESCEND":
            target = obj_pos.copy()
            target[2] += grasp_height  # Hauteur de saisie

            delta = target - eef_pos
            action[:3] = delta * pos_gain
            action[-1] = gripper_open  # Pince ouverte

            if abs(delta[2]) < 0.02:
                state = "GRASP"
                grasp_counter = 0
                print(f"  [Étape {step_count:4d}] ✓ En position de saisie → Fermeture pince...")

        # ── État 3 : Fermer la pince ──
        elif state == "GRASP":
            action[:3] = np.zeros(3)   # Ne pas bouger
            action[-1] = gripper_close  # Fermer la pince

            grasp_counter += 1
            if grasp_counter > 40:  # Attendre ~2 secondes pour que la pince se ferme
                state = "LIFT"
                print(f"  [Étape {step_count:4d}] ✓ Objet saisi → Levage...")

        # ── État 4 : Soulever l'objet ──
        elif state == "LIFT":
            target = obj_pos.copy()
            target[2] = bin2_pos[2] + lift_height  # Soulever haut

            delta = target - eef_pos
            action[:3] = delta * pos_gain
            action[-1] = gripper_close  # Maintenir la pince fermée

            if eef_pos[2] > bin2_pos[2] + lift_height - 0.03:
                state = "MOVE_TO_BIN"
                print(f"  [Étape {step_count:4d}] ✓ Objet levé → Déplacement vers le bac...")

        # ── État 5 : Se déplacer au-dessus du bac cible (trailer) ──
        elif state == "MOVE_TO_BIN":
            target = bin2_pos.copy()
            target[2] = bin2_pos[2] + lift_height  # Rester en hauteur

            delta = target - eef_pos
            action[:3] = delta * pos_gain
            action[-1] = gripper_close  # Maintenir la pince fermée

            if np.linalg.norm(delta[:2]) < 0.03:
                state = "LOWER"
                print(f"  [Étape {step_count:4d}] ✓ Au-dessus du bac → Descente dans le bac...")

        # ── État 6 : Descendre dans le bac ──
        elif state == "LOWER":
            target = bin2_pos.copy()
            target[2] = bin2_pos[2] + 0.10  # Au-dessus du fond du bac (accessible)

            delta = target - eef_pos
            action[:3] = delta * pos_gain
            action[-1] = gripper_close  # Maintenir la pince fermée

            if abs(delta[2]) < 0.05 and np.linalg.norm(delta[:2]) < 0.05:
                state = "RELEASE"
                grasp_counter = 0
                print(f"  [Étape {step_count:4d}] ✓ Dans le bac → Ouverture de la pince...")

        # ── État 7 : Lâcher l'objet ──
        elif state == "RELEASE":
            action[:3] = np.zeros(3)   # Ne pas bouger
            action[-1] = gripper_open  # Ouvrir la pince

            grasp_counter += 1
            if grasp_counter > 30:
                state = "DONE"
                success = True
                print(f"  [Étape {step_count:4d}] ✓ Objet lâché !")

        # ── État 8 : Terminé ──
        elif state == "DONE":
            break

        # Limiter l'amplitude de l'action (sécurité)
        action[:action_dim - 1] = np.clip(action[:action_dim - 1], -1.0, 1.0)

        # Exécuter l'action dans le simulateur
        obs, reward, done, info = env.step(action)
        env.render()
        step_count += 1

        # Limiter le framerate à ~20 FPS
        elapsed = time.time() - start_time
        diff = 1 / 20 - elapsed
        if diff > 0:
            time.sleep(diff)

    # ── 5. Résultat ───────────────────────────────────────────
    print("\n" + "=" * 60)
    if success:
        print("  🎉 PICK & PLACE RÉUSSI ! L'objet rouge est dans le bac !")
    else:
        print("  ⚠️  Temps écoulé. Le robot n'a pas terminé la tâche.")
    print("=" * 60)

    # Garder la fenêtre ouverte quelques secondes pour voir le résultat
    print("\n[INFO] Visualisation pendant 5 secondes...")
    for _ in range(100):
        env.render()
        time.sleep(0.05)

    env.close()
    print("[FIN] Simulation terminée.")


if __name__ == "__main__":
    main()
