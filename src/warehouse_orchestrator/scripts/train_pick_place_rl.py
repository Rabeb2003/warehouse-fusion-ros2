"""
Script d'entraînement RL (Reinforcement Learning) avec Stable-Baselines3 et Robosuite.
Entraîne un agent PPO / SAC sur la tâche Pick & Place du robot Panda (canette rouge).

Utilisation :
  1. Entraîner le modèle :
     python train_pick_place_rl.py --train --timesteps 50000
  2. Évaluer et visualiser le modèle entraîné :
     python train_pick_place_rl.py --eval
"""

import argparse
import os
import time
import numpy as np

import robosuite as suite
from robosuite.wrappers import GymWrapper

try:
    from stable_baselines3 import PPO, SAC
    from stable_baselines3.common.callbacks import CheckpointCallback
    from stable_baselines3.common.monitor import Monitor
except ImportError:
    print("[ERREUR] stable-baselines3 n'est pas encore installé dans le venv !")
    exit(1)


MODEL_DIR = "/home/rabeb/warehouse_fusion_ws/src/warehouse_orchestrator/rl_models"
MODEL_PATH = os.path.join(MODEL_DIR, "panda_pick_place_ppo")


def make_robosuite_env(has_renderer=False):
    """Crée l'environnement Robosuite encapsulé pour Gymnasium."""
    raw_env = suite.make(
        env_name="PickPlaceCan",       # Tâche : attraper la canette rouge
        robots="Panda",
        has_renderer=has_renderer,
        has_offscreen_renderer=False,
        use_camera_obs=False,
        use_object_obs=True,           # Position de l'objet et du robot
        reward_shaping=True,           # Récompense guidée
        control_freq=20,
        horizon=500,
        ignore_done=False,
    )
    # Convertir en interface Gym / Gymnasium
    env = GymWrapper(raw_env)
    return env


def train(timesteps=50000, algo="PPO"):
    """Entraîne l'agent RL."""
    print("=" * 60)
    print(f"  ENTRAÎNEMENT RL - Algorithme: {algo} | Timesteps: {timesteps}")
    print("=" * 60)

    os.makedirs(MODEL_DIR, exist_ok=True)

    # Créer l'environnement d'entraînement (headless)
    env = make_robosuite_env(has_renderer=False)

    print(f"[INFO] Espace d'observation : {env.observation_space}")
    print(f"[INFO] Espace d'action      : {env.action_space}")

    # Configuration du modèle
    if algo.upper() == "PPO":
        model = PPO(
            "MlpPolicy",
            env,
            verbose=1,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            tensorboard_log=os.path.join(MODEL_DIR, "tensorboard"),
        )
    elif algo.upper() == "SAC":
        model = SAC(
            "MlpPolicy",
            env,
            verbose=1,
            learning_rate=3e-4,
            buffer_size=50000,
            batch_size=256,
            tensorboard_log=os.path.join(MODEL_DIR, "tensorboard"),
        )
    else:
        raise ValueError(f"Algorithme inconnu : {algo}")

    # Callback pour sauvegarder des checkpoints réguliers
    checkpoint_callback = CheckpointCallback(
        save_freq=10000,
        save_path=MODEL_DIR,
        name_prefix=f"panda_pick_place_{algo.lower()}_ckpt",
    )

    print(f"\n[START] Lancement de l'apprentissage sur {timesteps} pas...")
    start_time = time.time()
    
    model.learn(total_timesteps=timesteps, callback=checkpoint_callback)
    
    elapsed = time.time() - start_time
    print(f"\n[INFO] Entraînement terminé en {elapsed / 60:.2f} minutes !")

    # Sauvegarder le modèle final
    model.save(MODEL_PATH)
    print(f"[SUCCESS] Modèle sauvegardé dans : {MODEL_PATH}.zip")

    env.close()


def evaluate(num_episodes=5):
    """Évalue et visualise le modèle entraîné."""
    print("=" * 60)
    print("  ÉVALUATION ET VISUALISATION DU MODÈLE ENTRAÎNÉ")
    print("=" * 60)

    if not os.path.exists(MODEL_PATH + ".zip"):
        print(f"[ERREUR] Aucun modèle trouvé à {MODEL_PATH}.zip")
        print("Veuillez d'abord entraîner un modèle avec l'option --train")
        return

    # Charger le modèle
    print(f"[INFO] Chargement du modèle depuis {MODEL_PATH}.zip...")
    model = PPO.load(MODEL_PATH)

    # Créer l'environnement avec rendu graphique
    env = make_robosuite_env(has_renderer=True)

    for ep in range(num_episodes):
        print(f"\n--- Épisode {ep + 1}/{num_episodes} ---")
        obs, info = env.reset()
        done = False
        total_reward = 0.0
        step_count = 0

        while not done and step_count < 500:
            start_time = time.time()
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            total_reward += reward
            step_count += 1
            env.env.render()

            elapsed = time.time() - start_time
            diff = 1 / 20 - elapsed
            if diff > 0:
                time.sleep(diff)

        print(f"  Récompense totale : {total_reward:.2f} | Pas : {step_count}")

    env.close()
    print("\n[FIN] Évaluation terminée.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Entraînement RL Pick & Place avec Robosuite")
    parser.add_argument("--train", action="store_true", help="Lancer l'entraînement")
    parser.add_argument("--eval", action="store_true", help="Lancer l'évaluation avec rendu 3D")
    parser.add_argument("--timesteps", type=int, default=50000, help="Nombre de timesteps d'entraînement")
    parser.add_argument("--algo", type=str, default="PPO", choices=["PPO", "SAC"], help="Algorithme RL")

    args = parser.parse_args()

    if args.eval:
        evaluate()
    else:
        # Par défaut, si aucun drapeau n'est spécifié ou si --train est passé
        train(timesteps=args.timesteps, algo=args.algo)
