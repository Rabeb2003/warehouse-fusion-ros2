# Modèle Cinématique - Robot Différentiel avec Remorque

## Différence entre Skid-Steer et Différentiel

### Skid-Steer (Husky)
- Les roues tournent à des vitesses différentes pour créer un virage
- Glissement latéral présent
- Modèle cinématique plus complexe avec facteurs de glissement
- Rayon de virage minimal plus petit

### Différentiel (Votre nouveau robot)
- Les roues tournent à des vitesses différentes sans glissement latéral
- Virage plus prévisible et précis
- Modèle cinématique standard
- Rayon de virage minimal plus grand

## Modèle Cinématique Adapté

### Équations du mouvement

Le modèle cinématique de la remorque passive reste le même pour les deux types:

```
β̇ = -ω - (v/d) * sin(β)
```

Où:
- β = angle d'attelage (angle entre le robot et la remorque)
- ω = vitesse angulaire du robot
- v = vitesse linéaire du robot
- d = distance de l'attelage à l'essieu de la remorque

### Pose de la remorque

```
θ_T = θ_r + β
x_T = x_r + hitch_offset * cos(θ_r) + d * cos(θ_T)
y_T = y_r + hitch_offset * sin(θ_r) + d * sin(θ_T)
```

Où:
- θ_r = orientation du robot
- θ_T = orientation de la remorque
- hitch_offset = décalage du centre du robot à l'attelage
- (x_r, y_r) = position du robot
- (x_T, y_T) = position de la remorque

### Rayon de virage minimal

```
R_min = d / sin(β_lim)
```

Avec les paramètres actuels:
- d = 0.50 m
- β_lim = 45°
- R_min = 0.50 / sin(45°) = 0.50 / 0.707 = 0.707 m

### Vitesse angulaire maximale

```
ω_max = v / R_min = v * sin(β_lim) / d
```

Exemple pour v = 0.26 m/s:
- ω_max = 0.26 * sin(45°) / 0.50 = 0.26 * 0.707 / 0.50 = 0.368 rad/s

## Adaptations dans le Code

### 1. cmd_vel_safety_node.py
- Utilise le même modèle cinématique β̇ = -ω - (v/d) * sin(β)
- Paramètres adaptés: d=0.50m, β_lim=45°, v_max=0.26m/s
- Ajout de réduction adaptive de ω selon β

### 2. trailer_aware_controller_node.py
- Cinématique inverse adaptée pour différentiel
- Correction proportionnelle pour réduire β
- Réduction de vitesse selon β

### 3. dynamic_footprint_node.py
- Modèle de footprint par disques
- Calcul de la pose de la remorque adapté
- Paramètres géométriques adaptés

## Paramètres Spécifiques au Différentiel

### Avantages du différentiel
1. **Pas de glissement latéral**: Virages plus précis
2. **Modèle plus simple**: Pas besoin de facteurs de glissement
3. **Meilleure estimation**: L'odométrie est plus précise
4. **Contrôle plus stable**: Moins d'oscillations

### Inconvénients
1. **Rayon de virage plus grand**: R_min plus élevé
2. **Manœuvres plus limitées**: Virages serrés plus difficiles
3. **Vitesse plus lente**: Nécessite de réduire v pour les virages

## Comparaison des Paramètres

| Paramètre | Husky (Skid-Steer) | Différentiel |
|-----------|-------------------|--------------|
| d (attelage) | 0.80 m | 0.50 m |
| β_lim | 70° | 45° |
| R_min | 0.851 m | 0.707 m |
| v_max | 0.40 m/s | 0.26 m/s |
| ω_max (stationnaire) | 0.47 rad/s | 1.0 rad/s |

## Intégration avec Nav2

### Smac Planner (State Lattice)
- Motion model: "DIFF_DRIVE" (pas "ACKERMANN")
- minimum_turning_radius: 0.80 m (plus grand que R_min pour marge)
- reverse_penalty: 5.0 (forte pénalité pour éviter jackknife)

### MPPI Controller
- motion_model: "DiffDrive"
- PreferForwardCritic: cost_weight=100.0 (évite marche arrière)
- vx_max: 0.26 m/s
- wz_max: 1.0 rad/s

## Conclusion

Le modèle cinématique de base reste le même, mais les paramètres sont adaptés pour le robot différentiel:
- Distance d'attelage plus courte (0.50 m vs 0.80 m)
- Limite β plus restrictive (45° vs 70°)
- Vitesse plus lente (0.26 m/s vs 0.40 m/s)
- ω_max stationnaire plus élevé (1.0 rad/s vs 0.47 rad/s)

Ces adaptations assurent que le système reste stable et sécurisé pour le robot différentiel avec remorque.
