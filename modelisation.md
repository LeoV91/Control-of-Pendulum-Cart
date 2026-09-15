# Segway (pendule inversé) — Dimensionnement physique et sélection des équipements

Ce document couvre deux volets :
1. **Dimensionnement physique réel du robot** (mécanique, matériaux, motorisation, électronique, alimentation), avec des choix compatibles MATLAB/Simulink.
2. **Mise en équation mathématique** : équations de la dynamique non linéaire (Lagrangien) intégrant les inerties réelles (roue, pendule), puis linéarisation pour obtenir un modèle linéaire nécessaire à la synthèse d'une loi de controle.




## Partie 1 — Dimensionnement physique réel
Notons que cette première partie n'est pour l'instant qu'une prospection des équipements permettant la réalisation d'un modèle référence. 
Le montage du systeme réel et l'implémentationd des méthodes de controle dévelopées (donc hardware in the loop) fera l'objet d'un document futur.

### Présentation des composants retenus et du systeme

| Composant | Masse estimée | Remarque |
|---|---|---|
| Châssis PETG (2 étages) | ~80 g | À affiner selon la CAO finale |
| 2× moto-réducteur JGA25-370 + roues | ~160 g | 80 g/ensemble |
| Carte Nucleo | ~25 g | |
| IMU MPU6050 | ~5 g | |
| Driver moteur TB6612FNG | ~10 g | |
| Batterie Li-Po 2S 1200 mAh | ~70 g | |
| Câblage, visserie ... | ~50 g | |
| **Total estimé** | **~400-450 g** | Hors marge de conception |

Le systeme total (en considérant une approximation des dimentions et de la disposition des équipements) est représenté schematiquement ci-dessous :
IMAGE ICI !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
![Architecture du projet](images/architecture.png)

La suite de ce document apporte plus de détails concernant le choix de chaque équipement.
### 1. Cahier des charges

- Hauteur totale maximale : **30 cm** (roue comprise)
- Doit tenir debout seul (auto-équilibré), se déplacer en translation
- Électronique facilement pilotable/déployable **depuis MATLAB/Simulink** (génération de code automatique, pas de portage manuel)
- Budget composants raisonnable (prototype hobbyiste/académique)

### 2. Structure mécanique et matériaux

| Élément | Matériau recommandé | Justification |
|---|---|---|
| Châssis principal (2-3 étages) | **PETG imprimé en 3D** | Léger, bonne résistance aux chocs (meilleure que le PLA), facile à itérer/réimprimer si le dimensionnement change, imprimable en atelier standard |
| Entretoises entre étages | Tiges filetées M3 en laiton ou acier + entretoises alu | Rigidité mécanique entre les étages (batterie / carte / IMU), standard en robotique hobby |
| Support moteurs | Plaque aluminium 2-3 mm découpée/usinée, ou renfort imprimé en PETG avec inserts filetés | Les moteurs sont la source de vibration/effort principale |
| Roues | Jante imprimée PETG + bandage caoutchouc (ou roues hobby "TT motor" du commerce, Ø 65-70 mm) | Le caoutchouc est nécessaire pour l'adhérence (hypothèse de roulement sans glissement du modèle) |

### 3. Roues et transmission

- Diamètre roue : **65 mm** (`R = 0.0325 m`) — La vitesse angulaire est modérée et facile à trouver sur le commerce
- Entraînement direct (pas de courroie) : réducteur intégré au moto-réducteur, roue montée directement sur l'axe de sortie → moins de jeu mécanique, améliore la pércision de l'odométrie (mesure de `x`, `x_dot`)

### 4. Motorisation

**Modèle retenu : Moto-réducteurs DC avec encodeur intégré JGA25-370** (12 V, réduction ~1:150, encodeur magnétique quadrature intégré).

IMAGE ICI !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
![Architecture du projet](images/architecture.png)

Une fiche technique du moto réducteur DC JGA25-370 est disponible ci-contre : LIEN ICI !!!!!!!!!!!!!!!!!!!

Avantages de ce choix plutôt qu'un servo ou un moteur brushless :
- L'**encodeur intégré** est indispensable : c'est lui qui donne la mesure de `x` et `x_dot` (odométrie par comptage d'impulsions), nécessaire pour le retour d'état complet (LQR/LQI) qu'on a construit précédemment — sans cette mesure, le sous-espace `[x, x_dot]` n'est pas observable (cf. l'analyse `obsv(A,C)` faite plus tôt dans le projet).
- Un servo continu n'offre pas nativement ce retour de position/vitesse.
- Un moteur brushless nécessiterait un ESC + capteur à effet Hall séparé : plus complexe, sans gain déterminant à notre échelle.


**Dimensionnement rapide du couple :**
Le couple crête doit permettre d'accélérer la masse totale du robot (`M_total ≈ 0.6-0.8 kg` estimé, cf. tableau récapitulatif) à une accélération de rattrapage typique de 2-3 m/s² en cas de perturbation, au rayon de roue choisi :
Le couple crête est le couple maximal atteignable temporairement par le moteur. 
Le choix qui peut sembler arbitraire de l'accélération de rattrapage est une valeur retenue lors de documentation réalisée en amont. 

```
Couple_necessaire ≈ M_total * a_max * R
                  ≈ 0.7 kg * 3 m/s² * 0.0325 m
                  ≈ 0.068 N.m par roue (ordre de grandeur, hors marge de sécurité)
```

Les 2 besoins ci-dessus dimentionant les caractéristiques du moteur orientent vers le choix d'équipement suivant :
Un moto-réducteur JGA25-370 délivre **0.3 à 0.5 N.m** en couple continu selon la réduction choisie. Il y a donc une large marge de sécurité. Cette dernière permet de garantir la couverture des pics dynamiques et des frottements non modélisés.

*NOTA : le choix de ce modèle industriel en particulier plutot qu'un autre concurent présentant les mêmes caractéristiques est basé sur les retours et conseils des utilisateurs des équipements de ce type.*

### 5. Calculateur / carte électronique (interfaçage MATLAB/Simulink)
**Choix technique retenu : STM32 Nucleo**, pour la robustesse temps réel de la boucle d'équilibrage — c'est le facteur le plus critique pour ce système (contrairement à un simple asservissement de position, une gigue de boucle peut ici faire diverger le pendule).

IMAGE ICI !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
![Architecture du projet](images/architecture.png)

Une fiche technique de la carte électronique STM32 Nucleo est disponible ci-contre : LIEN ICI !!!!!!!!!!!!!!!!!!!

La documentation sur le sujet des cartes permettant de transvaser depuis MATLAB/Simulink relativement facilement a mené à trois options :

| Carte | Support Simulink | Avantages | Limites |
|---|---|---|---|
| **STM32 Nucleo (F401RE ou F446RE)** | *Simulink Support Package for STMicroelectronics Nucleo Boards* (officiel MathWorks) | Cadence CPU élevée (84-180 MHz), ADC/timers matériels précis, boucle de contrôle temps réel déterministe. Ces caractéristiques sont mportantes car la bande passante en boucle fermée impose une fréquence d'échantillonnage d'au moins ×5 à ×10 (largement garantie d'après la documentation) | Un peu moins "plug-and-play" qu'Arduino |
| Arduino Mega/Uno | *Simulink Support Package for Arduino Hardware* | Le plus simple à prendre en main, très documenté | Cadence 16 MHz et gestion I2C logicielle : peut devenir limite si IMU (I2C) + 2 encodeurs + 2 PWM moteurs tournent tous à haute fréquence simultanément. Toutefois, de nombreux projets open source similaire utilisent cet équipement ... (à valider expérimentalement) |
| Raspberry Pi (3/4) | *Simulink Support Package for Raspberry Pi Hardware* | Puissance de calcul largement suffisante, permet de l'embarqué plus riche (logs, filtre de Kalman lourd) | Linux non temps-réel strict |


### 6. Capteurs

IMAGE ICI !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
![Architecture du projet](images/architecture.png)

Une fiche technique de l'IMU MPU6050 est disponible ci-contre : LIEN ICI !!!!!!!!!!!!!!!!!!!

- **IMU 6 axes (accéléromètre + gyroscope), type MPU6050** — I2C, bien supporté nativement par les support packages Arduino/Nucleo. Fournit `theta` (par fusion accéléro/gyro, filtre complémentaire ou de Kalman) et `theta_dot` (directement via le gyroscope).
- **Encodeurs quadrature** intégrés aux moto-réducteurs — fournissent `x` (intégration du comptage d'impulsions × périmètre roue / résolution encodeur) et `x_dot` (dérivée ou comptage différentiel par fenêtre de temps).

### 7. Alimentation

- **Batterie Li-Po 2S (7.4 V), 1000-1300 mAh** — bon compromis masse/autonomie pour un robot de cette taille, standard en robotique hobby, alimente directement le driver moteur.

### 8. Driver moteur

**TB6612FNG** (ou DRV8833 ou encore L298N) : bon rendement (peu de perte thermique, MOSFET au lieu de transistors bipolaires), format plus compact, adapté à des moteurs de cette puissance (≈ 0.068 N.m).



### 9. Modélisation du systeme et linéarisation

Ces valeurs remplacent les approximations initiales et alimentent directement la Partie 2 :

```
M (masse roue+moteur, équivalent par roue) ≈ 0.08 kg
m (masse pendule = châssis + carte + batterie, hors roues)  ≈ 0.32 kg
R (rayon roue)                              = 0.0325 m
L (distance axe roue - CG du pendule)       ≈ 0.11 m   (à mesurer sur la CAO finale)
I_w (inertie équivalente des 2 roues, disque plein) ≈ m_wheel * R² ≈ 3.2e-5 kg.m²
I_p (inertie du pendule autour de son CG, approx. tige uniforme)  ≈ (1/12)*m*h²  avec h ≈ 2L
g = 9.81 m/s²
```

> **Important :** `I_w` ci-dessus ne compte que l'inertie mécanique de la roue elle-même. En pratique, l'inertie du **rotor moteur réfléchie à travers le réducteur** (`I_moteur × N_réduction²`) peut être largement dominante et doit être ajoutée une fois le moteur définitivement choisi (valeur à extraire de la fiche technique ou à mesurer expérimentalement) — ne pas se fier uniquement à l'estimation géométrique de la roue seule.

---

## Partie 2 — Mise à plat mathématique : équations et linéarisation

### 1. Coordonnées généralisées et hypothèses

- `x` : position du point de contact roue/sol (roulement sans glissement supposé : `x = R * $\phi$`, `$\phi$` = angle de rotation de la roue)
- `$\theta$` : angle d'inclinaison du pendule par rapport à la verticale ($\theta$ = position d'équilibre haute)
- `$\tau$` : couple moteur net appliqué à la roue (entrée de commande `u`)
- Modèle plan (mouvement 2D dans le plan sagittal), pas de glissement latéral

Position du centre de gravité du pendule :
```
x_p = x + L*sin($\theta$)
y_p = R + L*cos($\theta$)
```

### 2. Énergies cinétique et potentielle (Lagrangien)

**Énergie cinétique de la roue** (translation + rotation propre, φ̇ = ẋ/R) :

> T_roue = ½\theta·M·ẋ² + ½·I_w·(ẋ/R)²

**Énergie cinétique du pendule** (translation du CG + rotation propre) :

> ẋ_p = ẋ + L·cos(θ)·θ̇
> ẏ_p = −L·sin(θ)·θ̇
>
> T_pendule = ½·m·(ẋ_p² + ẏ_p²) + ½·I_p·θ̇²
> T_pendule = ½·m·[ẋ² + 2·L·cos()·ẋ·θ̇ + L²·θ̇²] + ½·I_p·θ̇²

**Énergie potentielle** (hauteur du CG du pendule, terme constant `R` omis) :
```
V = m*g*L*cos(theta)
```

**Lagrangien :**
```
Lg = T_roue + T_pendule - V
```

### 3. Équations d'Euler-Lagrange (modèle non linéaire complet)

Les forces généralisées associées au couple moteur `tau` (appliqué entre la roue et le châssis, donc en réaction sur les deux coordonnées) :
- Sur `x` : `Q_x = tau/R` (force de propulsion transmise au sol par la roue)
- Sur `theta` : `Q_theta = -tau` (couple de réaction du moteur sur le châssis, 3ᵉ loi de Newton)

En appliquant `d/dt(∂Lg/∂q_dot) - ∂Lg/∂q = Q` pour `q = x` puis `q = theta`, et en notant `M_eff = M + m + I_w/R²` :

**Équation 1 (translation) :**
```
M_eff * x_ddot + m*L*cos(theta)*theta_ddot - m*L*sin(theta)*theta_dot^2 = tau/R
```

**Équation 2 (rotation du pendule) :**
```
m*L*cos(theta)*x_ddot + (I_p + m*L^2)*theta_ddot - m*g*L*sin(theta) = -tau
```

(Les termes de Coriolis croisés s'annulent naturellement dans l'équation 2 — vérification de cohérence classique de ce type de dérivation.)

### 4. Forme matricielle et résolution des accélérations

```
[ M_eff            m*L*cos(theta) ] [x_ddot    ]   [ m*L*sin(theta)*theta_dot^2 + tau/R ]
[ m*L*cos(theta)   I_p + m*L^2    ] [theta_ddot] = [ m*g*L*sin(theta) - tau             ]
```

Avec le déterminant :
```
Delta = M_eff*(I_p + m*L^2) - (m*L*cos(theta))^2
```

```
x_ddot     = [ (I_p+m*L^2)*(m*L*sin(theta)*theta_dot^2 + tau/R) - m*L*cos(theta)*(m*g*L*sin(theta) - tau) ] / Delta
theta_ddot = [ M_eff*(m*g*L*sin(theta) - tau) - m*L*cos(theta)*(m*L*sin(theta)*theta_dot^2 + tau/R)        ] / Delta
```

Ce sont les équations à implémenter dans le bloc Simulink non linéaire (remplace le modèle linéaire `A, B` pour la simulation "vérité terrain", à conserver séparément du modèle linéarisé utilisé pour la synthèse des lois de commande).

### 5. Linéarisation autour du point d'équilibre

Point d'équilibre : `theta = 0`, `theta_dot = 0` (position haute), `x` quelconque (mode intégrateur libre, cohérent avec l'analyse de contrôlabilité faite précédemment).

Approximations petits angles : `sin(theta) ≈ theta`, `cos(theta) ≈ 1`, `theta_dot^2 ≈ 0` (terme du second ordre négligé).

Avec `Delta_0 = M_eff*(I_p + m*L^2) - (m*L)^2` (déterminant évalué à `theta=0`) :

```
x_ddot     ≈ [ -(m^2*g*L^2)*theta + ( (I_p+m*L^2)/R + m*L )*tau ] / Delta_0
theta_ddot ≈ [  (M_eff*m*g*L)*theta - ( M_eff + m*L/R )*tau     ] / Delta_0
```

### 6. Matrices A, B mises à jour (avec I_w, I_p, R explicites)

Avec l'état `X = [x, x_dot, theta, theta_dot]'` et l'entrée `u = tau` :

```matlab
M_eff = M + m + I_w/R^2;
Delta0 = M_eff*(I_p + m*L^2) - (m*L)^2;

A = [0                                  1   0                                0;
     0                                  0   -(m^2*g*L^2)/Delta0              0;
     0                                  0   0                                1;
     0                                  0   (M_eff*m*g*L)/Delta0             0];

B = [0;
     ((I_p+m*L^2)/R + m*L)/Delta0;
     0;
     -(M_eff + m*L/R)/Delta0];

C = [1 0 0 0;    % mesure x  (odométrie encodeurs)
     0 0 1 0];   % mesure theta (IMU)
```

### 7. Comparaison avec le modèle simplifié précédent

Le modèle initial du projet (avec `R`, `I_w`, `I_p` mis en commentaire, donc implicitement nuls) correspondait à une **approximation masse ponctuelle** : pendule et roue traités sans inertie propre, seule la masse comptait. Cette nouvelle dérivation :

- Réintroduit `I_w` (inertie des roues + rotor moteur réfléchi), ce qui **augmente la masse effective `M_eff`** vue par l'actionneur — le système réel demandera un peu plus de couple que ne le laissait penser le modèle simplifié.
- Réintroduit `I_p` (inertie du pendule autour de son propre CG, pas seulement autour de l'axe des roues), ce qui **modifie légèrement la fréquence naturelle d'instabilité** du pendule (le pôle instable identifié précédemment, `sqrt(6g(M+m)/(L(4M+m)))`, doit être recalculé avec cette formule mise à jour une fois les valeurs numériques de `I_w`, `I_p` figées).
- Ne change pas la structure qualitative du système (toujours 1 pôle instable, 1 mode intégrateur libre sur `x`) : toutes les analyses de contrôlabilité/observabilité/marges faites précédemment restent valables dans leur principe, seules les valeurs numériques changent.

---

## Prochaines étapes suggérées

1. Finaliser la CAO du châssis pour obtenir `L`, `m`, `I_p` précis (au lieu des estimations analytiques ci-dessus).
2. Récupérer la fiche technique du moto-réducteur retenu pour `I_moteur` réfléchi et l'ajouter à `I_w`.
3. Recalculer `A, B, C` numériquement avec ces valeurs (script MATLAB à partir des formules de la Partie 2, en remplacement du script initial).
4. Revalider `rank(ctrb(A,B))` et `rank(obsv(A,C))` avec les nouvelles matrices — la conclusion structurelle ne devrait pas changer, mais c'est une vérification de non-régression utile.
5. Relancer la synthèse LQR/LQI/PID avec ces paramètres mis à jour et comparer aux résultats précédents.
