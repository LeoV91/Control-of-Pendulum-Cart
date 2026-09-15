# Segway (pendule inversé) — Modélisation du système

Ce document couvre la **Mise en équation mathématique** : équations de la dynamique non linéaire (Lagrangien) intégrant les inerties réelles (roue, pendule) du systeme. Puis la linéarisation de ce dernier pour obtenir un modèle linéaire nécessaire à la synthèse d'une loi de controle.


## 1. Coordonnées généralisées et hypothèses

- `x` : position du point de contact roue/sol (roulement sans glissement supposé : `x = R * $\phi$`, `$\phi$` = angle de rotation de la roue)
- `$\theta$` : angle d'inclinaison du pendule par rapport à la verticale ($\theta$ = position d'équilibre haute)
- `$\tau$` : couple moteur net appliqué à la roue (entrée de commande `u`)
- Modèle plan (mouvement 2D dans le plan sagittal), pas de glissement latéral

Position du centre de gravité du pendule :
```
x_p = x + L*sin($\theta$)
y_p = R + L*cos($\theta$)
```

Les variables présentées c-dessus sont schématisées dans le graphique suivant :

IMAGE ICI !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
![Architecture du projet](images/architecture.png)

## 2. Énergies cinétique et potentielle (Lagrangien)

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

## 3. Équations d'Euler-Lagrange (modèle non linéaire complet)

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

## 4. Forme matricielle et résolution des accélérations

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

## 5. Linéarisation autour du point d'équilibre

Point d'équilibre : `theta = 0`, `theta_dot = 0` (position haute), `x` quelconque (mode intégrateur libre, cohérent avec l'analyse de contrôlabilité faite précédemment).

Approximations petits angles : `sin(theta) ≈ theta`, `cos(theta) ≈ 1`, `theta_dot^2 ≈ 0` (terme du second ordre négligé).

Avec `Delta_0 = M_eff*(I_p + m*L^2) - (m*L)^2` (déterminant évalué à `theta=0`) :

```
x_ddot     ≈ [ -(m^2*g*L^2)*theta + ( (I_p+m*L^2)/R + m*L )*tau ] / Delta_0
theta_ddot ≈ [  (M_eff*m*g*L)*theta - ( M_eff + m*L/R )*tau     ] / Delta_0
```

## 6. Matrices A, B mises à jour (avec I_w, I_p, R explicites)

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

## 7. Comparaison avec le modèle simplifié précédent

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
