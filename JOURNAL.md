# Journal de développement – téléop clavier FR3

## 2026-11-03 — Contrôleur clavier cartésien avec IK (`keyboard_ik.py`)

### Objectif
Déplacer le bout de pince (`fr3_hand_tcp`) vers un point cible piloté au clavier :
z/s = ±X, q/d = ±Y (repère base `fr3_link0`), r/f = haut/bas. Pince toujours vers le bas.

### Choix de conception (validés avec l'utilisateur)
| Question | Choix |
|---|---|
| Où tourne l'IK | Script Python (comme `keyboard_joint1.py`), envoie des vitesses articulaires |
| Plateforme | Gazebo d'abord |
| Orientation | Fixe, pince vers le bas (IK 6D) |
| Pilotage | En vitesse, touche maintenue |
| Repère | Base du robot |
| Point piloté | TCP de la pince (10,34 cm sous la bride) |

Décisions prises par défaut :
- **Pas de nouveau contrôleur ros2_control** : `keyboard_joint1_controller` est déjà un
  `JointGroupVelocityController` sur les 7 joints → réutilisé tel quel, aucune modif de YAML.
- **Pas de KDL/MoveIt** : FK + jacobienne calculées en numpy à partir des paramètres DH
  (Craig modifié) officiels du FR3. Zéro dépendance en plus de numpy.
- **IK différentielle** : moindres carrés amortis (DLS, λ = 0,05) + espace nul qui tire vers
  la pose « ready » pour que le coude (7e DDL redondant) ne dérive pas.
- **Sécurité** : rampe sur la vitesse cible (0,3 m/s²) et sur les vitesses articulaires
  (2 rad/s²), plafond 0,8 rad/s par joint, arrêt d'un joint à 0,1 rad de sa butée,
  espace de travail borné (z ≥ 5 cm, ≥ 25 cm de l'axe de la base, ≤ 80 cm de l'épaule),
  la cible ne peut pas s'éloigner de plus de 5 cm du TCP réel.
- Vitesse ajustable avec **+ / −** (z et s étant pris par le déplacement).

### Fichiers
- Créé : `franka_bringup/scripts/keyboard_ik.py`
- Créé : `JOURNAL.md` (ce fichier)

### Utilisation
```
ros2 launch franka_gazebo_bringup gazebo_franka_arm_example_controller.launch.py \
    controller:=keyboard_joint1_controller load_gripper:=true
python3 src/franka_bringup/scripts/keyboard_ik.py
```
⚠ Le focus clavier doit être sur **le terminal du script** (il lit `stdin`). Dans RViz,
ZQSD déplace la caméra et n'atteint pas le robot. Capture clavier globale impossible
simplement (Wayland + Docker).

Auto-test (dans le conteneur) : `python3 keyboard_ik.py test` → `ok`

### Problèmes rencontrés et résolutions

**1. Pose initiale du robot inconnue dans Gazebo**
- Constat : aucune `initial_value` / pose initiale trouvée dans `franka_description` ni
  `franka_gazebo` → le bras démarre vraisemblablement à q = 0 (bras tendu vertical).
  C'est une singularité, et q4 = 0 et q6 = 0 sont hors des limites articulaires FR3.
- Résolution : le script commence par une phase de **homing** en espace articulaire vers
  la pose « ready » `[0, -π/4, 0, -3π/4, 0, π/2, π/4]`, puis verrouille la cible sur la
  position du TCP et active le clavier. Le blocage aux butées n'empêche que de s'enfoncer
  davantage : revenir dans les limites reste autorisé.

**2. Impossible de lancer l'auto-test sur l'hôte : `ModuleNotFoundError: No module named 'rclpy'`**
- Cause : ROS 2 n'est installé que dans le conteneur Docker, pas sur la machine hôte.
- Résolution : test des maths seules sur l'hôte en remplaçant `rclpy`, `sensor_msgs` et
  `std_msgs` par des modules vides. Résultat : `ok` (FK = pose TCP connue du FR3
  (0,307 ; 0 ; 0,487), jacobienne = différences finies, IK converge à < 1 mm d'une cible
  décalée de 10 cm sur les 3 axes en gardant la pince vers le bas).

**3. Au lancement de la sim : « ruby 3.2 crash » (Gazebo), RViz meurt aussi**
- Log : `rviz2: Authorization required, but no authorization protocol specified` /
  `could not connect to display :0` ; `gazebo-1` se termine aussitôt, puis le `spawner`
  meurt au bout de 30 s (pas de controller_manager, puisque Gazebo n'existe pas).
  Le « ruby » vient de la commande `gz` (CLI en Ruby) qui lance la GUI Gazebo.
- Cause : l'hôte est sous Wayland (XWayland sur `:0`) avec contrôle d'accès X actif et
  aucun client autorisé → le conteneur (uid 1000) ne peut pas ouvrir de fenêtre.
  Sans rapport avec `keyboard_ik.py`.
- Résolution : sur l'hôte, `xhost +SI:localuser:nathan`. N'autorise que l'utilisateur
  local uid 1000 (= user du conteneur), contrairement à `xhost +` qui ouvre à tous.
  Vérifié : `rviz2` dans le conteneur s'ouvre sans erreur d'autorisation.
- ⚠ Ce réglage est perdu à chaque nouvelle session graphique : relancer la commande
  après chaque reconnexion/redémarrage, avant `docker compose up`.

**4. « Le robot ne bouge pas quand j'appuie sur z »**
- Constat : sim OK (`gz sim` tourne), `keyboard_joint1_controller` actif, mais
  `/keyboard_joint1_controller/commands` avait 0 publisher → aucun script clavier en cours.
- Test : `keyboard_ik.py` piloté depuis un pseudo-terminal dans le conteneur (envoi de `z`
  toutes les 50 ms pendant 3 s). Résultat : TCP (0,307 ; 0 ; 0,487) → (0,469 ; 0 ; 0,487),
  pince toujours vers le bas. Le script fonctionne : +16 cm en X, Z constant.
- Pose de départ Gazebo : déjà en pose « ready » (et non q = 0 comme supposé au point 1) ;
  le homing se termine donc immédiatement.
- Causes probables côté utilisateur : `keyboard_joint1.py` lancé au lieu de `keyboard_ik.py`
  (z y règle la vitesse) ; script lancé sur l'hôte (pas de rclpy) ; focus clavier sur la
  fenêtre Gazebo au lieu du terminal ; appui avant « pose ready atteinte ».

### Réécriture « style étudiant » de `keyboard_ik.py`
Demande : code lisible par un étudiant en robotique. Le comportement est inchangé, avec
`VITESSE_MAX = 0.30` m/s (modifié par l'utilisateur).
- Noeud ROS 2 sous forme de classe `TeleopIK(Node)` avec un `create_timer` à 200 Hz, au
  lieu d'une boucle `while` + `select` avec timeout. `select` est maintenant appelé avec
  un timeout de 0 et sert seulement à savoir si une touche est disponible.
- Une fonction par notion de cours : `matrice_dh`, `cinematique_directe` (FK +
  jacobienne), `erreur_orientation`, `cinematique_inverse` (DLS + espace nul),
  `limiter_vitesses`, `limiter_espace_travail`, `rampe`.
- Boucles explicites au lieu des écritures numpy compactes, noms et commentaires en
  français, paramètres regroupés et commentés en tête de fichier.
- Vérifié : auto-test `ok` dans le conteneur.
- Non vérifié : test réel dans Gazebo, car la sim était arrêtée au moment du test.

## 2026-11-04 — Commande cinématique par UDP (`udp_ik.py`)

### Objectif
La pince suit une pose 6D envoyée en continu (≥ 50 Hz) par UDP depuis un programme Python.
D'abord sur le même PC, en Wi-Fi plus tard. Code minimal, sans watchdog (demande explicite).

### Choix de conception (validés avec l'utilisateur)
| Question | Choix |
|---|---|
| Émetteur | Programme Python de l'utilisateur |
| Contenu | Pose complète 6D de la pince |
| Orientation | Quaternion |
| Encodage | Binaire, 7 doubles little-endian |
| Retour d'état | Aucun (sens unique) |
| Organisation | Nouveau script séparé, qui réutilise la cinématique de `keyboard_ik.py` |

**Format d'un paquet** : `struct.pack('<7d', x, y, z, qx, qy, qz, qw)`, soit 56 octets.
La position est celle de `fr3_hand_tcp` en mètres, dans le repère `fr3_link0`, et
l'ordre (x, y, z, w) du quaternion est celui de ROS. Pince vers le bas = `(1, 0, 0, 0)`.

Décisions prises par défaut :
- Port **5005**, écoute sur `0.0.0.0` : le passage au Wi-Fi ne demande aucune modif côté robot.
- Seul le dernier paquet reçu compte (la file est vidée à chaque tour de 200 Hz).
- Paquet de mauvaise taille, valeur non finie ou quaternion nul → ignoré.
- Quaternion normalisé à la réception, position ramenée dans l'espace de travail.
- Avant le premier paquet, le robot ne bouge pas. Si le flux s'arrête, il reste sur la
  dernière pose reçue : c'est une consigne de position, il ne dérive pas.
- Pas d'anticipation de vitesse : le suivi a donc un léger retard (gain KP = 4 /s). À
  ajouter (dérivée des poses reçues) si le retard gêne.
- `cinematique_inverse()` de `keyboard_ik.py` prend maintenant une orientation cible
  optionnelle (par défaut, pince vers le bas : le clavier est inchangé).

### Utilisation
Il faut **deux terminaux** dans le conteneur : le récepteur doit tourner en permanence.
```
# terminal 1 : récepteur, reste ouvert
python3 /ros2_ws/src/franka_bringup/scripts/udp_ik.py
# terminal 2 : envoi d'une pose de test
python3 /ros2_ws/src/franka_bringup/scripts/udp_ik.py send 0.4 0.1 0.4 1 0 0 0
```
Côté émetteur, il suffit de :
```python
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(struct.pack('<7d', x, y, z, qx, qy, qz, qw), ('127.0.0.1', 5005))
```
⚠ Une seule instance à la fois, sinon `OSError: Address already in use` (port déjà pris).
⚠ Ne pas lancer `udp_ik.py` et `keyboard_ik.py` en même temps : ils publient sur le même
topic de commande.

### Problèmes rencontrés et résolutions

**5. L'IK 6D s'arrête à 1,2 mm de la cible (auto-test en échec)**
- Constat : avec une pince inclinée de 30°, l'erreur reste à 1,18 mm. Ce n'est pas une
  butée (marge mini 0,32 rad) ni une singularité (plus petite valeur singulière 0,2).
  Avec `K_POSTURE = 0`, l'erreur tombe à 0.
- Cause : le projecteur sur l'espace nul était calculé avec la pseudo-inverse **amortie**.
  Il n'est alors pas exact, et le rappel vers la pose ready fuit dans la tâche.
- Résolution : `projecteur = I - pinv(J) @ J` avec la vraie pseudo-inverse. La DLS reste
  utilisée pour la tâche principale. Les deux auto-tests passent.
- Le bug existait aussi dans `keyboard_ik.py` (erreur résiduelle d'~1 mm, invisible au
  clavier) : il est corrigé dans les deux scripts, puisque le code est partagé.

**6. Pièges rencontrés en testant (outillage, pas le code du robot)**
- Le conteneur était arrêté → `docker start franka_ros2_jazzy`.
- `kill -INT` ne stoppe pas un processus lancé en arrière-plan par un shell non interactif
  (SIGINT y est ignoré). L'instance restante gardait le port 5005, d'où
  `Address already in use` au test suivant.
- `pkill -f udp_ik.py` tue aussi le `bash -c` qui contient ce texte dans sa ligne de
  commande. Il faut cibler `pkill -f '^python3 .*udp_ik'`.
- L'erreur `context is not valid` vue à la fin d'un test venait de cet arrêt brutal, pas
  du script. Avec un vrai Ctrl+C dans un terminal, l'arrêt est propre (code de sortie 0).

**7. « Le robot simulé ne réagit pas à `udp_ik.py send ...` »**
- Constat : sim et contrôleur actifs, mais 0 publisher sur le topic de commande : aucun
  récepteur `udp_ik.py` ne tournait. `send` ne fait qu'émettre un paquet. Sans personne
  qui écoute, l'UDP le perd sans aucune erreur (pas de connexion, pas d'accusé de réception).
- Cause : mes instructions mettaient les deux commandes l'une sous l'autre, sans préciser
  que le récepteur doit tourner en parallèle dans un autre terminal.
- Résolution : doc du script et du journal corrigées (« deux terminaux »).
- Vérifié sur la sim en cours : récepteur lancé, puis `send 0.4 0.1 0.4 1 0 0 0` → pince
  de (0,307 ; 0 ; 0,487) à (0,400 ; 0,100 ; 0,400), orientation vers le bas conservée.

### Tests réalisés
- Auto-tests `udp_ik.py test` et `keyboard_ik.py test` : `ok`. Ils couvrent la conversion
  quaternion → matrice, le décodage des paquets bons et mauvais, et l'IK 6D avec une pince
  inclinée de 30°, qui converge à moins d'1 mm.
- Chaîne complète sans Gazebo : faux `/joint_states` + paquet UDP → vitesses non nulles
  publiées sur `/keyboard_joint1_controller/commands`, plafonnées à 0,8 rad/s. Vitesses
  nulles avant le premier paquet, paquet de 3 octets ignoré, Ctrl+C propre.

## 2026-11-05 — IK par optimisation (QP) avec scipy

### Objectif
Remplacer « pseudo-inverse amortie + espace nul + limites appliquées après coup » par une
optimisation sous contraintes, utilisée par le clavier et par l'UDP. Pas de contrainte
de table pour l'instant (demande utilisateur).

### Formulation (`cinematique_inverse_qp` dans `keyboard_ik.py`)
```
min  ‖J q̇ − v_voulue‖² + λ² ‖q̇‖² + μ ‖q̇ − K_POSTURE (q_ready − q)‖²
s.c. −q̇_max ≤ q̇ ≤ q̇_max                                       vitesse
     q̇_prec − q̈_max·dt ≤ q̇ ≤ q̇_prec + q̈_max·dt               accélération
     q̇ ≤ GAIN_BUTEE·(q_max − marge − q), idem côté q_min      butées (amortisseur)
```
- Toutes les contraintes sont des bornes sur q̇ → moindres carrés bornés, résolus par
  `scipy.optimize.lsq_linear(method='bvls')`. scipy 1.11.4 était déjà installé dans le
  conteneur (osqp, quadprog, proxsuite et cvxpy absents, pip non installé).
- Si les bornes de butée et d'accélération sont incompatibles, l'accélération l'emporte :
  le joint freine au maximum plutôt que de s'arrêter instantanément.
- Avec GAIN_BUTEE = 2 /s, un joint lancé à 0,8 rad/s vers une butée décélère d'au plus
  1,6 rad/s², sous le max de 2 rad/s² : les deux contraintes restent compatibles en approche normale.
- Bascule `METHODE_IK = 'qp' | 'dls'` en tête de fichier ; l'ancienne méthode est conservée.
- `cinematique_inverse()` prend maintenant `q_point_precedent` (nécessaire aux bornes
  d'accélération). Appels mis à jour dans les deux scripts.
- Nouvelle fonction `simuler()` (robot idéal) pour les auto-tests, partagée avec `udp_ik.py`.

### Choix du poids de posture μ (mesuré)
La posture n'est plus strictement prioritaire, c'est une tâche pondérée : elle tire un peu
la pince. Erreur finale de la pince selon μ (λ² = 0,0025) :

| μ | Pince vers le bas | Pince inclinée de 30° |
|---|---|---|
| 0,01 | 2,30 mm | 4,62 mm |
| 0,0025 | 0,58 mm | 1,18 mm |
| **0,0003 (retenu)** | **0,07 mm** | **0,15 mm** |

L'écart du coude à la pose ready reste du même ordre dans tous les cas : la posture est
toujours tenue. Temps de calcul complet (FK + jacobienne + QP) : **0,24 ms** par pas,
pour un budget de 5 ms.

### Problèmes rencontrés et résolutions

**8. Le premier test de butée ne testait rien**
- Constat : le test cherchait à amener le joint 4 contre sa butée avec une cible hors
  d'atteinte. Le joint s'éloignait en fait de sa butée (q4 → −0,47 pour une limite à −0,25).
  Un deuxième essai, avec une cible derrière le robot pour forcer le joint 1, n'a pas
  marché non plus : le bras se replie avec les autres joints, et q1 ne dépasse pas 2,2.
- Résolution : tester directement la contrainte. Joint 1 posé sur sa marge, tâche qui le
  pousse à fond vers la butée → q̇1 ≤ 0. Lancé à 0,5 rad/s → freinage au maximum
  (q̇1 = 0,49). Sans la borne de butée, ce test échoue.
- Constat utile au passage, sur cette même cible : la DLS demande jusqu'à **6,4 rad/s²**
  avant la rampe, alors que le QP respecte 2,0 rad/s² par construction. Erreur finale de
  la pince : 0,4 mm en QP, 3,2 mm en DLS.

### Tests réalisés
- `keyboard_ik.py test` et `udp_ik.py test` : `ok`. Ils couvrent :
  - la convergence des deux méthodes ;
  - le respect de la vitesse et de l'accélération sur tout un mouvement ;
  - la butée à l'arrêt et en mouvement ;
  - l'IK 6D avec la pince inclinée.
- Non testé dans Gazebo : la sim était arrêtée.

## 2026-11-09 — Refonte inspirée de `delay-robust-teleop-6dof` (projet S7)

Source : https://github.com/NathanVayva/delay-robust-teleop-6dof (commit 2b10964). Cloné
dans un dossier temporaire et lu entièrement : `rob_lib.cpp/.hpp`, `main.cpp`, serveur et
client UDP, `delay.cpp`. Rien n'a été exécuté.

### Ce qui a été repris (choix validés avec l'utilisateur)
**1. Notations du dépôt (`keyboard_ik.py`)**
| Avant | Maintenant (comme `rob_lib.cpp`) |
|---|---|
| constantes `DH` (a, d, alpha) | `DH_robot(alpha, d, r)` (Khalil-Kleinfinger), `Robot(DH, V_max, a_max, q_min, q_max)`, instance `FR3` |
| `matrice_dh` + `cinematique_directe` | `MGD(theta, DH)` → `(T, MT)`, et `Jacobienne(theta, DH, P)` |
| pince = transformation ajoutée à part | 8e ligne DH « outil » (θ fixe = −45°) + `Rob2Model(q)` qui l'ajoute |
| `erreur_orientation` | `calcul_epsilon0(Ad, Ae)` |
| ω = K0 ε₀ | `calcul_L(Ad, Ae)` + `we = L_amortie (K0 ε₀ + Lᵀ wd)` |
| `cinematique_inverse(q, …)` | `cmd_cinematique(robot, q, dq_prec, Pd, Ad, dPd, wd, Kp, K0, lambda_L)` |
| `KP_POSITION`, `KP_ORIENTATION` | `KP`, `K0`, `LAMBDA_L = 0.1` (valeur du dépôt) |
| `VITESSE/ACCEL_ARTICULAIRE_MAX` | `FR3.V_max`, `FR3.a_max` (vecteurs par joint) |

Différences volontaires avec le dépôt :
- **`calcul_L`** : dans le dépôt elle renvoie l'identité (version simplifiée). Ici, c'est
  la vraie formule du cours, `L = −½ Σ S(d_i) S(e_i)`. Vérifié : L = I quand Ae = Ad, et
  dε₀/dt = −L·we par différences finies.
- **Zone morte supprimée** : le dépôt fait `we = wd` si ‖ε₀‖ < 1e-2, ce qui laisse jusqu'à
  0,6° d'erreur d'orientation.
- **`cmd_cinematique` calcule un seul pas**, appelé à 200 Hz par le timer ROS, au lieu de
  boucler jusqu'à convergence.

**2. Protocole UDP (`udp_ik.py`)**, comme `struct mesg` du dépôt
- Commande `'<Qq13d'` (120 octets) : `label`, `time_send` (µs), Pd(3), quaternion(4), dPd(3), wd(3).
- Retour `'<Qq7d'` (72 octets), un par commande reçue : `label` et `time_send` recopiés,
  plus Pe(3) et quaternion(4) mesurés. L'émetteur calcule RTT = maintenant − time_send.
- `label` : une commande dont le label n'est pas plus grand que le dernier appliqué est
  ignorée, comme `if (message.label > last_message_label)` dans `server_UDP.cpp`.
- Anticipation `dPd`, `wd` transmise jusqu'à `cmd_cinematique`.
- Ajout de `matrice_vers_quaternion` (méthode de Shepperd), pour le retour.

### Ce qui n'a pas été repris, et pourquoi
- **Gain K adaptatif selon le RTT** (`calcule_K_Adaptatif`). Dans le dépôt, la boucle
  d'asservissement passe par le réseau (client → robot → mesure → client), donc le retard
  la déstabilise. Ici, la boucle est fermée localement à 200 Hz sur le PC du robot. Le
  réseau ne transporte que la consigne : un retard la décale dans le temps sans rien
  déstabiliser. Le gain adaptatif deviendra utile si l'émetteur ferme sa propre boucle
  sur le retour d'état (ex. : asservissement visuel côté émetteur).
- **Émulateur de retard** (`delay.cpp`) : non demandé pour l'instant.

### Problèmes rencontrés et résolutions

**9. Un émetteur qui redémarre serait ignoré pour toujours**
- Constat (aussi présent dans le dépôt) : `last_message_label` ne redescend jamais. Un
  client relancé repart à label 1, et toutes ses commandes sont rejetées.
- Résolution : après `DELAI_REDEMARRAGE = 1 s` sans commande acceptée, on accepte de
  nouveau n'importe quel label. Testé dans `accepter_label`.

**10. L'anticipation fait dériver le robot quand le flux s'arrête**
- Constat (raisonnement) : la dernière commande garde son dPd. Si le flux s'arrête, Pd
  reste fixe, mais dPd pousse toujours. Le robot se stabilise à dPd/Kp au-delà de Pd
  (2,5 cm pour 0,1 m/s).
- Résolution : dPd et wd sont remis à 0 après `DELAI_ANTICIPATION = 0,1 s` sans commande,
  et dPd est aussi annulé si Pd a été ramené dans l'espace de travail.

**11. Faux « SIM_ON »**
- `pgrep -f "gz sim server"` lancé dans `bash -c '…'` se trouve lui-même (même piège
  qu'au point 6). L'état de la sim se vérifie avec `ros2 topic echo --once /joint_states`.

### Tests réalisés
- `keyboard_ik.py test` : `ok`. Il couvre :
  - le MGD en pose ready ;
  - la jacobienne comparée aux différences finies ;
  - L = I et la relation dε₀/dt = −L·we ;
  - la convergence en DLS et en QP ;
  - les bornes du QP (vitesse, accélération, butée) ;
  - **le suivi d'une Pd en mouvement à 5 cm/s à moins d'1 mm grâce à dPd**.
- `udp_ik.py test` : `ok`. Il couvre l'aller-retour quaternion ↔ matrice sur 1000 rotations
  aléatoires, l'encodage et le décodage des commandes et des retours, la règle du label
  (désordre, redémarrage) et la commande 6D avec la pince inclinée de 30°.
- Protocole en conditions réelles avec de faux `/joint_states` (sim arrêtée) :
  - 100 commandes à 50 Hz, 100 retours avec le bon label ;
  - RTT local moyen de 4,9 ms (max 5,3 ms) ;
  - label 50 envoyé après le label 100 → « commande ancienne ignorée » ;
  - paquet de 3 octets ignoré ;
  - Ctrl+C propre.
- Remarque sur le RTT : la socket n'est lue qu'une fois par tour de 5 ms, donc le RTT
  mesuré inclut jusqu'à 5 ms d'attente côté robot. Un thread de réception dédié la
  supprimerait si on a besoin de mesurer finement de petits retards.
- Non testé dans Gazebo : la sim était arrêtée (voir test Gazebo ci-dessous).

### Test du protocole dans Gazebo (même jour, sim relancée par l'utilisateur)
Préalable : `keyboard_ik.py` tournait et publiait sur le même topic. L'arrêter moi-même
(SIGINT) a été refusé par le système de permissions, puisque c'était un processus de
l'utilisateur. C'est donc l'utilisateur qui l'a arrêté avant le test. Vérifié avant de
lancer le test : 0 publisher sur le topic.

Test (`/tmp/live2.py` dans le conteneur ; récepteur `udp_ik.py` lancé dans un pseudo-terminal) :
| Étape | Résultat |
|---|---|
| Flux 50 Hz pendant 3 s, Pd avance de 5 cm/s en x avec dPd | RTT moyen **4,92 ms** (max 5,25) ; à la fin, Pd x = 0,457, Pe x = 0,459 → écart **1,5 mm** (la pince est légèrement en avance), y et z exacts |
| Flux arrêté, 1 s d'attente | Pe = (0,457 ; 0 ; 0,487) = Pd : **pas de dérive** (dPd bien remis à 0) |
| Commande label 10 (après 151) vers y = 0,2 | « commande ancienne ignorée (label 10 <= 151) », **pince immobile** |
| Retour à la pose ready (label 153), puis Ctrl+C | arrêt propre, code de sortie 0 |

L'écart de 1,5 mm en suivi continu est à comparer aux ~12 mm de retard qu'aurait donnés
le seul correcteur P sans anticipation (v/Kp = 0,05/4). Il vient probablement de la
mesure : la pose renvoyée date de la réception du paquet, à moins de 5 ms près, et
/joint_states a sa propre latence. Il n'a pas été analysé plus loin.

### Reste à vérifier
- [x] Test réel dans Gazebo : déplacement +X validé (point 4).
- [ ] Retester dans Gazebo la version réécrite (timer ROS + freinage au Ctrl+C).
- [x] Tester `udp_ik.py` dans Gazebo avec une pose ponctuelle (point 7).
- [ ] Tester le QP dans Gazebo (clavier et UDP), et le comparer à `METHODE_IK = 'dls'`.
- [x] Tester le nouveau protocole UDP dans Gazebo : flux continu avec dPd, retour d'état.
- [ ] Adapter l'émetteur de l'utilisateur au nouveau format (120 octets + lecture du retour).
- [ ] Tester `udp_ik.py` dans Gazebo avec un flux continu à 50 Hz.
- [ ] Wi-Fi : ouvrir le port UDP 5005 dans le pare-feu de l'hôte, et mesurer la latence
      et les pertes de paquets.
- [ ] Tester les autres axes et le comportement aux bornes de l'espace de travail.
- [x] Pose de départ Gazebo : c'est la pose « ready », pas q = 0 (point 4).
- [ ] Vrai robot : non prévu pour l'instant. Avant, baisser les gains et vitesses, et
      vérifier les réflexes libfranka.

### Livrable
Document de synthèse créé (Claude Docs) : https://claude.ai/artifact/ApLfMQPPjeowpot1oTeCH5
Il couvre le contexte, les fichiers, le clavier, la commande cinématique (QP), le protocole UDP, les tests, les problèmes 1–11, les différences avec le dépôt S7, le mode d'emploi et ce qui reste à faire.

### Dépôt Git prêt à pousser
- Branche `fr3-teleop` créée depuis `jazzy` (v3.5.3), avec 4 commits : correctif Dockerfile, contrôleur YAML, scripts, docs (README + journal).
- Le remote `origin` (frankarobotics) est renommé en `upstream`. Il reste à créer le dépôt `franka_ros2` sur GitHub et à l'ajouter comme `origin`.
- `gh` n'est pas installé : le dépôt GitHub doit être créé à la main sur le site.
