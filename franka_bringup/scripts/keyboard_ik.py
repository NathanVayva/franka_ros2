#!/usr/bin/env python3
"""Téléopération clavier du bout de pince du FR3 par commande cinématique.

  z / s : avancer / reculer (±X)       q / d : gauche / droite (±Y)
  r / f : monter / descendre (±Z)      + / - : plus vite / moins vite
  espace : stop                        Ctrl+C : quitter
Maintenir une touche pour bouger. La pince reste toujours orientée vers le bas.

Principe (mêmes notations que rob_lib.cpp du projet S7 delay-robust-teleop-6dof) :
  1. Le clavier déplace la position désirée Pd à la vitesse dPd.
  2. cmd_cinematique calcule le torseur désiré de la pince :
       dPe = dPd + Kp * epsilon_p           (epsilon_p = Pd - Pe)
       we  = L^-1 (K0 * epsilon0 + L^T wd)  (epsilon0 : erreur d'orientation entre Ae et Ad)
  3. Il le convertit en vitesses articulaires dq (QP sous contraintes ou pseudo-inverse).
  4. dq est envoyé au contrôleur keyboard_joint1_controller
     (JointGroupVelocityController sur les 7 joints).

Test hors robot des calculs : python3 keyboard_ik.py test
"""
import select
import sys
import termios
import tty
from dataclasses import dataclass

import numpy as np
import rclpy
from scipy.optimize import lsq_linear
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# =============================================================================
# Paramètres de réglage (sur le vrai robot : les garder bas)
# =============================================================================
FREQUENCE = 200.0          # Hz, fréquence de la boucle de commande
DT = 1.0 / FREQUENCE       # s, pas de temps

VITESSE_INITIALE = 0.05    # m/s, vitesse de Pd au démarrage
VITESSE_MAX = 0.30         # m/s, plafond réglable avec +
ACCEL_CARTESIENNE = 0.3    # m/s², rampe sur dPd

KP = 4.0                   # 1/s, gain sur l'erreur de position epsilon_p
K0 = 3.0                   # 1/s, gain sur l'erreur d'orientation epsilon0
LAMBDA_L = 0.1             # amortissement de l'inverse de L (comme dans rob_lib.cpp)
K_POSTURE = 0.5            # 1/s, rappel du coude vers la pose ready
AMORTISSEMENT = 0.05       # lambda des moindres carrés amortis sur J

METHODE_IK = 'qp'          # 'qp' : optimisation sous contraintes, 'dls' : pseudo-inverse amortie
POIDS_POSTURE = 3e-4       # QP : poids de la posture (plus grand = coude mieux tenu, pince moins précise)
GAIN_BUTEE = 2.0           # 1/s, QP : un joint ralentit en approchant de sa butée (amortisseur)

ECART_CIBLE_MAX = 0.05     # m, Pd ne peut pas s'éloigner plus de la pince
DELAI_RELACHEMENT = 0.6    # s sans répétition de touche = touche relâchée
MARGE_BUTEE = 0.1          # rad, on arrête un joint à cette distance de sa butée

# Espace de travail autorisé pour Pd (repère de la base)
Z_MIN = 0.05               # m, au-dessus de la table
RAYON_MIN = 0.25           # m, distance mini à l'axe vertical de la base
PORTEE_MAX = 0.80          # m, distance maxi à l'épaule

TOPIC_COMMANDE = '/keyboard_joint1_controller/commands'


# =============================================================================
# Modèle du robot FR3 (documentation Franka)
# =============================================================================
@dataclass
class DH_robot:
    """Paramètres DH modifiés (Khalil-Kleinfinger), une ligne par repère."""
    alpha: np.ndarray  # rotation autour de x(i-1)
    d: np.ndarray      # translation le long de x(i-1)
    r: np.ndarray      # translation le long de z(i)


@dataclass
class Robot:
    DH: DH_robot
    V_max: np.ndarray  # rad/s, vitesse max envoyée à chaque joint
    a_max: np.ndarray  # rad/s², accélération max de chaque joint
    q_min: np.ndarray  # rad, butées
    q_max: np.ndarray


NB_JOINTS = 7
NOMS_JOINTS = [f'fr3_joint{i}' for i in range(1, NB_JOINTS + 1)]

# 7 joints + 1 ligne "outil" fixe : bride -> bout de pince
# (0.107 m de bride + 0.1034 m de pince, pince montée à -45° autour de z)
FR3 = Robot(
    DH=DH_robot(
        alpha=np.array([0.0, -np.pi / 2, np.pi / 2, np.pi / 2, -np.pi / 2, np.pi / 2, np.pi / 2, 0.0]),
        d=np.array([0.0, 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088, 0.0]),
        r=np.array([0.333, 0.0, 0.316, 0.0, 0.384, 0.0, 0.0, 0.107 + 0.1034]),
    ),
    V_max=np.full(NB_JOINTS, 0.8),
    a_max=np.full(NB_JOINTS, 2.0),
    q_min=np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]),
    q_max=np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159]),
)
ANGLE_PINCE = -np.pi / 4   # theta de la ligne outil

# Pose de départ "ready" de Franka
Q_READY = np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4])

# Orientation désirée de la pince : x vers l'avant, z vers le bas
AD_PINCE_VERS_LE_BAS = np.array([[1.0, 0.0, 0.0],
                                 [0.0, -1.0, 0.0],
                                 [0.0, 0.0, -1.0]])

# Touche -> direction de déplacement dans le repère de la base
DIRECTIONS = {
    'z': np.array([1.0, 0.0, 0.0]),
    's': np.array([-1.0, 0.0, 0.0]),
    'q': np.array([0.0, 1.0, 0.0]),
    'd': np.array([0.0, -1.0, 0.0]),
    'r': np.array([0.0, 0.0, 1.0]),
    'f': np.array([0.0, 0.0, -1.0]),
}


# =============================================================================
# Modèles géométrique et cinématique
# =============================================================================
def Rob2Model(q):
    """Angles des joints du robot -> theta du modèle DH (ajout de la ligne outil)."""
    return np.append(q, ANGLE_PINCE)


def MGD(theta, DH):
    """Modèle géométrique direct.

    Renvoie T (4x4, base -> bout de pince) et MT, la liste des matrices base -> repère i.
    """
    T = np.eye(4)
    MT = []
    for i in range(len(DH.alpha)):
        ct, st = np.cos(theta[i]), np.sin(theta[i])
        ca, sa = np.cos(DH.alpha[i]), np.sin(DH.alpha[i])
        d, r = DH.d[i], DH.r[i]
        Ai = np.array([
            [ct,      -st,      0.0, d],
            [st * ca, ct * ca,  -sa, -r * sa],
            [st * sa, ct * sa,  ca,  r * ca],
            [0.0,     0.0,      0.0, 1.0],
        ])
        T = T @ Ai
        MT.append(T)
    return T, MT


def Jacobienne(theta, DH, P):
    """Jacobienne géométrique (6x7) au point P : [vitesse linéaire ; vitesse angulaire]."""
    _, MT = MGD(theta, DH)
    J = np.zeros((6, NB_JOINTS))
    for i in range(NB_JOINTS):        # la ligne outil n'est pas un joint
        z = MT[i][:3, 2]              # axe du joint i
        o = MT[i][:3, 3]              # origine du repère i
        J[:3, i] = np.cross(z, P - o)
        J[3:, i] = z
    return J


def S(v):
    """Matrice de pré-produit vectoriel : S(v) @ u = v x u."""
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def calcul_epsilon0(Ad, Ae):
    """Erreur d'orientation entre l'orientation courante Ae et désirée Ad."""
    epsilon0 = np.zeros(3)
    for i in range(3):
        epsilon0 += np.cross(Ae[:, i], Ad[:, i])
    return 0.5 * epsilon0


def calcul_L(Ad, Ae):
    """Matrice L telle que d(epsilon0)/dt = L^T wd - L we. Tend vers I quand Ae -> Ad."""
    L = np.zeros((3, 3))
    for i in range(3):
        L += S(Ad[:, i]) @ S(Ae[:, i])
    return -0.5 * L


def cmd_cinematique(robot, q, dq_precedent, Pd, Ad, dPd, wd,
                    Kp=KP, K0=K0, lambda_L=LAMBDA_L):
    """Un pas de commande cinématique : vitesses articulaires dq qui amènent la pince en (Pd, Ad).

    Contrairement à cmd_cinematique de rob_lib.cpp qui boucle jusqu'à convergence,
    cette fonction calcule un seul pas : elle est appelée à chaque tour du timer.
    """
    theta = Rob2Model(q)
    Te, _ = MGD(theta, robot.DH)
    Ae = Te[:3, :3]
    Pe = Te[:3, 3]

    # Vitesse angulaire désirée
    epsilon0 = calcul_epsilon0(Ad, Ae)
    L = calcul_L(Ad, Ae)
    # Inverse amortie de L : reste bornée si L est mal conditionnée (grande erreur d'orientation)
    L_amortie = L.T @ np.linalg.inv(L @ L.T + lambda_L**2 * np.eye(3))
    we = L_amortie @ (K0 * epsilon0 + L.T @ wd)

    # Vitesse linéaire désirée
    epsilon_p = Pd - Pe
    dPe = dPd + Kp * epsilon_p

    # Torseur désiré -> vitesses articulaires
    J = Jacobienne(theta, robot.DH, Pe)
    torseur = np.concatenate([dPe, we])
    if METHODE_IK == 'qp':
        return resoudre_qp(robot, q, dq_precedent, J, torseur)
    return resoudre_dls(robot, q, J, torseur)


def resoudre_qp(robot, q, dq_precedent, J, torseur):
    """dq par optimisation : les limites font partie du problème au lieu d'être appliquées après.

        min  ‖J dq − torseur‖² + λ² ‖dq‖² + μ ‖dq − K_POSTURE (q_ready − q)‖²
        s.c. bornes min ≤ dq ≤ bornes max   (vitesse, accélération, butées)

    Comme toutes les contraintes sont des bornes sur dq, ce QP est un problème de moindres
    carrés bornés, que scipy.optimize.lsq_linear résout directement (~0,2 ms ici).
    """
    # Objectif : on empile les trois termes en un seul ‖A dq − b‖²
    vitesse_posture = K_POSTURE * (Q_READY - q)
    A = np.vstack([J,                                            # suivre la pince
                   AMORTISSEMENT * np.eye(NB_JOINTS),            # rester calme près des singularités
                   np.sqrt(POIDS_POSTURE) * np.eye(NB_JOINTS)])  # garder le coude près de ready
    b = np.concatenate([torseur,
                        np.zeros(NB_JOINTS),
                        np.sqrt(POIDS_POSTURE) * vitesse_posture])

    # Bornes 1 : vitesse max, et ralentissement en approchant des butées
    # (dq ≤ GAIN_BUTEE * distance restante : la vitesse tombe à 0 en arrivant à la marge)
    borne_min = np.maximum(-robot.V_max, GAIN_BUTEE * (robot.q_min + MARGE_BUTEE - q))
    borne_max = np.minimum(robot.V_max, GAIN_BUTEE * (robot.q_max - MARGE_BUTEE - q))

    # Bornes 2 : accélération max par rapport à la commande précédente (remplace la rampe).
    # Si les deux familles de bornes sont incompatibles, l'accélération l'emporte :
    # on ne peut pas freiner plus fort que ce que le robot accepte.
    pas = robot.a_max * DT
    borne_min = np.clip(borne_min, dq_precedent - pas, dq_precedent + pas)
    borne_max = np.clip(borne_max, dq_precedent - pas, dq_precedent + pas)
    borne_max = np.maximum(borne_max, borne_min + 1e-9)  # lsq_linear veut min < max

    return lsq_linear(A, b, bounds=(borne_min, borne_max), method='bvls').x


def resoudre_dls(robot, q, J, torseur):
    """dq par pseudo-inverse amortie + posture dans l'espace nul, limites appliquées après."""
    # 1. Pseudo-inverse amortie (moindres carrés amortis) : reste stable près des singularités
    J_pinv = J.T @ np.linalg.inv(J @ J.T + AMORTISSEMENT**2 * np.eye(6))
    dq = J_pinv @ torseur

    # 2. Le robot a 7 joints pour 6 contraintes : le 7e degré de liberté (le coude) est libre.
    #    On l'utilise pour rester proche de la pose ready, sans perturber la pince
    #    (projection dans l'espace nul de J). Le projecteur utilise la vraie pseudo-inverse :
    #    avec la version amortie il n'est pas exact et la posture tire la pince d'~1 mm.
    projecteur_espace_nul = np.eye(NB_JOINTS) - np.linalg.pinv(J) @ J
    dq += projecteur_espace_nul @ (K_POSTURE * (Q_READY - q))

    return limiter_vitesses(robot, q, dq)


def position_pince(q):
    """Pe : position du bout de pince pour les angles q."""
    Te, _ = MGD(Rob2Model(q), FR3.DH)
    return Te[:3, 3]


# =============================================================================
# Sécurités
# =============================================================================
def limiter_vitesses(robot, q, dq):
    """Bloque les joints qui s'approchent d'une butée et plafonne la vitesse."""
    dq = dq.copy()
    for i in range(NB_JOINTS):
        # Note : arrêt sec du joint ; un champ répulsif glisserait le long de la butée
        va_vers_butee_basse = q[i] < robot.q_min[i] + MARGE_BUTEE and dq[i] < 0
        va_vers_butee_haute = q[i] > robot.q_max[i] - MARGE_BUTEE and dq[i] > 0
        if va_vers_butee_basse or va_vers_butee_haute:
            dq[i] = 0.0

    # On divise toutes les vitesses par le même facteur pour garder la direction du mouvement
    depassement = np.max(np.abs(dq) / robot.V_max)
    if depassement > 1.0:
        dq /= depassement
    return dq


def limiter_espace_travail(P):
    """Ramène le point P dans la zone autorisée."""
    P = P.copy()

    # Au-dessus de la table
    if P[2] < Z_MIN:
        P[2] = Z_MIN

    # Pas trop près de l'axe vertical de la base
    rayon = np.hypot(P[0], P[1])
    if rayon < RAYON_MIN:
        if rayon < 1e-6:
            P[0], P[1] = RAYON_MIN, 0.0
        else:
            P[:2] *= RAYON_MIN / rayon

    # À portée du bras
    epaule = np.array([0.0, 0.0, 0.333])
    distance = np.linalg.norm(P - epaule)
    if distance > PORTEE_MAX:
        P = epaule + (P - epaule) * PORTEE_MAX / distance

    return P


def rampe(valeur, consigne, pas_max):
    """Fait évoluer valeur vers consigne d'au plus pas_max (marche aussi sur des vecteurs)."""
    return valeur + np.clip(consigne - valeur, -pas_max, pas_max)


# =============================================================================
# Noeud ROS 2
# =============================================================================
class TeleopIK(Node):

    def __init__(self):
        super().__init__('keyboard_ik')
        self.publisher = self.create_publisher(Float64MultiArray, TOPIC_COMMANDE, 10)
        self.create_subscription(JointState, '/joint_states', self.callback_joint_states, 10)
        self.create_timer(DT, self.boucle)

        self.q = None                        # positions articulaires mesurées
        self.dq = np.zeros(NB_JOINTS)        # vitesses articulaires envoyées
        self.Pd = None                       # position désirée de la pince (None = homing)
        self.dPd = np.zeros(3)               # m/s, vitesse désirée de la pince
        self.direction = np.zeros(3)         # direction demandée au clavier
        self.vitesse_max = VITESSE_INITIALE  # m/s, réglée avec + / -
        self.temps_sans_touche = 0.0

    def callback_joint_states(self, msg):
        # /joint_states contient aussi les doigts de la pince : on trie par nom
        positions = dict(zip(msg.name, msg.position))
        if all(nom in positions for nom in NOMS_JOINTS):
            self.q = np.array([positions[nom] for nom in NOMS_JOINTS])

    def lire_clavier(self):
        """Lit une touche si disponible, sans bloquer."""
        # select avec un timeout de 0 : liste non vide si une touche attend dans stdin
        touche_disponible = select.select([sys.stdin], [], [], 0)[0]
        if not touche_disponible:
            # Le terminal répète la touche tant qu'elle est maintenue :
            # plus de répétition pendant un moment = touche relâchée
            self.temps_sans_touche += DT
            if self.temps_sans_touche > DELAI_RELACHEMENT:
                self.direction = np.zeros(3)
            return

        touche = sys.stdin.read(1)
        self.temps_sans_touche = 0.0
        if touche in DIRECTIONS:
            self.direction = DIRECTIONS[touche]
        elif touche == ' ':
            self.direction = np.zeros(3)
        elif touche == '+':
            self.vitesse_max = min(VITESSE_MAX, self.vitesse_max + 0.01)
            print(f'\rvitesse max: {self.vitesse_max:.2f} m/s   ', end='', flush=True)
        elif touche == '-':
            self.vitesse_max = max(0.01, self.vitesse_max - 0.01)
            print(f'\rvitesse max: {self.vitesse_max:.2f} m/s   ', end='', flush=True)

    def boucle(self):
        """Appelée à FREQUENCE Hz par le timer."""
        self.lire_clavier()
        if self.q is None:
            return  # pas encore reçu /joint_states

        if self.direction.any() and self.publisher.get_subscription_count() == 0:
            print('\rkeyboard_joint1_controller inactif: lancez la sim avec '
                  'controller:=keyboard_joint1_controller', end='', flush=True)

        if self.Pd is None:
            dq_voulu = self.homing()
        else:
            dq_voulu = self.suivre_Pd()

        # Rampe sur les vitesses articulaires : pas d'à-coup envoyé au robot
        self.dq = rampe(self.dq, dq_voulu, FR3.a_max * DT)
        self.envoyer(self.dq)

    def homing(self):
        """Rejoint la pose ready en espace articulaire, puis active le clavier."""
        erreur = Q_READY - self.q
        if np.max(np.abs(erreur)) < 0.01:
            self.Pd = position_pince(self.q)
            print('\rpose ready atteinte, clavier actif   ', end='', flush=True)
        return limiter_vitesses(FR3, self.q, 1.0 * erreur)

    def suivre_Pd(self):
        """Déplace Pd selon le clavier et calcule la commande cinématique."""
        # dPd : rampe vers la vitesse demandée au clavier
        vitesse_voulue = self.direction * self.vitesse_max
        self.dPd = rampe(self.dPd, vitesse_voulue, ACCEL_CARTESIENNE * DT)

        # Nouvelle Pd = ancienne + dPd * dt, gardée dans l'espace de travail
        nouvelle_Pd = limiter_espace_travail(self.Pd + self.dPd * DT)

        # Si la pince n'arrive pas à suivre (butée, zone inatteignable), Pd l'attend
        Pe = position_pince(self.q)
        ecart = nouvelle_Pd - Pe
        if np.linalg.norm(ecart) > ECART_CIBLE_MAX:
            nouvelle_Pd = Pe + ecart * ECART_CIBLE_MAX / np.linalg.norm(ecart)

        # Vitesse réellement suivie par Pd après ces limites (sert d'anticipation)
        self.dPd = (nouvelle_Pd - self.Pd) / DT
        self.Pd = nouvelle_Pd
        return cmd_cinematique(FR3, self.q, self.dq, self.Pd, AD_PINCE_VERS_LE_BAS,
                               self.dPd, np.zeros(3))

    def envoyer(self, dq):
        self.publisher.publish(Float64MultiArray(data=dq.tolist()))

    def arreter(self):
        """Freine en rampe jusqu'à l'arrêt au lieu de couper la vitesse d'un coup."""
        while np.max(np.abs(self.dq)) > 1e-9:
            self.dq = rampe(self.dq, np.zeros(NB_JOINTS), FR3.a_max * DT)
            self.envoyer(self.dq)
            select.select([], [], [], DT)  # attente de DT secondes


def main():
    # Pas de gestion du Ctrl+C par ROS : le noeud doit rester vivant pour freiner
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    noeud = TeleopIK()

    # Mode cbreak : les touches arrivent tout de suite, sans attendre Entrée
    reglages_terminal = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)
    print(__doc__)
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    finally:
        noeud.arreter()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, reglages_terminal)
        noeud.destroy_node()
        rclpy.try_shutdown()


# =============================================================================
# Tests des calculs (sans ROS ni robot)
# =============================================================================
def tester():
    global METHODE_IK

    # 1. MGD : pose ready connue du FR3
    theta = Rob2Model(Q_READY)
    Te, _ = MGD(theta, FR3.DH)
    Pe = Te[:3, 3]
    assert np.allclose(Pe, [0.3069, 0.0, 0.4866], atol=1e-3), Pe
    assert np.allclose(Te[:3, :3], AD_PINCE_VERS_LE_BAS, atol=1e-6), Te

    # 2. Jacobienne : comparée aux différences finies
    J = Jacobienne(theta, FR3.DH, Pe)
    eps = 1e-6
    for i in range(NB_JOINTS):
        dq = np.zeros(NB_JOINTS)
        dq[i] = eps
        assert np.allclose(J[:3, i], (position_pince(Q_READY + dq) - Pe) / eps, atol=1e-5), i

    # 3. L : identité quand l'orientation est atteinte, et d(epsilon0)/dt = -L we (wd = 0)
    assert np.allclose(calcul_L(AD_PINCE_VERS_LE_BAS, AD_PINCE_VERS_LE_BAS), np.eye(3))
    a = np.radians(20)
    Ae = AD_PINCE_VERS_LE_BAS @ np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0],
                                          [-np.sin(a), 0, np.cos(a)]])
    we = np.array([0.1, -0.2, 0.3])
    Ae_apres = (np.eye(3) + S(we) * 1e-6) @ Ae
    d_epsilon0 = (calcul_epsilon0(AD_PINCE_VERS_LE_BAS, Ae_apres)
                  - calcul_epsilon0(AD_PINCE_VERS_LE_BAS, Ae)) / 1e-6
    assert np.allclose(d_epsilon0, -calcul_L(AD_PINCE_VERS_LE_BAS, Ae) @ we, atol=1e-5)

    # 4. Commande cinématique (les deux méthodes) : converge vers une cible décalée de 10 cm
    Pd = Pe + np.array([0.1, -0.1, -0.1])
    for METHODE_IK in ('dls', 'qp'):
        q, _ = simuler(Q_READY, Pd, AD_PINCE_VERS_LE_BAS, 2000)
        Te, _ = MGD(Rob2Model(q), FR3.DH)
        assert np.linalg.norm(Te[:3, 3] - Pd) < 1e-3, (METHODE_IK, Te[:3, 3])
        assert np.allclose(Te[:3, :3], AD_PINCE_VERS_LE_BAS, atol=1e-2), METHODE_IK

    # 5. QP : vitesse et accélération respectées pendant tout le mouvement
    _, historique = simuler(Q_READY, Pd, AD_PINCE_VERS_LE_BAS, 2000)
    dqs = np.array(historique)
    assert np.all(np.abs(dqs) <= FR3.V_max + 1e-6)
    assert np.all(np.abs(np.diff(dqs, axis=0)) <= FR3.a_max * DT + 1e-6)

    # 6. QP : joint 1 posé sur sa marge de butée, la tâche le pousse à fond vers la butée
    q = Q_READY.copy()
    q[0] = FR3.q_max[0] - MARGE_BUTEE
    J = Jacobienne(Rob2Model(q), FR3.DH, position_pince(q))
    pousse_vers_butee = 10 * J[:, 0]
    a_l_arret = resoudre_qp(FR3, q, np.zeros(NB_JOINTS), J, pousse_vers_butee)
    assert a_l_arret[0] <= 1e-6, a_l_arret[0]          # il ne s'enfonce pas
    dq_lance = np.zeros(NB_JOINTS)
    dq_lance[0] = 0.5
    lance = resoudre_qp(FR3, q, dq_lance, J, pousse_vers_butee)
    assert np.isclose(lance[0], 0.5 - FR3.a_max[0] * DT)  # il freine au maximum

    # 7. Anticipation : Pd qui avance à vitesse constante, suivie sans retard grâce à dPd
    dPd = np.array([0.05, 0.0, 0.0])
    q, dq = Q_READY.copy(), np.zeros(NB_JOINTS)
    Pd = position_pince(Q_READY)
    for _ in range(400):
        Pd = Pd + dPd * DT
        dq = cmd_cinematique(FR3, q, dq, Pd, AD_PINCE_VERS_LE_BAS, dPd, np.zeros(3))
        q = q + dq * DT
    assert np.linalg.norm(position_pince(q) - Pd) < 1e-3, position_pince(q) - Pd

    # 8. Espace de travail : Pd ne passe pas sous la table
    assert limiter_espace_travail(np.array([0.0, 0.0, -1.0]))[2] == Z_MIN
    print('ok')


def simuler(q_depart, Pd, Ad, nb_pas):
    """Robot idéal : intègre les vitesses commandées. Renvoie q final et les dq envoyées."""
    q = q_depart.copy()
    dq = np.zeros(NB_JOINTS)
    historique = []
    for _ in range(nb_pas):
        dq = cmd_cinematique(FR3, q, dq, Pd, Ad, np.zeros(3), np.zeros(3))
        q = q + dq * DT
        historique.append(dq)
    return q, historique


if __name__ == '__main__':
    if sys.argv[1:] == ['test']:
        tester()
    else:
        main()
