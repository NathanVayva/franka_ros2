#!/usr/bin/env python3
"""Commande cinématique du FR3 par UDP, avec retour d'état (inspiré de delay-robust-teleop-6dof).

Commande (émetteur -> robot), 120 octets : struct.pack('<Qq13d', ...)
  label            : numéro de séquence, croissant (entier non signé 64 bits)
  time_send        : instant d'envoi en µs (entier 64 bits), renvoyé tel quel dans le retour
  Pd  (x, y, z)    : position désirée du bout de pince en m, repère de la base (fr3_link0)
  Ad  (qx,qy,qz,qw): orientation désirée (quaternion, normalisé à la réception)
  dPd (vx, vy, vz) : vitesse linéaire désirée en m/s (anticipation, 0 si inconnue)
  wd  (wx, wy, wz) : vitesse angulaire désirée en rad/s (anticipation, 0 si inconnue)

Retour (robot -> émetteur), 72 octets : struct.pack('<Qq7d', ...), un par commande reçue
  label, time_send : recopiés de la commande -> RTT = maintenant - time_send
  Pe  (x, y, z)    : position mesurée du bout de pince
  Ae  (qx,qy,qz,qw): orientation mesurée

Règles :
  - un paquet dont le label n'est pas plus grand que le dernier appliqué est ignoré
    (UDP peut livrer dans le désordre), sauf après DELAI_REDEMARRAGE sans paquet accepté
    (l'émetteur a pu redémarrer et repartir de 1) ;
  - le robot ne bouge pas avant le premier paquet, puis suit la dernière commande acceptée ;
  - si le flux s'arrête plus de DELAI_ANTICIPATION, dPd et wd sont remis à 0 (le robot
    reste sur Pd au lieu de dériver).

Deux terminaux (le récepteur doit tourner, sinon les paquets sont perdus sans erreur) :
  terminal 1, récepteur (reste ouvert) : python3 udp_ik.py
  terminal 2, envoi d'une pose de test : python3 udp_ik.py send 0.4 0.1 0.4 1 0 0 0
Test des calculs : python3 udp_ik.py test
"""
import socket
import struct
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Modèle et commande cinématique communs avec le contrôleur clavier
from keyboard_ik import (DT, FR3, NB_JOINTS, NOMS_JOINTS, Q_READY, AD_PINCE_VERS_LE_BAS,
                         TOPIC_COMMANDE, MGD, Rob2Model, cmd_cinematique,
                         limiter_espace_travail, rampe, simuler)

PORT = 5005
ADRESSE_ECOUTE = '0.0.0.0'   # toutes les interfaces : marche en local et en Wi-Fi
FORMAT_COMMANDE = '<Qq13d'   # label, time_send, Pd(3), Ad(4), dPd(3), wd(3)
FORMAT_RETOUR = '<Qq7d'      # label, time_send, Pe(3), Ae(4)
TAILLE_COMMANDE = struct.calcsize(FORMAT_COMMANDE)  # 120 octets
TAILLE_RETOUR = struct.calcsize(FORMAT_RETOUR)      # 72 octets

DELAI_REDEMARRAGE = 1.0      # s sans paquet accepté : on accepte de nouveau tout label
DELAI_ANTICIPATION = 0.1     # s sans paquet accepté : dPd et wd remis à 0


# =============================================================================
# Quaternions
# =============================================================================
def quaternion_vers_matrice(qx, qy, qz, qw):
    """Matrice de rotation 3x3 d'un quaternion (normalisé ici)."""
    n = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw),  2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),  1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw),  2 * (qy * qz + qx * qw),  1 - 2 * (qx**2 + qy**2)],
    ])


def matrice_vers_quaternion(A):
    """Quaternion (qx, qy, qz, qw) d'une matrice de rotation (méthode de Shepperd).

    On part du plus grand terme diagonal pour éviter de diviser par un nombre proche de 0.
    """
    trace = A[0, 0] + A[1, 1] + A[2, 2]
    if trace > 0:
        s = 2 * np.sqrt(trace + 1)
        return np.array([(A[2, 1] - A[1, 2]) / s, (A[0, 2] - A[2, 0]) / s,
                         (A[1, 0] - A[0, 1]) / s, s / 4])
    if A[0, 0] >= A[1, 1] and A[0, 0] >= A[2, 2]:
        s = 2 * np.sqrt(1 + A[0, 0] - A[1, 1] - A[2, 2])
        return np.array([s / 4, (A[0, 1] + A[1, 0]) / s,
                         (A[0, 2] + A[2, 0]) / s, (A[2, 1] - A[1, 2]) / s])
    if A[1, 1] >= A[2, 2]:
        s = 2 * np.sqrt(1 + A[1, 1] - A[0, 0] - A[2, 2])
        return np.array([(A[0, 1] + A[1, 0]) / s, s / 4,
                         (A[1, 2] + A[2, 1]) / s, (A[0, 2] - A[2, 0]) / s])
    s = 2 * np.sqrt(1 + A[2, 2] - A[0, 0] - A[1, 1])
    return np.array([(A[0, 2] + A[2, 0]) / s, (A[1, 2] + A[2, 1]) / s,
                     s / 4, (A[1, 0] - A[0, 1]) / s])


# =============================================================================
# Messages
# =============================================================================
def encoder_commande(label, time_send, Pd, quaternion, dPd=(0, 0, 0), wd=(0, 0, 0)):
    return struct.pack(FORMAT_COMMANDE, label, time_send, *Pd, *quaternion, *dPd, *wd)


def decoder_commande(donnees):
    """Renvoie un dict {label, time_send, Pd, Ad, dPd, wd} ou None si le paquet est invalide."""
    if len(donnees) != TAILLE_COMMANDE:
        return None
    champs = struct.unpack(FORMAT_COMMANDE, donnees)
    label, time_send = champs[0], champs[1]
    valeurs = np.array(champs[2:])
    Pd, quaternion, dPd, wd = valeurs[0:3], valeurs[3:7], valeurs[7:10], valeurs[10:13]
    if not np.all(np.isfinite(valeurs)) or np.linalg.norm(quaternion) < 1e-6:
        return None

    Pd_limitee = limiter_espace_travail(Pd)
    if not np.allclose(Pd_limitee, Pd):
        dPd = np.zeros(3)  # Pd est au bord de la zone : anticiper le ferait sortir
    return {'label': label, 'time_send': time_send, 'Pd': Pd_limitee,
            'Ad': quaternion_vers_matrice(*quaternion), 'dPd': dPd, 'wd': wd}


def encoder_retour(label, time_send, Pe, Ae):
    return struct.pack(FORMAT_RETOUR, label, time_send, *Pe, *matrice_vers_quaternion(Ae))


def decoder_retour(donnees):
    """Renvoie (label, time_send, Pe, quaternion) ou None si le paquet est invalide."""
    if len(donnees) != TAILLE_RETOUR:
        return None
    champs = struct.unpack(FORMAT_RETOUR, donnees)
    return champs[0], champs[1], np.array(champs[2:5]), np.array(champs[5:9])


def accepter_label(label, dernier_label, temps_depuis_dernier):
    """Vrai si la commande est plus récente que la dernière appliquée."""
    if dernier_label is None or temps_depuis_dernier > DELAI_REDEMARRAGE:
        return True  # première commande, ou émetteur redémarré
    return label > dernier_label


def maintenant_us():
    return time.time_ns() // 1000


# =============================================================================
# Noeud ROS 2 (côté robot)
# =============================================================================
class UdpIK(Node):

    def __init__(self):
        super().__init__('udp_ik')
        self.publisher = self.create_publisher(Float64MultiArray, TOPIC_COMMANDE, 10)
        self.create_subscription(JointState, '/joint_states', self.callback_joint_states, 10)
        self.create_timer(DT, self.boucle)

        # Socket non bloquante : on lit ce qui est arrivé sans attendre
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((ADRESSE_ECOUTE, PORT))
        self.socket.setblocking(False)

        self.q = None                  # positions articulaires mesurées
        self.dq = np.zeros(NB_JOINTS)  # vitesses articulaires envoyées
        self.commande = None           # dernière commande acceptée (None = rien reçu)
        self.dernier_label = None
        self.instant_derniere_commande = 0.0
        print(f'en écoute UDP sur le port {PORT} (commande {TAILLE_COMMANDE} octets, '
              f'retour {TAILLE_RETOUR} octets)')

    def callback_joint_states(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(nom in positions for nom in NOMS_JOINTS):
            self.q = np.array([positions[nom] for nom in NOMS_JOINTS])

    def lire_udp(self):
        """Vide la file de réception : répond à chaque commande, applique la plus récente."""
        while True:
            try:
                donnees, adresse_emetteur = self.socket.recvfrom(1024)
            except BlockingIOError:
                return  # plus rien à lire
            commande = decoder_commande(donnees)
            if commande is None:
                print(f'paquet ignoré ({len(donnees)} octets au lieu de {TAILLE_COMMANDE})')
                continue

            # Retour d'état à chaque commande reçue, même ancienne : l'émetteur mesure le RTT
            if self.q is not None:
                Te, _ = MGD(Rob2Model(self.q), FR3.DH)
                retour = encoder_retour(commande['label'], commande['time_send'],
                                        Te[:3, 3], Te[:3, :3])
                self.socket.sendto(retour, adresse_emetteur)

            temps_depuis_dernier = time.monotonic() - self.instant_derniere_commande
            if not accepter_label(commande['label'], self.dernier_label, temps_depuis_dernier):
                print(f'commande ancienne ignorée (label {commande["label"]} '
                      f'<= {self.dernier_label})')
                continue
            if self.commande is None:
                print('première commande reçue, le robot suit')
            self.commande = commande
            self.dernier_label = commande['label']
            self.instant_derniere_commande = time.monotonic()

    def boucle(self):
        """Appelée à 200 Hz par le timer."""
        self.lire_udp()
        if self.q is None:
            return  # pas encore reçu /joint_states

        if self.commande is None:
            dq_voulu = np.zeros(NB_JOINTS)  # rien reçu : on ne bouge pas
        else:
            c = self.commande
            dPd, wd = c['dPd'], c['wd']
            if time.monotonic() - self.instant_derniere_commande > DELAI_ANTICIPATION:
                dPd, wd = np.zeros(3), np.zeros(3)  # flux arrêté : on reste sur Pd
            dq_voulu = cmd_cinematique(FR3, self.q, self.dq, c['Pd'], c['Ad'], dPd, wd)

        # Rampe sur les vitesses articulaires : pas d'à-coup envoyé au robot
        self.dq = rampe(self.dq, dq_voulu, FR3.a_max * DT)
        self.envoyer(self.dq)

    def envoyer(self, dq):
        self.publisher.publish(Float64MultiArray(data=dq.tolist()))

    def arreter(self):
        """Freine en rampe jusqu'à l'arrêt au lieu de couper la vitesse d'un coup."""
        while np.max(np.abs(self.dq)) > 1e-9:
            self.dq = rampe(self.dq, np.zeros(NB_JOINTS), FR3.a_max * DT)
            self.envoyer(self.dq)
            time.sleep(DT)


def main():
    # Pas de gestion du Ctrl+C par ROS : le noeud doit rester vivant pour freiner
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    noeud = UdpIK()
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    finally:
        noeud.arreter()
        noeud.socket.close()
        noeud.destroy_node()
        rclpy.try_shutdown()


# =============================================================================
# Exemple d'émetteur
# =============================================================================
def envoyer_pose(valeurs, adresse='127.0.0.1'):
    """Envoie une commande (Pd, quaternion), attend le retour et affiche le RTT."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0.5)
    label = maintenant_us()  # toujours croissant d'un appel à l'autre
    s.sendto(encoder_commande(label, maintenant_us(), valeurs[0:3], valeurs[3:7]),
             (adresse, PORT))
    try:
        retour = decoder_retour(s.recv(1024))
    except socket.timeout:
        print('pas de retour en 0,5 s : le récepteur udp_ik.py tourne-t-il ?')
        return
    finally:
        s.close()
    _, time_send, Pe, quaternion = retour
    print(f'RTT = {(maintenant_us() - time_send) / 1000:.2f} ms')
    print(f'Pe mesurée (avant mouvement) = {np.round(Pe, 3)}, Ae = {np.round(quaternion, 3)}')


# =============================================================================
# Tests des calculs (sans ROS ni robot)
# =============================================================================
def tester():
    # Quaternions : cas connus, puis aller-retour sur des rotations quelconques
    assert np.allclose(quaternion_vers_matrice(0, 0, 0, 1), np.eye(3))
    assert np.allclose(quaternion_vers_matrice(1, 0, 0, 0), AD_PINCE_VERS_LE_BAS)
    generateur = np.random.default_rng(0)
    for _ in range(1000):
        quaternion = generateur.normal(size=4)
        quaternion /= np.linalg.norm(quaternion)
        retrouve = matrice_vers_quaternion(quaternion_vers_matrice(*quaternion))
        assert np.allclose(retrouve, quaternion) or np.allclose(retrouve, -quaternion)

    # Commande : bonne, mauvaise taille, quaternion nul, Pd hors zone (anticipation coupée)
    c = decoder_commande(encoder_commande(7, 123, (0.4, 0, 0.3), (1, 0, 0, 0), (0.1, 0, 0)))
    assert c['label'] == 7 and c['time_send'] == 123
    assert np.allclose(c['Pd'], [0.4, 0, 0.3]) and np.allclose(c['Ad'], AD_PINCE_VERS_LE_BAS)
    assert np.allclose(c['dPd'], [0.1, 0, 0])
    assert decoder_commande(b'abc') is None
    assert decoder_commande(encoder_commande(1, 0, (0.4, 0, 0.3), (0, 0, 0, 0))) is None
    c = decoder_commande(encoder_commande(1, 0, (0.4, 0, -1.0), (1, 0, 0, 0), (0, 0, -0.1)))
    assert np.allclose(c['dPd'], 0)

    # Retour : aller-retour
    label, time_send, Pe, quaternion = decoder_retour(
        encoder_retour(7, 123, np.array([0.3, 0, 0.5]), AD_PINCE_VERS_LE_BAS))
    assert (label, time_send) == (7, 123) and np.allclose(Pe, [0.3, 0, 0.5])
    assert np.allclose(np.abs(quaternion), [1, 0, 0, 0])

    # Label : désordre ignoré, redémarrage de l'émetteur accepté après un silence
    assert accepter_label(1, None, 0.0)
    assert accepter_label(5, 4, 0.01)
    assert not accepter_label(3, 4, 0.01)
    assert not accepter_label(4, 4, 0.01)
    assert accepter_label(1, 500, DELAI_REDEMARRAGE + 0.1)

    # Commande 6D : pince inclinée de 30° par rapport à la verticale
    a = np.radians(15)
    Ad = quaternion_vers_matrice(np.cos(a), 0, np.sin(a), 0)
    assert abs(np.degrees(np.arccos(-Ad[2, 2])) - 30) < 1e-6
    Pd = np.array([0.4, 0.1, 0.4])
    q, _ = simuler(Q_READY, Pd, Ad, 3000)
    Te, _ = MGD(Rob2Model(q), FR3.DH)
    assert np.linalg.norm(Te[:3, 3] - Pd) < 1e-3, Te[:3, 3]
    assert np.allclose(Te[:3, :3], Ad, atol=1e-2), Te[:3, :3]
    print('ok')


if __name__ == '__main__':
    if sys.argv[1:2] == ['test']:
        tester()
    elif sys.argv[1:2] == ['send']:
        envoyer_pose([float(v) for v in sys.argv[2:9]])
    else:
        main()
