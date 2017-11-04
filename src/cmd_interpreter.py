#!/usr/bin/env python
# coding=utf-8

'''
Ce noeud traduit les commandes en vitesse en commande moteur de 0 à 100 à envoyer au driver pwm.
'''

import rospy
from geometry_msgs.msg import Wrench
from std_msgs.msg import Int16
import numpy as np


class Motor:
    def __init__(self, namespace, x=0.0, y=0.0, tension=12):
        self.namespace = namespace

        self.x = x
        self.y = y
        self.tension = tension
        self.maxThrustTab = {6: 1.0, 12: 3.0}
        self.maxThrust = self.maxThrustTab[self.tension]

        self.cmdThr = 0
        self.cmdPos = 0

        self.pubCmdThr = rospy.Publisher(namespace + '/cmd_thr', Int16, queue_size=1)  # -100 to 100
        self.pubCmdPos = rospy.Publisher(namespace + '/cmd_pos', Int16, queue_size=1)  # -180 to 180

    def compute_cmd(self, F):

        self.cmdThr = np.trunc(np.linalg.norm(F) / self.maxThrust * 100)
        if np.isnan(self.cmdThr):
            rospy.logwarn("cmdThr is NaN")
            self.cmdThr = 0.0

        self.cmdPos = np.trunc(np.arctan2(F[1], F[0]) * 180 / np.pi)
        if np.isnan(self.cmdPos):
            rospy.logwarn("cmdPos is NaN")
            self.cmdPos = 0.0

        # Bornage entre 180 et -180 de la position des servo
        self.cmdThr = np.cos(self.cmdPos / 180.0 * np.pi) * self.cmdThr
        self.cmdPos = np.mod(self.cmdPos + 90, 180) - 90

    def publish_cmd(self):
        # print 'Publishing cmd for',self.namespace,': pos:',self.cmdPos,' and thr:',self.cmdThr
        self.pubCmdPos.publish(self.cmdPos)
        self.pubCmdThr.publish(self.cmdThr)


class BuggyCommand:
    def __init__(self):
        self.subWrench = rospy.Subscriber('cmd_wrench', Wrench, self.update_wrench)

        self.motorG = Motor('gauche', y=0.1)
        self.motorD = Motor('droite', y=-0.1)

        self.cmdWrenchTorZ = 0.0
        self.cmdWrenchForX = 0.0
        self.cmdWrenchForY = 0.0

    def update_wrench(self, msg):
        # print 'Received ',msg
        self.cmdWrenchTorZ = msg.torque.z
        self.cmdWrenchForX = msg.force.x
        self.cmdWrenchForY = msg.force.y

        Fg, Fd = self.compute_forces_char(self.motorG.y,
                                     self.cmdWrenchTorZ,
                                     self.cmdWrenchForX)
        print "Fg :", Fg
        print "Fd :", Fd

        self.motorG.compute_cmd(Fg)
        self.motorD.compute_cmd(Fg)
        self.motorG.publish_cmd()
        self.motorD.publish_cmd()
        rospy.loginfo('\n[Pav:%d, Tav:%d,\n Par:%d, Tar:%d]',
                      self.motorG.cmdPos, self.motorG.cmdThr,
                      self.motorD.cmdPos, self.motorD.cmdThr)

    def process(self):
        pass  # Inutile ici mais au moins c'est la si jamais on a besoin d'action asynchrones

    def compute_forces_char(self, dlat, m, r):
        x_fg = 0.25 * (dlat * r + m) / dlat
        x_fd = 0.25 * (dlat * r - m) / dlat

        return np.array([x_fg, 0.0, 0.0]), np.array([x_fd, 0.0, 0.0])


if __name__ == '__main__':
    rospy.init_node('cmd_buggy')
    r = rospy.Rate(50)
    buggy = BuggyCommand()
    rospy.spin()
    # while not rospy.is_shutdown():
    #    buggy.process()
    #    r.sleep()
