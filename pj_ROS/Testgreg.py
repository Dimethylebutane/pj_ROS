import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
import numpy as np

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, Vector3

class MinimalSubscriber(Node):
    def Ll(self, angles):
        return 0.4 * np.cos(angles)**7 #longueur ressort
    def La(self, angles):
        return 0.3 * np.cos(angles/4)**2 + 0.1 #longueur ressort

    def __init__(self):
        super().__init__('Testgreg')

        #angle entre -pi/2 et pi/2
        self.theta = np.linspace(-np.pi/2, np.pi/2, 180, True)

        self.Ka = 0.8 #raideur ressort angulaire
        self.Kl = 0.5 #raideur ressort linéaire
        print(self.Ka)

        #Dynamic state
        self.m = 10.0
        self.v = 0.0 #forward speed
        self.I = 1.5
        self.w = 0.0 #rotation speed

        #subscribe to lidar
        self.lidarSub = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_cb,
            10)
        self.lidarSub  # prevent unused variable warning

        #commande envoyee au robot (vitesse + rotation)
        self.botPub = self.create_publisher(
                Twist,
                '/cmd_vel',
                10)
        self.botMsg = Twist()
        self.botPub

    def moving_average(self, r, n=10):
        ret = np.cumsum(r, dtype=np.float32)
        ret[n:] = ret[n:] - ret[:-n]
        return ret[n - 1:] / n

    def clock_cb(self, msg):
        self.T = msg.sec

    def lidar_cb(self, msg):
        inc = msg.angle_increment #rad
        #print("inc:", inc)
        r = msg.ranges #get ranges
        N = len(r) #number of value = 360


        #convert to np array
        r = np.abs(np.array(r, dtype=np.float32))
        r = np.append(r[-N//4:][::-1], r[:N//4]) #-pi/2 à pi/2
        r[r == np.inf] = 99 #remove inf
        r[r == np.nan] = 99 #remove nan
        N = N//2 #discard backward, we want to move forward
        n = 4
        r = self.moving_average(r, n=n) #moyenne glissante
        N = N - n + 1 #la moyenne à supprimer quelque valeurs
        #print("r:", r)

        #angles en radian des faisceaux
        angles = np.linspace(-np.pi/2 + inc*(n-1)/2, np.pi/2 - inc*(n-1)/2, N, True) #en rad l'angle de chaque rayon
        
        #force de freinage
        #print("")
        #print("r", r)
        #print("Ll", self.Ll(angles))
        Fx = (r-self.Ll(angles)) * np.cos(angles) * self.Kl
        Fx[Fx > 0] = 0 #don't be pulled by ressort
        #print("Fx:", Fx)
        #plt.plot(Fx); plt.show()
        #fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}); ax.plot(angles, Fx); plt.show()

        #couple de rotation
        Mz = (r-self.La(angles)) * np.cos(angles) * self.Kl * self.La(angles)
        Mz[r>=10] = 0 #supprime les inf
        Mz = Mz * np.sin(angles) #si obstacle gauche, theta < 0 => sin < 0 => Mz < 0 => tourne à droite
        #print("Mz:", Mz, np.sum(Mz))
        #fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}); ax.plot(angles, Mz); plt.show()

        #Physics
        Fx = np.sum(Fx)
        Mz = np.sum(Mz)
        print("Fx:", Fx, "Mz:", Mz)
        
        self.v = max(self.m*(1 + Fx), 0.3) #clamp 0
        self.w = self.I*Mz
        print("v:", self.v, "w:", self.w)

        self.botMsg = Twist()
        self.botMsg.linear.x = self.v
        self.botMsg.angular.z = self.w

        self.botPub.publish(self.botMsg)

def main(args=None):
    print("[I] Starting challenge IA solver")
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
