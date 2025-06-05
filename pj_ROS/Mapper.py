import numpy as np

import matplotlib.pyplot as plt

matvec = lambda A, B : np.array([A@b for b in B])

import rclpy
from rclpy.node import Node
import numpy as np

from scipy.interpolate import make_splprep, splev
from scipy.signal import find_peaks

from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3

def flatten(xss):
   return np.vstack(xss)

def l2c(l, Ii=0, If=360): #lidar to xy % robot center
    angl = np.linspace(Ii*np.pi/180, If*np.pi/180, If-Ii, True)
    return l*np.cos(angl), l*np.sin(angl)

def tilde(P): #[x, y] -> [x, y, 1]
    return np.column_stack((P, np.ones(P.shape[0])))

def moving_average(a, n=6):
    ret = np.cumsum(a, axis=0, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n

def group(ld, Nmin = 10):
    xy = np.array(l2c(ld))
    xy = np.vstack((xy, ld)).T#, np.linspace(0, 2*np.pi, 360, True))).T
    xy = xy[ np.isfinite(xy[:,2]) ] #remove +/-inf

    dim2 = xy.shape[-1]

    dst = xy[:,:2]
    dst = dst[1:]-dst[:-1] #écart
    dst = np.sum(dst*dst, axis=1) #distance avec le point suivant

    dl = xy[1:,2]+xy[:-1,2] #2 * moyenne(distance au robot, sur **2 points**) -> 2pts : ca annule le 1/0.5 ligne suivante
    bump = dst > 0.1/(dl) #on groupe les points tq : dist avec le suivant < 0.1 * (1/(2*dl)) / 0.5
    bump = np.where(bump)[0]+1 #weird index décalage
    r = np.split(xy, bump, axis=0)

    #clean up
    r = [moving_average(a) for a in r if len(a) >= Nmin] #uniquement les groupes de Nmin pts ou plus
    #print(r[0])

    #fusion premier et dernier si mm groupe:
    v = r[0][0,:2]-r[-1][-1,:2] #distance entre les xy
    if np.dot(v, v) < 0.1/(r[-1][-1,2] + r[0][0,2]):
        r[0] = np.append(r[-1], r[0])
        r[0] = r[0].reshape((len(r[0])//dim2), dim2)
        r = r[:-1] #suppr le dernier que l'on a bougé

    return r

def minDev(MAP, MES, maxIt=3, errObj=1e-3):
    MAP = flatten(MAP.copy())[:, :2]
    MES = flatten(MES.copy())[:, :2]

    err = 3*errObj
    it = 0

    R = np.eye(3)[:2]

    while err > errObj and it < maxIt:
        # point projeté = distance min
        twin = np.zeros((len(MES), 2, 2))
        for i, xy in enumerate(MES):
            dst = np.sum( (xy - MAP)**2, axis=1 )
            ind = np.argmin(dst)
            twin[i, 0] = xy
            twin[i, 1] = MAP[ind]

        # matrice de passage
        Rr = ( np.linalg.pinv(tilde(twin[:,0])) @ twin[:,1] ).T
        U, _, Vt = np.linalg.svd(Rr[:,:-1])
        Rr[:,:-1] = np.dot(U, Vt)

        # nouvelle erreur
        MES = (Rr @ tilde(MES).T).T
        #print("")
        #print(R, "\n", np.vstack( (R,np.array([0, 0, 1])) ) )
        #print("")
        R = Rr @ np.vstack( (R,np.array([0, 0, 1])) )
        sqerr = np.zeros(len(MES))
        for i, xy in enumerate(MES):
            dst = np.min(np.sum( (xy - MAP)**2, axis=1 ))
            sqerr[i] = dst
        err = np.mean(sqerr)
        it += 1
    return R

#Fuse Lidar and IMU to:
# - create map of stuff around
# - estimate position and orientation of robot
class Mapper(Node):
    def __init__(self):
        super().__init__('Testgreg')
        #paramètre
        self.declare_parameter('DEBUG', '0')
        plt.ion()

        self.DEBUG = (self.get_parameter('DEBUG').get_parameter_value().string_value == '1')

        #subscribe to lidar
        self.lidarSub = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_cb,
            10)
        self.lidarSub  # prevent unused variable warning
        
        self.triggerPub = self.create_publisher(String, '/vision_trigger', 10)
        
        self.MAP = []

        self.imuSub = self.create_subscription(
            Imu,
            'imu',
            self.imu_cb,
            10)
        self.prevTime = 0 #Dt calculation - do not use bcs optimisation float
        self.Position = np.zeros(2) #x, y position % posiiton de départ [m]
        self.orientation = 0.0      #orientation % orientation de départ [rad]
        self.V = np.zeros(2)        #speed [m/s]
        self.W = 0.0                #rot speed [rad/s]
        self.Matrix = np.array([[1, 0, 0],[0, 1, 0]]) #rotation from robot to map

    def lidar_cb(self, msg):
        if self.DEBUG:
            plt.cla()

        ranges = np.array(msg.ranges)

        if len(self.MAP) == 0:
            self.MAP = group(msg.ranges)
            ##### self.triggerPub.publish(String(data="go")) #<----------------------------

        Rr = minDev(self.MAP, group(msg.ranges))
        
        self.Matrix = Rr @ np.vstack( (self.Matrix, np.array([0, 0, 1])) )

        self.orientation = (np.arccos(Rr[0,0])+np.arcsin(-Rr[0,1])+np.arcsin(Rr[1,0])+np.arccos(Rr[1,1])) / 4
        self.Position = self.Matrix[:,-1]
        
        print("LIDAR : xy, theta:", self.Position, self.orientation)
        print(self.Matrix.shape, "matrix")
        return
        
    def imu_cb(self, msg):#Euler null, à changer ?
        Dt = msg.header.stamp
        Dt = Dt.nanosec/(1e9)
        Dt, self.prevTime = Dt-self.prevTime, Dt
        if Dt <0 : #handle second decalage, evite de monter trop haut en seconde pr garder précision en ns
            Dt += 1.0
        
def main(args=None):
    print("[I] Starting couloir challenge IA solver of the dead")
    rclpy.init(args=args)

    minimal_subscriber = Mapper()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
