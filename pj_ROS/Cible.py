import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Vector3
import numpy as np
import cv2 as cv
from .smallest_enclosing_circle import make_circle #plus petit cercle qui entoure une liste de point, source = voir fichier
from .estimate_pose import estimatePose #corrd image vers coord 3d

def tilde(P):
    return np.column_stack((P, np.ones(P.shape[0])))

def moving_average(a, n=3):
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n

matvec = lambda A, B : np.array([A@b for b in B])

def clamp(m, a, M):
    return min(max(m, a), M)

class Cible_folower(Node):
    def __init__(self):
        super().__init__('cible_folower')
        self.detec = False   #True if cible présent et detectée

        self.c3d = np.zeros(2) #Z = 0

        self.alpha = 0.52 #pitch cam - voir rviz
        self.h = 0.11 #hauteur cam - voir rviz
        self.a = 0.04 #avance cam - voir rviz
        self.camRez = [640, 480]
        self.camPos = np.array([self.a, 0, self.h])

        #default value, see cameraInfo_cb
        self.calib = False
        self.KRTi = np.eye(4)[:, :-1] #pour inverser pixel -> direction 3d
        sa = np.sin(self.alpha)
        ca = np.cos(self.alpha)
        self.R = np.linalg.inv(np.array([
            [0,  sa,  ca, self.a],
            [1,  0,   0,  0],
            [0,  ca, -sa, self.h],
            [0,  0,   0,  1],
            ])) #camera -> robot

        # Souscription à l'image compressée
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.imageCam_cb,
            10
        )
        self.CameraInfoSub = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.infoCam_cb,
            10
        )

        # Paramètres
        self.declare_parameter('output_topic', '/Cible_loc')
        outTopicName = self.get_parameter('output_topic').get_parameter_value().string_value
        self.declare_parameter('DEBUG', '0')
        self.DEBUG = (self.get_parameter('DEBUG').get_parameter_value().string_value == '1')

        #Publication position cible
        self.pub_loc = self.create_publisher(Vector3, outTopicName, 10) #[dist, angle, confiance]

    def pubCibl(self, d, r, c):
        a = Vector3()
        a.x = float(d)
        a.y = float(r)
        a.z = float(c)

        self.pub_loc.publish(a)

    #def estimatePose(self, uvs):
    #    U = tilde(uvs)
    #    pinv = self.KRTi
    #    di = matvec(pinv, U)[:,:-1]
    #    
    #    dz = di[:,-1]
    #    t = self.camPos[-1]/dz #taille du vecteur pour toucher le sol (z=0)
    #    Pest = self.camPos - np.multiply(t.reshape(t.shape[0], 1), di) #point = position camera + vecteur direction*taille
    #    return Pest

    def infoCam_cb(self, msg):
        if not self.calib:
            P = np.reshape(msg.p, (3, 4))
            self.KRTi = np.linalg.pinv(P@self.R)
            self.calib = True
        #matrice de calibration de la camera

    def imageCam_cb(self, msg):
        if not self.calib: #la camera n'est pas calibrée
            return

        # Conversion du message compressé en image OpenCV
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv.imdecode(np_arr, cv.IMREAD_COLOR)

        if image is None:
            self.get_logger().warn("Échec du décodage de l'image")
            return

        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV) #cenvertion en HSV pour algo de sistance de couleur
        lower_blue = np.array([80, 52, 45])        #couleur min
        upper_blue = np.array([140, 200, 200])     #couleur max
        maskb = cv.inRange(hsv, lower_blue, upper_blue)#couleur in [min, max]

        #kernel = np.ones((25, 25), np.uint8)
        D = 7
        Y, X = np.ogrid[:D, :D]
        dist_from_center = np.sqrt((X-D//2)**2 + (Y-D//2)**2)
        kernel = np.array((dist_from_center <= D//2)*1, dtype=np.uint8)

        maskb = cv.morphologyEx(maskb, cv.MORPH_CLOSE, kernel) #fermeture pour éviter les trous
        
        cs, _ = cv.findContours(maskb, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE) #contour externe de la zone détectée
        if len(cs) == 0: #no cible detected
            self.pubCibl(np.inf, 0, 0) #confiance = 0
            return

        contour = np.concatenate(cs)
        #contour = max(cs, key=lambda c: len(c))
        self.detec = len(contour) > 50

        if not self.detec:
            #not detected:
            self.pubCibl(np.inf, 0, 0) #confiance = 0
        else:
            contour3D = np.array(contour).reshape((contour.shape[0], 2))
            contour3D[:, 1] = self.camRez[1] - contour3D[:, 1]
            #contour3D = self.estimatePose(contour3D) #convertion en point 3d
            contour3D = estimatePose(contour3D, self.KRTi, self.camPos, self.camPos[-1])

            coord = make_circle(contour3D[:,:2])[:2] #x,y only, ignore radius

            self.c3d = coord
            if self.DEBUG:
                print("3d center:", coord, np.linalg.norm(coord))

            x, y = zip(*contour3D[:,:-1])

            angl = -np.arctan2(self.c3d[1], self.c3d[0]) #z vers le haut, inversion
            Norm = np.linalg.norm(self.c3d)

            conf = (Norm < 2.3)*1 #detec if distance < 2.5m

            if self.DEBUG:
                print("dist =",Norm,"t=",angl, "conf=", conf)

            self.pubCibl(Norm, angl, conf)
       
        if self.DEBUG:
            maskb = cv.add(image, cv.cvtColor(maskb, cv.COLOR_GRAY2BGR))
            #col = [(255, 255, 122), (255, 0, 255), (255, 0, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255), ]
            maskb = cv.drawContours(maskb, contour, -1, (255, 0, 255), 3)

            if self.detec:
                pinv = np.linalg.pinv(self.KRTi)
                cx = matvec(pinv, tilde(np.array([[self.c3d[0], self.c3d[1], 0]])))
                cx, cy = cx[0,:-1]/cx[0,-1]
                #print("Cible @", "x:", cx, 'y:', self.cy)
                cv.circle(maskb, (int(cx), self.camRez[1]-int(cy)), 5, (0, 255, 0), -1)
                #cv.line(maskb, (image.shape[1]//2, image.shape[0]), (image.shape[1]//2 + int(Vx), image.shape[0] - int(Vy)), (255, 255, 0), 1)
                #cv.line(maskb, (image.shape[1]//2, image.shape[0]), (image.shape[1]//2, image.shape[0] - int(Vy)), (255, 255, 0), 1)

            cv.imshow("mask", maskb)
            cv.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Cible_folower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv.destroyAllWindows()


if __name__ == '__main__':
    main()
