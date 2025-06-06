import rclpy
import matplotlib.pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import cv2 as cv
from .smallest_enclosing_circle import make_circle #plus petit cercle qui entoure une liste de point, source = voir fichier
from .estimate_pose import estimatePose #corrd image vers coord 3d

def clamp(m, a, M):
    return min(max(m, a), M)

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')
        # Souscription à l'image compressée
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.imageCam_cb,
            2
        )
        self.subscription  # évite un warning

        self.cibleSub = self.create_subscription(
            Vector3,
            '/Cible_loc',
            self.Cible_cb,
            10
        )
    
        #balle géometrie
        self.balle_rad = 0.05 #sdf file

        #robot géometrie
        self.alpha = 0.52 #pitch cam - voir rviz
        self.h = 0.11 #hauteur cam - voir rviz
        self.a = 0.04 #avance cam - voir rviz
        self.camRez = [640, 480]
        self.camPos = np.array([self.a, 0, self.h])
        sa = np.sin(self.alpha)
        ca = np.cos(self.alpha)
        self.R = np.linalg.inv(np.array([
            [0,  sa,  ca, self.a],
            [1,  0,   0,  0],
            [0,  ca, -sa, self.h],
            [0,  0,   0,  1],
            ])) #camera -> robot

        self.calib = False
        self.KRTi = np.eye(4)[:, :-1] #pour inverser pixel -> direction 3d
        self.CameraInfoSub = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.infoCam_cb,
            10
        )

        # Paramètres
        self.declare_parameter('output_topic', '/cmd_vel')
        cmdveltopic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.declare_parameter('DEBUG', '0')
        self.DEBUG = (self.get_parameter('DEBUG').get_parameter_value().string_value == '1')

        # Publisher Twist pour cmd_vel
        self.botPub = self.create_publisher(Twist, cmdveltopic, 10)

        #stop epreuve 2
        self.triggerPub = self.create_publisher(String, '/vision_trigger', 10)

        #Cible
        self.cib_dst = 0
        self.cib_ang = 0
        self.cib_detec = 0

        if self.DEBUG:
            plt.ion()
 
        self.GOGOGO = False
 
    def Cible_cb(self, msg):
        self.cib_dst = msg.x
        self.cib_ang = msg.y*1.03
        self.cib_detec = msg.z
        #if self.DEBUG and self.cib_detec:
        #    print("cible at", self.cib_dst, self.cib_ang)
           
    def infoCam_cb(self, msg):
        if not self.calib:
            P = np.reshape(msg.p, (3, 4))
            self.KRTi = np.linalg.pinv(P@self.R)
            self.calib = True
        #matrice de calibration de la camera

    def imageCam_cb(self, msg):
        # Conversion du message compressé en image OpenCV
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv.imdecode(np_arr, cv.IMREAD_COLOR)

        if image is None:
            self.get_logger().warn("Échec du décodage de l'image")
            return

        balle = self.detec_balle(image.copy())

        if balle == None and not self.GOGOGO:
            return


        if balle != None and self.cib_dst > 0.5:
            bx, by = balle
            bxy = estimatePose(np.asarray([[bx, 480-by]]), self.KRTi, self.camPos, self.camPos[-1]-self.balle_rad)[0][:-1] #estimation de pose

            #Conversion polaire
            r = np.linalg.norm(bxy) #norm = distance
            t = -np.arctan2(bxy[1], bxy[0]) #angle, z vers le haut

            if r < 1.5:
                self.triggerPub.publish(String(data="balle")) #stop suivi de ligne
                self.GOGOGO = True
            
            if not self.GOGOGO:
                return

            if self.cib_detec: #si plus loin que 20cm
                cxy = np.asarray([self.cib_dst*np.cos(-self.cib_ang), self.cib_dst*np.sin(-self.cib_ang)]) #cible x,y, z vers le bas
                #t = t + np.sign(t)*10*np.pi/180
                D = bxy - cxy #direction cible → balle
                D = D / np.linalg.norm(D)
                Obj = bxy + D*0.3 #point que l'on vise, un peu en retrait de la balle pour l'aligner avec la cible

                if self.DEBUG:
                    plt.plot([Obj[0]], [Obj[1]], "d")
                    plt.plot([cxy[0]], [cxy[1]], "+")
                    plt.plot([bxy[0]], [bxy[1]], ".")
                    plt.plot([0], [0], "o")
                    plt.pause(0.00001)

                v = clamp(0.0, self.cib_dst, 1.0)

                cxy = cxy / np.linalg.norm(cxy) #direction cible
                bxy = bxy / np.linalg.norm(bxy) #direction balle

                if (np.arccos(np.dot(cxy, bxy)) > 2*np.pi/180) and (r>0.4): # > 0 rad → pas aligné → target point retrait
                    t = np.arctan2(-Obj[1], Obj[0]) #angle, z vers le haut
                else:
                    v = 0.05
                    print("------", np.arccos(np.dot(cxy, bxy)), ">", np.pi/180, r)

                msg = Twist()
                msg.linear.x = v
                msg.angular.z = t * 2.0 #correcteur proportionelle
                self.botPub.publish(msg)
            else: #cible not detected
                if r > 0.4: #la balle est loin, on continue tout droit
                    msg = Twist()
                    msg.linear.x = 0.1
                    msg.angular.z = 0.0
                    self.botPub.publish(msg)
                else: #todo: stop ?
                    msg = Twist()
                    msg.linear.x = 0.1
                    msg.angular.z = t * 2.0 #correcteur proportionelle
                    self.botPub.publish(msg)

        else:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = -0.2 #scan autour
            self.botPub.publish(msg)

                    
        if self.DEBUG:
            # Affichage
            cv.waitKey(1)

    def detec_balle(self, img):
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        lower_yellow = np.array([30, 180, 10])
        upper_yellow = np.array([40, 255, 255])
        maskb = cv.inRange(hsv, lower_yellow, upper_yellow)

        D = 11
        Y, X = np.ogrid[:D, :D]
        dist_from_center = np.sqrt((X-D//2)**2 + (Y-D//2)**2)
        kernel = np.array((dist_from_center <= D//2)*1, dtype=np.uint8)

        maskb = cv.morphologyEx(maskb, cv.MORPH_CLOSE, kernel) #fermeture pour éviter les trous
        cs, _ = cv.findContours(maskb, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE) #contour externe de la zone détectée
        if len(cs) == 0:
            return None #no balle detectée

        contour = max(cs, key=lambda c: len(c))

        #draw contour
        maskb = cv.drawContours(maskb, contour, -1, (255, 0, 255), 3)

        self.detec = len(contour) > 20

        if self.detec:
            contour = contour.reshape(max(contour.shape), 2)
         
            cx, cy, _ = make_circle(contour) #x,y only, ignore radius

       
            if self.DEBUG:
                cy = int(cy + 0.5)
                cx = int(cx + 0.5)
                maskb = cv.add(img, cv.cvtColor(maskb, cv.COLOR_GRAY2BGR))
                cv.circle(maskb, (cx, cy), 5, (0, 255, 0), -1)

                cv.imshow("mask", maskb)
                cv.waitKey(1)
        
            return cx, cy

        return None
        

def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageSubscriber()
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
