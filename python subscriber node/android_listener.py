import argparse
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default, qos_profile_sensor_data, qos_profile_services_default, qos_profile_parameters
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import cv2
import time

class ListenerQos(Node):
    def __init__(self, qos_profile,msgtype):
        super().__init__('listener')
        if qos_profile.reliability is QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE:
            self.get_logger().info('QoS傳輸模式[Reliable listener]')
        else:
            self.get_logger().info('QoS傳輸模式[Sensor data/Best effort listener]')
        if msgtype == "text":
            self.sub = self.create_subscription(String, 'chatter', self.chatter_callback_text, qos_profile=qos_profile)
        else:
            self.sub = self.create_subscription(Image, 'chatter', self.chatter_callback, qos_profile=qos_profile)
        #self.check=0
        self.i=1
        self.start_time = time.time()
        self.fps_interval = 1
        self.fps_count = 0
        self.realtime_fps = 0
        self.windows=0

    def chatter_callback(self, msg):
        self.get_logger().info('I heard')
        self.fps_count += 1
        buf = np.asarray(msg.data, dtype=np.uint8)
        img=cv2.imdecode(buf,cv2.IMREAD_COLOR)
        if (time.time() - self.start_time) > self.fps_interval:
            self.realtime_fps = self.fps_count / (time.time() - self.start_time)
            self.fps_count = 0
            self.start_time = time.time()
        fps_label = 'FPS:{0:.2f}'.format(self.realtime_fps)
        cv2.putText(img, fps_label, (img.shape[1]-160, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        
        if self.windows!=img.shape[1]:
            if self.i!=1:
                cv2.destroyAllWindows()
            cv2.namedWindow('Ros2 Stream', cv2.WINDOW_NORMAL) 
            cv2.resizeWindow("Ros2 Stream", img.shape[1], img.shape[0])
            self.windows=img.shape[1]
            self.i+=1
        
        #cv2.imwrite('/home/skyline/output_{:04}.png'.format(self.i), img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        cv2.imshow('Ros2 Stream',img)
        cv2.waitKey(1)
        
    def chatter_callback_text(self, msg):
        self.get_logger().info('I heard: [%s]' % msg.data)
        
    #def get_isend(self):
        #return self.check

def main(args=None):
    #cv2.namedWindow('Ros2 Stream', cv2.WINDOW_NORMAL)
    parser = argparse.ArgumentParser(description="Ros2 Listener")
    parser.add_argument("--qos", default="default", help="Setting ROS 2 Quality of Service policies (default, services, parameters, sensor) .")
    parser.add_argument("--msg", default="camera", help="Setting ROS 2 message (camera, text) .")
    args = parser.parse_args()
    print(args.qos)
    rclpy.init(args=None)
    if args.qos=="default":
        #reliable
        custom_qos_profile=qos_profile_default
    elif args.qos=="services":
        #services
        custom_qos_profile=qos_profile_services_default
    elif args.qos=="parameters":
        #parameters
        custom_qos_profile=qos_profile_parameters
    elif args.qos=="sensor":
        #best effort
        custom_qos_profile=qos_profile_sensor_data
    else:
        custom_qos_profile=qos_profile_default
        
    node = ListenerQos(custom_qos_profile,args.msg)
    
    while rclpy.ok():
        #if(node.get_isend()==1):
            #break
        rclpy.spin_once(node)
    #rclpy.spin(node)
        
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
