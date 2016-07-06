#! /usr/bin/env python
import rospy
import actionlib
import cv2
import image_geometry

from sub_vision.msg import TrackObjectAction, TrackObjectGoal, TrackObjectFeedback, TrackObjectResult

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class VisionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('track_object', TrackObjectAction, self.execute, False)
        self.server.start()
        
        self.bridge = CvBridge()
        
        self.downSub = rospy.Subscriber('/down_camera/image_raw', Image, self.downwardsCallback)
        self.downImage = None
        self.downModel = None
        
        self.targetType = TrackObjectGoal.navbar
        
        self.response = TrackObjectResult()
        
    def execute(self, goal):
        self.targetType = goal.objectType
        
        self.running = True
        self.ok = True
        
        feedback = TrackObjectFeedback()
        while running:
            if self.server.is_preempt_requested() or self.server.is_new_goal_available():
                self.running = False
                continue
            
            
            if self.targetType == TrackObjectGoal.navbar:
                #Process navbar stuff
                self.server.publish_feedback()
            elif self.targetType == TrackObjectGoal.redBouy:
                #process red bouy
            elif self.targetType == TrackObjectGoal.yellowBouy:
                #process yellow bouy
            elif self.targetType == TrackObjectGoal.greenBouy:
                #process green bouy
        self.response.stoppedOk = self.ok
        self.server.set_succeeded(self.response)
    
    def downwardsCallback(self, msg):
        try:
            self.downImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        if self.downModel is None:
            print("No camera model for downwards camera")
            return
        
        self.downModel.rectifyImage(self.downImage, self.downImage)
        
        
    
if __name__ == '__main__':
    rospy.init_node('vision_server')
    server = VisionServer()
    rospy.spin()
