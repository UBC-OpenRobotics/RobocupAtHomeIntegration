from .robot import Robot
import rospy
from cv_bridge import CvBridge, CvBridgeError

import json
import ob1nav
from ob1nav.srv import NamedLocation, RelativeLocation
import nlp, nav, vis
from collections import deque

import face_recognition

class ReceptionistRobot(Robot):
    speech_buffer = deque(maxlen=10)
    person_location = None
    chair_location = None
    memory = {}
    
    def __init__(self):
        self.speech_sub = rospy.Subscriber(nlp.speech_to_text.topic, callback=self.save_speech)
        self.person_sub = rospy.Subscriber(vis.objection_detection.topic, callback=self.update_obj_loc)

        rospy.wait_for_service('named_location_service')
        self.name_location_srv = rospy.ServiceProxy("named_location_service", NamedLocation)
        rospy.wait_for_service('relative_location_service')
        self.follow_server = rospy.ServiceProxy("named_location_service", RelativeLocation)

        self.cv_bridge = CvBridge()
        
    def save_speech(self, msg):
        self.speech_buffer.append(msg)
        
    def update_obj_loc(self, msg):
        self.person_location = msg.person_location # Do we need to handle multiple people?
        self.chair_location = msg.chair_location
    
    def guest_reception(self, guest):
        while not self.person_location:
            rospy.sleep(1)
            print('waiting for guest')
        self.follow_server(loc=self.person_location)
        
        # nlp.story(['welcome', 'gather info about guest'])
        img_data = rospy.wait_for_message(vis.camera_rgb_sub)
        img = self.cv_bridge.imgmsg_to_cv2(img_data, 'rgb8')
        face_enc = face_recognition.face_encoding(img)[0]
        
        self.memory[guest]['face-encode'] = face_enc
        self.memory[guest]['name'] = nlp.analyse(nlp.stt, find='name') #TODO: extract name
        self.memory[guest]['drink'] = nlp.analyse(nlp.stt, find='drink') #TODO: extract drink

        ##################### TODO: Feature extraction
        for feature, description in vis.person_features(img):
            self.memory[guest][feature] = description
        #####################
        nlp.tts("Follow me")
        self.name_location_srv(nav.named_location.srv, 'GOTO living room')

    def save_guests(self):
        with open(vis.GUEST_INFO_FILE, 'w+') as f:
            json.dump(self.memory, f)
        
    def main(self):
        ##### Optional
        # arm.open_door_squence() # Not sure How this would look
        #####
        nav.self_localize()# TODO: localization does not exist. Robot needs to be placed in known starting point
        self.name_location_srv(nav.named_location.srv, 'GOTO front door')
        self.guest_reception('guest1')
        nlp.story(['introduce guest to host'])(self.mem['guest1']) # TODO: python story call
        self.guest_reception('guest2')
        nlp.story(['introduce guest to host'])(self.mem['guest2']) # TODO: python story call

if __name__ == '__main__':
    robot = ReceptionistRobot()
    robot.main()