from .robot import Robot
import json
import ob1nav
from ob1nav.srv import NamedLocation, RelativeLocation
import nlp, nav, vis


import rospy
from std_msgs.msg import String 

class CarryLuggageRobot(Robot):
    def __init__(self) -> None:
        super().__init__()
        self.speech_sub = rospy.Subscriber(nlp.speech_to_text.topic, callback=self.save_speech)
        self.person_sub = rospy.Subscriber(vis.objection_detection.topic, callback=self.update_obj_loc)

        self.gripper_pub = rospy.Publisher('girpper', String, queue_size=10)
        rospy.wait_for_service('named_location_service')
        self.name_location_srv = rospy.ServiceProxy("named_location_service", NamedLocation)
        rospy.wait_for_service('relative_location_service')
        self.follow_server = rospy.ServiceProxy("named_location_service", RelativeLocation)

        self.named_map_locations = json.load(open(ob1nav.NAMED_LOCATIONS_FILE))
        

    def save_speech(self, msg):
        self.speech_buffer.append(msg)
        
    def update_obj_loc(self, msg):
        self.person_location = msg.person_location # Do we need to handle multiple people?
        self.bag_location = msg.bag_location
        
    def main(self):
        nav.self_localize()
        self.name_location_srv(nav.named_location.srv, 'GOTO front door')
        self.follow_server(loc=self.person_location)
        nlp.story(['welcome', 'Let me take your bag'])
        self.gripper_pub.publish("open")
        ###############
		# ros.call(arm.gripper.goto(loc=self.bag_location)) # Will we need handle? Do we need to do handover
        ###############
        self.gripper_pub.publish("closed")
        while not (nlp.analyse(self.speech_buffer, find='stop')):
            self.follow_server(loc=self.person_location) 
            # what if we miss person? (last update is too old) Recovery action.
            # if nav.slam.check_unovercomeable_object():
            #     recovery()
        image = rospy.wait_for_message(vis.camera_rgb_sub)
        car_location = vis.objection_detection.model(['car'])(image)
        self.follow_server(loc=car_location) 

if __name__ == '__main__':
    robot = CarryLuggageRobot()
    robot.main()