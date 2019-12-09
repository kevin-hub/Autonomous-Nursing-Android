#!/usr/bin/env python
import rospy
import rospkg
import sys
from std_msgs.msg import String
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
import sqlite3
import tf
from random import randrange

speech_pub = rospy.Publisher("speech_out", String, queue_size=10)
face_pub = rospy.Publisher("file_out", String, queue_size=10)
waypoint_pub = rospy.Publisher('/waypoint',PoseStamped,queue_size=10)
arm_commander_pub = rospy.Publisher('/arm_commander',String,queue_size=10)
#pose_estimate_request_pub = rospy.Publisher('/pose_estimate_request',String,queue_size=10)

# Feeling we're going to need flags to make sure there's traciblity of where the robot is
onRoute = False
locations = []

# Sleeping flag
sleeping = False
# Number of matching nouns said in current input
nouns = 0
most_recent_pose = ""

rospack = rospkg.RosPack()
main_path = rospack.get_path('main')

#The main node of the system, responsible for the connecting all the nodes accordingly to create a control flow through the system.
#The main node will get inputs from Speech and will provide outputs to movements, speech and UI.
#Vision nodes will be contacted directly from the vision nodes

base_movement = None

def update_pose_estimate_callback(pose_estimate):
    global most_recent_pose
    most_recent_pose = pose_estimate.data

def noun_callback(data):
    global nouns
    nouns = data.data

def incoming_command_callback(data):
    global speech_pub
    global locations
    global sleeping
    global nouns
    global base_movement
    global most_recent_pose
    global pose

    words = data.data.split(" ")

# Check if sleeping
    if sleeping == True:
        if "anna" in words:
            sleeping = False
        else:
            return

    elif nouns > 1:
        select = randrange(3)
        if select == 0:
            speech_pub.publish("Okay, I'll get all those things")
            face_pub.publish("many1.mp4")
        elif select == 1:
            speech_pub.publish("That's a lot of stuff you want")
            face_pub.publish("many2.mp4")
        elif select == 2:
            speech_pub.publish("I'll be one minute")
            face_pub.publish("many3.mp4")

    elif "name" in words and "what" in words:
        speech_pub.publish("My name is Anna")
        face_pub.publish("sorry1.mp4")

    elif "be" in words and "quiet" in words:
        speech_pub.publish("Okay, I'll leave you for a bit")
        face_pub.publish("sorry1.mp4")
        sleeping = True

    else:
        if 'that' in words and 'there' in words:
            location = base_movement.get_location(most_recent_pose)
            base_movement.publish_waypoint(location)
        if 'help' in words or 'nurse' in words:
            speech_pub.publish("Calling the nurse, please wait")
            face_pub.publish("help.mp4")
        elif 'book' in words:
            speech_pub.publish("Sure, I'll get you a book")
            face_pub.publish("book.mp4")
            item_loc = base_movement.get_item_location('book')
            base_movement.publish_waypoint(item_loc)
        elif "thank" in words:
            speech_pub.publish("You're welcome!")
            face_pub.publish("sorry1.mp4")
        elif 'bottle'  in words or 'thirsty'  in words or 'water' in words:
            speech_pub.publish("Okay, I'll grab some water")
            #key = 'bottle'
            face_pub.publish("water.mp4")
        elif 'bear' in words or 'teddy' in words:
            speech_pub.publish("One teddy bear coming right up")
            face_pub.publish("bear.mp4")
            item_loc = base_movement.get_item_location('teddy')
            base_movement.publish_waypoint(item_loc)
        elif 'hello' in words:
            speech_pub.publish("Hello There! I hope you're well")
            face_pub.publish("hello.mp4")
        else:
            select = randrange(3)
            if select == 0:
                speech_pub.publish("Sorry, I didn't understand that")
                face_pub.publish("sorry1.mp4")
            elif select == 1:
                speech_pub.publish("What was that?")
                face_pub.publish("sorry1.mp4")
            elif select == 2:
                speech_pub.publish("I couldn't quite catch that")
                face_pub.publish("sorry1.mp4")


    rospy.sleep(0.1)

    # Waits for the speech to respond
    #location = db_function(data.data)
    # # Going to have to create a while loop to make sure we're waiting for each value to finish
    #location = (-1.7,-0.66,0.9,0)
    #publish_waypoint(location)

    # if onRoute == False:
    #     # while(onRoute):
    #     #     # Wait for flag to have reached
    #     #     # Continue with the rest of the tasks
    # else:
    #     # Add next destination to the queue
    #     locations.append(data.data)



    # speech_pub.publish("I'm on my way!")
    # height = location[3] - Future extension

def main():
    rospy.init_node('CentralNode')
    #Speech Input
    rospy.Subscriber("noun_number", Int8, noun_callback)
    rospy.Subscriber("nlp_out", String, incoming_command_callback)
    rospy.Subscriber("pose_estimation",String, update_pose_estimate_callback)
    # Create a publisher to be able to send to other nodes

    #wp = base_movement.get_waypoint_location('A') #example get a waypoint
    #base_movement.publish_waypoint(wp)
    #item_loc = test.get_item_location('teddy') #example get an item location
    #test.publish_waypoint(item_loc)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


class base_movement_manager():
    def __init__(self):
        self.conn = sqlite3.connect(main_path + '/src/world.db')
        self.c = self.conn.cursor()

    def get_item_location(self,cmd):
        t = (cmd,)
        print "i am going to get item " + cmd
        self.c.execute('SELECT location_id FROM items NATURAL JOIN locations WHERE item_id=?', t)
        print "this is at location " + self.c.fetchone()[0]
        self.c.execute('SELECT pos_x,pos_y,pos_theta,height FROM items NATURAL JOIN locations WHERE item_id=?', t)
        item_location = self.c.fetchone() # Get the item information
        return item_location # return tuple of values

    def get_waypoint_location(self,cmd):
        t = (cmd,)
        print "I am going to move to location" + cmd
        self.c.execute('SELECT pos_x,pos_y,pos_theta,height FROM locations WHERE location_id=?', t)
        item_location = self.c.fetchone() # Get the item information
        return item_location # return tuple of values

    def publish_waypoint(self,location_tuple):
            # stuff here
        global waypoint_pub
        global arm_commander_pub
        pose = PoseStamped()
        pose.header.seq = 1
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = location_tuple[0] # x position
        pose.pose.position.y = location_tuple[1] # y position
        pose.pose.position.z = 0

        quaternion = tf.transformations.quaternion_from_euler(0, 0,location_tuple[2]) # angle
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        rospy.loginfo(pose)
        #rospy.Rate(5).sleep()
        waypoint_pub.publish(pose)
        #rospy.sleep(20)
        raw_input()
        arm_commander_pub('Pick up')


if __name__ == '__main__':
    base_movement = base_movement_manager()
    rospy.loginfo('Starting the Main Node')
    rospy.sleep(1)
    main()
