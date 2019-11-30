#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import sqlite3
import tf
from random import randrange

speech_pub = rospy.Publisher("speech_out", String, queue_size=10)
face_pub = rospy.Publisher("file_out", String, queue_size=10)
waypoint_pub = rospy.Publisher('/waypoint',PoseStamped,queue_size=10)

# Feeling we're going to need flags to make sure there's traciblity of where the robot is
onRoute = False
locations = []

# Sleeping flag
sleeping = False


#The main node of the system, responsible for the connecting all the nodes accordingly to create a control flow through the system.
#The main node will get inputs from Speech and will provide outputs to movements, speech and UI.
#Vision nodes will be contacted directly from the vision nodes

def incoming_command_callback(data):
    global speech_pub
    global locations
    global sleeping

    items = 0
    rospy.sleep(0.1)
    words = data.data.split(" ")

    if sleeping == True:
        if "anna" in words:
            sleeping = False
        else:
            return

    if "fuck" in words:
        select = randrange(3)
        if select == 0:
            speech_pub.publish("Fuck you")
            face_pub.publish("sorry1.mp4")
        elif select == 1:
            speech_pub.publish("Fuck off you cunt")
            face_pub.publish("sorry1.mp4")
        elif select == 2:
            speech_pub.publish("That's a bit fucking rude")
            face_pub.publish("sorry1.mp4")
    elif len(words) > 3:
        select = randrange(3)
        if select == 0:
            speech_pub.publish("Okay, I'll get all those things")
            face_pub.publish("sorry1.mp4")
        elif select == 1:
            speech_pub.publish("That's a lot of stuff you want")
            face_pub.publish("sorry2.mp4")
        elif select == 2:
            speech_pub.publish("I'll be one minute")
            face_pub.publish("sorry3.mp4")
    elif "name" in words and "what" in words:
        speech_pub.publish("My name is Anna")
        face_pub.publish("sorry1.mp4")
    elif "thank" in words and "you" in words:
        speech_pub.publish("You're welcome!")
        face_pub.publish("sorry1.mp4")
    elif "be" in words and "quiet" in words:
        speech_pub.publish("Okay, I'll leave you for a bit")
        face_pub.publish("sorry1.mp4")
        sleeping = True
    else:
        if words[0] == 'book':
            speech_pub.publish("Sure, I'll get you a book")
            face_pub.publish("book.mp4")
        elif words[0] == 'hello':
            speech_pub.publish("Hello There! I hope you're well")
            face_pub.publish("hello.mp4")
        elif words[0] == 'bottle' or words[0] == 'thirsty' or words[0] == 'water':
            speech_pub.publish("Okay, I'll grab some water")
            face_pub.publish("water.mp4")
        elif words[0] == 'bear' or words[0] == 'teddy':
            speech_pub.publish("One teddy bear coming right up")
            face_pub.publish("bear.mp4")
        elif words[0] == 'help' or words[0] == 'nurse':
            speech_pub.publish("Calling the nurse, please wait")
            face_pub.publish("help.mp4")
        elif words[0] == 'sorry'or words[0] == 'what' or words[0] == 'name':
            select = randrange(3)
            if select == 0:
                speech_pub.publish("Sorry, I didn't understand that")
                face_pub.publish("sorry1.mp4")
            elif select == 1:
                speech_pub.publish("What was that?")
                face_pub.publish("sorry2.mp4")
            elif select == 2:
                speech_pub.publish("I couldn't quite catch that")
                face_pub.publish("sorry3.mp4")


    rospy.sleep(0.1)

    # Waits for the speech to respond
    # location = db_function(data.data)
    # # Going to have to create a while loop to make sure we're waiting for each value to finish
    # publish_waypoint(location)

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
    rospy.Subscriber("nlp_out", String, incoming_command_callback)
    # Create a publisher to be able to send to other nodes


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def db_function(command):
    conn = sqlite3.connect('/home/prl1/Documents/EE4-Human-Centered-Robotics/src/main/src/world.db')
    # Aim of the function is to fetch the correct information
    c = conn.cursor()

    t = (command,)
    #use a query like this to extract the target coordinates for an item
    c.execute('SELECT pos_x,pos_y,pos_theta,height FROM items NATURAL JOIN locations WHERE item_id=?', t)
    test2 = c.fetchone()
    item_location = test2 # Get the item information

    return item_location # return tuple of values


def publish_waypoint(location_tuple):
        # stuff here
    global waypoint_pub
    pose = PoseStamped()
    pose.header.seq = 1
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = location_tuple[0] # x position
    pose.pose.position.y = location_tuple[1] # y position
    pose.pose.position.z = 0

    quaternion = tf.transformations.quaternion_from_euler(0, 0, location_tuple[2]) # angle
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    rospy.loginfo(pose)
    #rospy.Rate(5).sleep()
    waypoint_pub.publish(pose)

if __name__ == '__main__':
    rospy.loginfo('Starting the Main Node')
    rospy.sleep(2)
    main()
