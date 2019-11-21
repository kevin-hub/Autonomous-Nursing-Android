#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String

#The main node of the system, responsible for the connecting all the nodes accordingly to create a control flow through the system.
#The main node will get inputs from Speech and will provide outputs to movements, speech and UI.
#Vision nodes will be contacted directly from the vision nodes

def incoming_command_callback(data):
    # Create a publisher to be able to send to other nodes
    speech_pub = rospy.Publisher("speech_out", String, queue_size=10)

    if data.data == 'book': speech_pub("Hello There! I think you want a book")
    if data.data == 'hello': speech_pub("Hello There! I hope you're well")
    if data.data == 'water': speech_pub("Hello There! I think you would like some water?)

    rospy.sleep(1)

def main():

    rospy.init_node('CentralNode', anonymous=True)
    #Speech Input
    rospy.Subscriber("nlp_out", String, incoming_command_callback)
    
    #Speech Output

    # TODO: Add the additional publishers to connect with the other nodes
    # #Movement Outputs
    # rospy.Publisher("nlp_outputs", String, callback)
    # #UI outputs
    # rospy.Publisher("nlp_outputs", String, callback)
    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print(('Starting the Main Node'))
    rospy.loginfo('Starting the Main Node')
    rospy.sleep(2)
    main()