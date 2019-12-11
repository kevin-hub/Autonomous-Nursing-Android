#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import tf

def publish_waypoint(location_tuple):
    rospy.init_node('wp_publisher')


    pose = PoseStamped()
    pose.header.seq = 1
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = location_tuple[0]
    pose.pose.position.y = location_tuple[1]
    pose.pose.position.z = 0

    quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
    pose.pose.orientation.x = 0#quaternion[0]
    pose.pose.orientation.y = 0#quaternion[1]
    pose.pose.orientation.z = location_tuple[3]#quaternion[2]
    pose.pose.orientation.w = location_tuple[4] #quaternion[3]

    pub = rospy.Publisher('/waypoint',PoseStamped,queue_size=10)

    #while not rospy.is_shutdown():
    rospy.loginfo(pose)
    rospy.Rate(5).sleep()
    pub.publish(pose)





if __name__=='__main__':
    try:
        # define location tuples in form # (x_pos,y_pos, z_ori,w_ori)
        start_pos = (3.98385245187,2.35139133361,0.750390288536,0.660995018794)
        patient_pos = (3.52314562365,3.72042830714,0.780326618642,0.625372183775)
        front_pos = (5.31591619403,4.37975566763,0.774795335762,0.632212138195)
        bear_pos = front_pos
        right_pos = (6.29352000778,3.5092050533,-0.589169881958,0.808009189424)
        book_pos = right_pos

        while True:
            key = raw_input('Location?: ')
            if key =='1':
                print("moving to the starting location")
                publish_waypoint(start_pos)
                rospy.sleep(30)
            elif key == '2':
                print("moving to the bedside")
                publish_waypoint(patient_pos)
                rospy.sleep(30)
            elif key == '3':
                print("moving to the position of the bear")
                publish_waypoint(bear_pos)
                rospy.sleep(30)
            elif key =='4':
                print("moving to the position of the book")
                publish_waypoint(book_pos)
                rospy.sleep(30)
            elif key =='5':
                print("moving to the desk to front")
                publish_waypoint(front_pos)
                rospy.sleep(30)
            elif key =='6':
                print("moving to the desk to right")
                publish_waypoint(right_pos)
                rospy.sleep(30)

    except rospy.ROSInterruptException:
        pass
