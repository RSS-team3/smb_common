#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped


class RssStateMachine:
    def __init__(self) -> None:
        """Tare for usual exploration while far for going back to the home point"""
        # Start with idle mode, receives nothing and do not publish anything
        self.state = "tare"
        self.rate = rospy.Rate(10)  # 10hz
        self.tare_timeout_thres = rospy.Duration.from_sec(20)  # seconds

        # Topic names
        self.tare_waypoint_topic = "/sensor_coverage_planner/tare_planner/way_point"
        self.tare_waypoint_sub = rospy.Subscriber(
            self.tare_waypoint_topic, PointStamped, self.tare_waypoint_cb)
        self.tare_waypoint = PointStamped()
        self.tare_done_topic = "/sensor_coverage_planner/tare_planner/exploration_finish"

        self.far_waypoint_topic = "/far_planner/way_point"
        self.far_waypoint_sub = rospy.Subscriber(
            self.far_waypoint_topic, PointStamped, self.far_waypoint_cb)
        self.far_waypoint = PointStamped()
        self.far_done_topic = "/far_reach_goal_status"
        self.far_done_sub = rospy.Subscriber(
            self.far_done_topic, Bool, self.far_done_cb)
        self.far_goal_topic = "/goal_point"
        self.far_goal_publisher = rospy.Publisher(
            self.far_goal_topic, PointStamped, queue_size=10)

        self.waypoint_topic = "/way_point"
        self.waypoint_publisher = rospy.Publisher(
            self.waypoint_topic, PointStamped, queue_size=10)

        # Timer
        self.start_time = rospy.Time.now()

        # Keep publishing 0,0,0 for homing
        while not rospy.is_shutdown():
            print("Current state: ", self.state)
            self.waypoint_mux_publish()
            self.far_goal_pub()
            self.check_tare_timeout()
            self.rate.sleep()

    def far_waypoint_cb(self, msg):
        self.far_waypoint = msg

    def tare_waypoint_cb(self, msg):
        self.tare_waypoint = msg

    def waypoint_mux_publish(self):
        """Check current state and publish waypoint"""
        if self.state == "far":
            self.waypoint_publisher.publish(self.far_waypoint)
        elif self.state == "tare":
            self.waypoint_publisher.publish(self.tare_waypoint)
        else:
            return

    def far_done_cb(self, msg):
        """Receives bolean whether the far planner has sent the robot back to home successfully"""
        if self.state == "tare":
            return
        elif self.state == "far":
            if msg.data == True:
                self.state = "tare"
                rospy.logwarn(
                    "Far planner reaches home, switching to Tare planner")
            else:
                print("Far planner moving")
                return

    def far_goal_pub(self):
        """Publish home once"""
        # if self.state == "far":
        #     return
        # elif self.state == "tare":
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "/map"
        point.point.x = 0.0
        point.point.y = 0.0
        point.point.z = 0.0
        self.far_goal_publisher.publish(point)

    def check_tare_timeout(self):
        """We have a limit on exploration time, switch to far planner and return to base 
        if time has exceeded threshold
        """
        if self.state == "tare":
            # Check if time exceeds
            time_elapsed = rospy.Time.now() - self.start_time
            if (time_elapsed > self.tare_timeout_thres):
                self.state = "far"
                self.start_time = rospy.Time.now()
                rospy.logwarn(
                    "Tare planner exceeds time limit, switching to far planner")
        else:
            self.start_time = rospy.Time.now()


if __name__ == '__main__':
    try:
        rospy.init_node('RssStateMachine', anonymous=True)
        RssStateMachine()
    except rospy.ROSInterruptException:
        pass
