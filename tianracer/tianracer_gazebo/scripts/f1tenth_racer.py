#! /usr/bin/env python
# @Source: https://www.guyuehome.com/35146
# @Time: 2023/10/20 17:02:46
# @Author: Jeff Wang(Lr_2002)
# LastEditors: sujit-168 su2054552689@gmail.com
# LastEditTime: 2024-03-22 10:23:42
#export TIANRACER_WORLD=racetrack_1
#export TIANRACER_WORLD=tianracer_racetrack
#export TIANRACER_WORLD=test_indoor
#export TIANRACER_WORLD=raicom

#rosrun tianracer_gazebo judge_system_node.py
import os
import rospy
import rospkg
import actionlib
import subprocess
import waypoint_race.utils as utils
import move_base_msgs.msg as move_base_msgs
import visualization_msgs.msg as viz_msgs

world = os.getenv("TIANRACER_WORLD", "tianracer_racetrack")

class RaceStateMachine(object):
    def __init__(self, filename, repeat=True):
        self._waypoints = utils.get_waypoints(filename)
        action_name = 'move_base'
        self._ac_move_base = actionlib.SimpleActionClient(action_name, move_base_msgs.MoveBaseAction)
        rospy.loginfo('Wait for %s server' % action_name)
        self._ac_move_base.wait_for_server()
        self._counter = 0
        self._repeat = repeat
        self._pub_viz_marker = rospy.Publisher('viz_waypoints', viz_msgs.MarkerArray, queue_size=1, latch=True)
        self._viz_markers = utils.create_viz_markers(self._waypoints)

    def move_to_next(self):
        # pos = self._get_next_destination()
        # if not pos:
        #     rospy.loginfo("Finishing Race")
        #     return True
        # goal = utils.create_move_base_goal(pos)
        # rospy.loginfo("Move to %s" % pos['name'])
        # self._ac_move_base.send_goal(goal)
        # self._ac_move_base.wait_for_result()
        # result = self._ac_move_base.get_result()
        # rospy.loginfo("Result : %s" % result)
        return False

    def _get_next_destination(self):
        if self._counter == len(self._waypoints):
            if self._repeat:
                self._counter = 0
            else:
                next_destination = None
        next_destination = self._waypoints[self._counter]
        self._counter = self._counter + 1
        return next_destination
    
    def path_start_launch(self):
        launch_file = "path.launch"
        subprocess.Popen(["roslaunch", "tianracer_navigation", launch_file])
        
    def start_launch(self):
        launch_file = "Pure_Pursuit_ce.launch"
        subprocess.Popen(["roslaunch", "tianracer_navigation", launch_file])
        
    def spin(self):
        rospy.sleep(1.0)
        self._pub_viz_marker.publish(self._viz_markers)
        finished = False
        while not rospy.is_shutdown() and not finished:
            finished = self.move_to_next()
            rospy.sleep(2.0)

if __name__ == '__main__':
    rospy.init_node('race')
    
    package_name = "tianracer_gazebo"

    try:
        pkg_path = rospkg.RosPack().get_path(package_name)
        filename= os.path.join(pkg_path, f"scripts/waypoint_race/raicom_check_points.yaml")
        print(f"yaml: {filename}")
    except rospkg.ResourceNotFound:
        rospy.logerr("Package '%s' not found" % package_name)
        exit(1)

    filename = rospy.get_param("~filename", filename)
    repeat = rospy.get_param('~repeat', True)

    m = RaceStateMachine(filename, repeat)
    
    # 使用subprocess同时启动两个launch文件
    m.path_start_launch()
    m.start_launch()
    
    rospy.loginfo('Initialized')
    m.spin()
    rospy.loginfo('Finished')
