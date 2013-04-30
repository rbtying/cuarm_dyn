#!/usr/bin/env python

import roslib
roslib.load_manifest('cuarm_dyn')
import rospy, actionlib

from control_msgs.msg import FollowJointTrajectoryAction
# from dynamixel_msgs.msg import JointState as JointStateDynamixel
from std_msgs.msg import Float64

class FollowController():
    def __init__(self):
        self.ns = 'arm_controller'

        #self.rate = 50.0 #rospy.get_param('~controllers/'+name+'/rate',50.0)
        #self.joints = rospy.get_param('~' + self.ns + '/arm_joints')
        # self.joints = rospy.get_param('/dynamixel/arm_controller/arm_joints')

        # rospy.loginfo('Configured for ' + str(len(self.joints)) + 'joints')
        # self.joint_subs = [JointSubscriber(name + '_controller') for name in self.joints]
        # self.joint_pubs = [JointCommander(name + '_controller') for name in self.joints]

        # self.joints_names = []

        # for idx in range(0,len(self.joints)):
        #     self.joints_names.append(self.joints[idx] + '_joint')

        # action server
        #name = rospy.get_param('~controllers/'+name+'/action_name','follow_joint_trajectory')
        self.name = self.ns + '/follow_joint_trajectory'

        self.server = actionlib.SimpleActionServer(self.name, FollowJointTrajectoryAction, execute_cb=self.actionCb, auto_start=False)
        rospy.loginfo('Started followcontroller')

    def startup(self):
        self.server.start()

    def actionCb(self, goal):
        rospy.loginfo(self.name + ": Action goal recieved.")
        traj = goal.trajectory

        if self.executeTrajectory(traj):
            self.server.set_succeeded()
            rospy.loginfo('Executed!')
        else:
            rospy.logerr('Execution failed!')
            self.server.set_aborted(text='Execution failed!')

        # if set(self.joints_names) != set(traj.joint_names):
        #     msg = "Trajectory joint names does not match action controlled joints." + str(traj.joint_names)
        #     rospy.logerr(msg)
        #     self.server.set_aborted(text=msg)
        #     return
        # if not traj.points:
        #     msg = "Trajectory empty."
        #     rospy.logerr(msg)
        #     self.server.set_aborted(text=msg)
        #     return
        # try:
        #     indexes = [traj.joint_names.index(joint) for joint in self.joints_names]
        # except ValueError as val:
        #     msg = "Trajectory invalid."
        #     rospy.logerr(msg)
        #     self.server.set_aborted(text=msg)
        #     return
        # if self.executeTrajectory(traj):   
        #     self.server.set_succeeded()
        #     rospy.loginfo('Executed.')
        # else:
        #     rospy.logerr('Execution failed.')
        #     self.server.set_aborted(text="Execution failed.")

    def executeTrajectory(self, traj):
        rospy.loginfo("Executing trajectory with " + str(len(traj.points)) + ' point(s)')
        # try:
        #     indexes = [traj.joint_names.index(joint) for joint in self.joints_names]
        # except ValueError as val:
        #     return False
        time = rospy.Time.now()
        start = traj.header.stamp

        #success = True
        for point in traj.points:
            if self.server.is_preempt_requested():
                rospy.loginfo('Stopping arm movement')
                self.server.set_preempted()
                #success = False
                break
            while rospy.Time.now() + rospy.Duration(0.01) < start:
                rospy.sleep(0.01)

            # desired = [ point.positions[k] for k in indexes ]
            endtime = start + point.time_from_start

            toprint = 'time: ' + str(point.time_from_start) + '\t'
            for i in range(0, len(traj.joint_names)):
                toprint += '[' + traj.joint_names[i] + ': ' + str(point.positions[i]) + '], '
            print toprint

            while rospy.Time.now() + rospy.Duration(0.01) < endtime:
                rospy.sleep(0.01)
        return True

if __name__ == '__main__':
    rospy.init_node('follow_joint_controller', anonymous=True)
    rospy.loginfo('Follow joint action node.')
    c = FollowController()
    rospy.loginfo('Starting action server')
    c.startup()
    rospy.loginfo('Spinning')
    rospy.spin()
