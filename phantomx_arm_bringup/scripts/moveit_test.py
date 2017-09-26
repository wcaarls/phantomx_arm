#!/usr/bin/env python

# BEFORE EXECUTING, APPLY THIS PATCH TO
# /usr/lib/python2.7/dist-packages/pyassimp/core.py:
#
#--- old/core.py	2016-06-07 14:13:11.948880646 +0200
#+++ new/core.py	2016-06-07 14:12:59.357306268 +0200
#@@ -30,7 +30,7 @@
#     """
#     Assimp-Singleton
#     """
#-    load, load_mem, release, dll = helper.search_library()
#+    load_mem, release, dll = helper.search_library()
# _assimp_lib = AssimpLib()
# 
# def make_tuple(ai_obj, type = None):
#
# https://launchpadlibrarian.net/263969718/patch.txt

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
## END_SUB_TUTORIAL

from std_msgs.msg import String

def move_group_python_interface_tutorial():
  ## BEGIN_TUTORIAL
  ##
  ## Setup
  ## ^^^^^
  ## CALL_SUB_TUTORIAL imports
  ##
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("arm")

  print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ Reference frame: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"

  ## Moving to a pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^
  ##
  ## Moving to a pose goal is similar to the step above
  ## except we now use the go() function. Note that
  ## the pose goal we had set earlier is still active 
  ## and so the robot will try to move to that goal. We will
  ## not use that function in this tutorial since it is 
  ## a blocking function and requires a controller to be active
  ## and report success on execution of a trajectory.

  print "============ Generating plan 1"
  pose_target = geometry_msgs.msg.Pose()
  #- Translation: [-0.027, 0.003, 0.481]
  #- Rotation: in Quaternion [-0.044, -0.746, -0.039, 0.663]
  pose_target.orientation.x = -0.044
  pose_target.orientation.y = -0.746
  pose_target.orientation.z = -0.039
  pose_target.orientation.w = 0.663
  pose_target.position.x = -0.027
  pose_target.position.y = 0.003
  pose_target.position.z = 0.481
  group.set_pose_target(pose_target)

  group.go(wait=True)
  
  print "============ Generating plan 2"
  pose_target = geometry_msgs.msg.Pose()
  #- Translation: [-0.128, 0.014, 0.288]
  #- Rotation: in Quaternion [0.054, 0.999, 0.000, -0.001]
  pose_target.orientation.x = 0.054
  pose_target.orientation.y = 0.999
  pose_target.orientation.z = 0.000
  pose_target.orientation.w = -0.001
  pose_target.position.x = -0.128
  pose_target.position.y = 0.014
  pose_target.position.z = 0.288
  group.set_pose_target(pose_target)
  
  group.go(wait=True)

  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"

if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass

