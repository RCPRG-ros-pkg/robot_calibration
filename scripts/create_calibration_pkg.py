#!/usr/bin/python

# Copyright (c) 2015, Warsaw University of Technology
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in the
#        documentation and/or other materials provided with the distribution.
#      * Neither the name of the Warsaw University of Technology nor the
#        names of its contributors may be used to endorse or promote products
#        derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL Warsaw University of Technology BE LIABLE FOR ANY
#  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import os
import sys
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
import PyKDL as kdl
import rospkg

def joint2mat(joint, joint_index):
  ret = '[rot2(q({0}), [{1}, {2}, {3}]), [{4}; {5}; {6}]; [0, 0, 0, 1]] * inv([rot2(0.0, [{1}, {2}, {3}]), [{4}; {5}; {6}]; [0, 0, 0, 1]])'
  vec = joint.JointAxis()
  origin = joint.JointOrigin()
  return ret.format(joint_index + 1, vec[0], vec[1], vec[2], origin[0], origin[1], origin[2])

def frame2mat(frame):
  return '[' + str(frame.M[0, 0]) + ', ' + str(frame.M[0, 1]) + ', ' + str(frame.M[0, 2]) + ', ' + str(frame.p[0]) + '; ' + str(frame.M[1, 0]) + ', ' + str(frame.M[1, 1]) + ', ' + str(frame.M[1, 2]) + ', ' + str(frame.p[1]) + '; ' + str(frame.M[2, 0]) + ', ' + str(frame.M[2, 1]) + ', ' + str(frame.M[2, 2]) + ', ' + str(frame.p[2]) + '; ' + '0, 0, 0, 1]'

def chain2mat(chain, calib_links, calib_joints):
  frame_str = ''
  fun_args = []
  for i in range(chain.getNrOfSegments()):

    seg = chain.getSegment(i)

    if seg.getJoint().getType() != kdl.Joint.None:
      if seg.getJoint().getType() == kdl.Joint.RotAxis:
        joint = seg.getJoint()

        if not joint.getName() in calib_joints:
          calib_joints.append(joint.getName())
        global_joint_index = calib_joints.index(joint.getName())

        if len(frame_str) != 0:
          frame_str = frame_str + ' * '
        frame_str = frame_str + joint2mat(joint, global_joint_index)
      else:
        print 'false'

    tip = seg.getFrameToTip()

    if len(frame_str) != 0:
      frame_str = frame_str + ' * '

    if seg.getJoint().getName() in calib_links:
      frame_str = frame_str + '{0}_id.transform()'.format(seg.getJoint().getName())
      fun_args.append(seg.getJoint().getName() + '_id')
    else:
      frame_str = frame_str + frame2mat(tip)
  return [frame_str, fun_args]

def findcommonframe(tree, base_frame, frame1, frame2):
  chain1 = tree.getChain(base_frame, frame1)
  chain2 = tree.getChain(base_frame, frame2)

  i = 0
  common_link = ''
  while 1:
    if chain1.getSegment(i).getJoint().getName() == chain2.getSegment(i).getJoint().getName():
      common_link = chain1.getSegment(i).getName()
    else:
      break
    i = i+1
  return common_link

def getparent(chain, frame):
  for i in range(chain.getNrOfSegments()):
    if chain.getSegment(i).getName() == frame:
      return chain.getSegment(i-1).getName()

def genmsrfun(tree, chain_start, chain_end, calib_joints, joints):
  base_frame = 'world'
  fun_args = []
  common_link = findcommonframe(tree, base_frame, chain_start, chain_end)

  x2cam_chain = tree.getChain(common_link, chain_start)
  [x2cam_str, args] = chain2mat(x2cam_chain, calib_joints, joints)
  fun_args = fun_args + args
  x2marker_chain = tree.getChain(common_link, chain_end)
  end_link = getparent(x2marker_chain, chain_end)
  x2arm_chain = tree.getChain(common_link, end_link)
  [x2arm_str, args] = chain2mat(x2arm_chain, calib_joints, joints)
  fun_args = fun_args + args

  fun_name = chain_start + '2' + chain_end
  chain_fun_header = 'function z = {0}('.format(fun_name)
  for arg in fun_args:
    chain_fun_header = chain_fun_header + arg + ', '
  chain_fun_header = chain_fun_header + 'p, joint_offset_id, jnt, intrinsic)\n'
  chain_fun_header = chain_fun_header + '  q = jointOffset(jnt, joint_offset_id.vec);\n'
  chain_fun_header = chain_fun_header + '  x2cam = mtk.SE3(' + x2cam_str + ');\n'
  chain_fun_header = chain_fun_header + '  x2arm = mtk.SE3(' + x2arm_str + ');\n'
  chain_fun_header = chain_fun_header + '  z = project_point(intrinsic, x2arm, x2cam, p);'
  return {'name': fun_name, 'fun': chain_fun_header, 'fun_args': fun_args, 'chain_start': chain_start, 'chain_end': chain_end}

def genjointoffset(joints, jnt_off):
  ret = 'function [ j ] = jointOffset(jnt, offset)\n'
  ret = ret + '  j = jnt + ['
  i = 1
  for jnt in joints:
    if jnt in jnt_off:
      ret = ret + 'offset({0})'.format(i)
      i = i + 1
    else:
      ret = ret + '0'

    if joints.index(jnt) < (len(joints)-1):
      ret = ret + '; '
  ret = ret + '];\n'
  ret = ret + 'end'
  return ret

def haveoffset(chain, calib_joints, marker_links, num, direction):
  index = num + direction
  if index < 0:
    return 1
  if index >= chain.getNrOfSegments():
    return 1

  seg = chain.getSegment(index)

  if seg.getJoint().getType() != kdl.Joint.None:
    return 1
  else:
    if (seg.getJoint().getName() in calib_joints) or (seg.getName() in marker_links):
      return 0
    else:
      return haveoffset(chain, calib_joints, marker_links, index, direction)

def jointofflist(tree, begin, end, calib_joints, marker_links):
  chain = tree.getChain(begin, end)
  size = chain.getNrOfSegments()

  jnt_off = []

  for i in range(size):
    seg = chain.getSegment(i)
    if seg.getJoint().getType() != kdl.Joint.None:
      if haveoffset(chain, calib_joints, marker_links, i, -1) and haveoffset(chain, calib_joints, marker_links, i, 1):
        jnt_off.append(seg.getJoint().getName())
  return jnt_off

def ischainusefule(tree, begin, end, calib_joints):
  chain = tree.getChain(begin, end)
  if chain.getNrOfJoints() < 1:
    return False

  for i in range(chain.getNrOfSegments()):
    seg = chain.getSegment(i)
    if seg.getJoint().getName() in calib_joints:
      return 1
  return False

def genmatload():
  ret = 'clear all\n'

  path = rospkg.RosPack().get_path('robot_calibration')

  ret = ret + 'addpath(\'{0}/matlab/MTK/matlab\');\n'.format(path)
  ret = ret + 'addpath(\'{0}/matlab/calibration_common\');\n'.format(path)

  ret = ret + 'load(\'/tmp/calibration.mat\');\n'
  return ret

def matmain(calib_links, chains_spec, marker_links, camera_links, joint_off):
  ret = '''load_data
intrinsic_t = mtk.make_compound('focal_length', @mtk.Rn, 2, ...
                                'offset', @mtk.Rn, 2, ...
                                'distortion', @mtk.Rn, 4);

o = mtk.OptimizationProblem();\n'''

  ret = ret + 'joint_offset_id = o.add_random_var(mtk.Rn(zeros({0}, 1)));\n'.format(len(set(joint_off)))

  for cam in camera_links:
    ret = ret + '{0}_intrinsic = intrinsic_t();\n'.format(cam)
    ret = ret + '{0}_intrinsic.focal_length.vec = [{0}_K(1, 1); {0}_K(2, 2)];\n'.format(cam)
    ret = ret + '{0}_intrinsic.offset.vec = [{0}_K(3, 1); {0}_K(3, 2)];\n'.format(cam)
    ret = ret + '{0}_intrinsic.distortion.vec = {0}_D\';\n'.format(cam)

  for jnt in calib_links:
    ret = ret + '{0}_id = o.add_random_var(mtk.SE3({0}\'));\n'.format(jnt)

  for mkr in marker_links:
    ret = ret + '{0}_id = o.add_random_var({0});\n'.format(mkr)

  for fun in chains_spec:
    ret = ret + 'for i=1:size({0}_{1}_points, 2)\n'.format(fun['chain_start'], fun['chain_end'])
    ret = ret + '  o.add_measurement({0}_{1}_points(:, i), @{2}, '.format(fun['chain_start'], fun['chain_end'], fun['name']) + '{'
    for arg in fun['fun_args']:
      ret = ret + '{0}'.format(arg)
      ret = ret + ', '
    ret = ret + '{0}_id'.format(fun['chain_end'])
    ret = ret + ', joint_offset_id}, {'
    ret = ret + '{0}_{1}_joints(:, i), {0}_intrinsic'.format(fun['chain_start'], fun['chain_end'])
    ret = ret + '}, 0.1);\n'
    ret = ret + 'end\n\n'
  ret = ret + '''[X] = nrlm(@o.fun, o.X0, [20, 4, 1e-3]); % solve problem

[r, J, r_orig] = o.fun(X);

rms_chk = sqrt(sum(r_orig.^2,1) / size(r_orig,1));
disp(['RMS pattern reprojection error:' num2str(rms_chk)])

'''
  ret = ret + '''chk = reshape(r_orig, 2, []);
index = 0;
'''
  for fun in chains_spec:
    ret = ret + 'chk_size_{0}_{1} = 1;\n'.format(fun['chain_start'], fun['chain_end'])
    ret = ret + 'chk_{0}_{1} = chk(:, index+1:index+size({0}_{1}_points, 2)*chk_size_{0}_{1});\n'.format(fun['chain_start'], fun['chain_end'])
    ret = ret + 'index = index + size({0}_{1}_points, 2)*chk_size_{0}_{1};\n'.format(fun['chain_start'], fun['chain_end'])
    ret = ret + 'figure(\'name\', \'{0}_{1}\', \'NumberTitle\', \'off\');\n'.format(fun['chain_start'], fun['chain_end'])
    ret = ret + 'plot(chk_{0}_{1}(1,:), chk_{0}_{1}(2,:), \'+\');\n\n'.format(fun['chain_start'], fun['chain_end'])

  for jnt in calib_links:
    ret = ret + 'e = X.get_random_var({0}_id);\n'.format(jnt)
    ret = ret + 'calibOut(\'{0}\', e.transform());\n\n'.format(jnt)
  for mkr in marker_links:
    ret = ret + 'e = X.get_random_var({0}_id);\n'.format(mkr)
    ret = ret + 'calibOut(\'{0}\', [eye(3), e; 0 0 0 1]);\n\n'.format(mkr)
  return ret

def rosconfig(marker_links, calib_joints, joints):
  ret = 'marker_links :\n'
  for mkr in marker_links:
    ret = ret + '  - {}\n'.format(mkr)
  ret = ret + 'calibration_joints :\n'
  for jnt in calib_joints:
    ret = ret + '  - {}\n'.format(jnt)
  ret = ret + 'joints_ordering :\n'
  for jnt in joints:
    ret = ret + '  - {}\n'.format(jnt)
  return ret

def genroslaunch(camera_links, pkg_name):
  ret = '''<?xml version="1.0"?>
<launch>
  <rosparam command="load" file="$(find {})/config/calibration.yaml" ns="/calibration" />

  <node pkg="robot_calibration" type="data_collector_node" name="calibration_data_collector"/>
'''.format(pkg_name)
  for cam in camera_links:
    ret = ret + '  <node pkg="robot_calibration" type="marker_detector_node" name="{0}_marker_detector">\n'.format(cam)
    ret = ret + '    <remap from="image" to="put path to image topic"/>\n'
    ret = ret + '  </node>\n'
  ret = ret + '</launch>'
  return ret

def main(args):
  #inputs
  calib_joints = ['stereo_left_joint', 'stereo_right_joint', 'torso_link2_left_arm_base_joint', 'head_base_joint', 'head_kinect_rgb_joint', 'left_hand_camera_joint']
  marker_links = ['right_gripper_calib_link1', 'right_gripper_calib_link2', 'left_gripper_calib_link1', 'left_gripper_calib_link2']
  camera_links = ['stereo_left_optical_frame', 'stereo_right_optical_frame', 'head_kinect_rgb_optical_frame', 'left_hand_camera_optical_frame']

  #internals
  chains_spec = []
  joints = []
  jnt_off = []

  #get robot model from param server
  robot = URDF.from_parameter_server()
  tree = kdl_tree_from_urdf_model(robot)
  print '\n\n'
  #generate measurement model for each camera sensor pair
  for camera in camera_links:
    for marker in marker_links:
      if ischainusefule(tree, camera, marker, calib_joints):
        jnt_off = jnt_off + jointofflist(tree, camera, marker, calib_joints, marker_links)
        chains_spec.append(genmsrfun(tree, camera, marker, calib_joints, joints))
  #generate main calibration function

  joint_offset_str = genjointoffset(joints, jnt_off)

  mat_load_str = genmatload()

  mat_main_str = matmain(calib_joints, chains_spec, marker_links, camera_links, jnt_off)

  ros_config_str = rosconfig(marker_links, calib_joints, joints)

  ros_launch_str = genroslaunch(camera_links, args[0])

  #prepare directories
  output_dir = args[0]

  matlab_dir = output_dir + '/matlab'
  launch_dir = output_dir + '/launch'
  config_dir = output_dir + '/config'

  os.mkdir(output_dir)
  os.mkdir(matlab_dir)
  os.mkdir(launch_dir)
  os.mkdir(config_dir)

  #write matlab functions
  for fun in chains_spec:
    filename = matlab_dir + '/' + fun['name'] + '.m'
    output_file = open(filename, 'w')
    output_file.write(fun['fun'])
    output_file.close()

  output_file = open(matlab_dir + '/load_data.m', 'w')
  output_file.write(mat_load_str)
  output_file.close()

  output_file = open(matlab_dir + '/jointOffset.m', 'w')
  output_file.write(joint_offset_str)
  output_file.close()

  output_file = open(matlab_dir + '/calibmain.m', 'w')
  output_file.write(mat_main_str)
  output_file.close()

  #write ROS config
  output_file = open(config_dir + '/calibration.yaml', 'w')
  output_file.write(ros_config_str)
  output_file.close()

  #write ROS launch
  output_file = open(launch_dir + '/calibration.launch', 'w')
  output_file.write(ros_launch_str)
  output_file.close()

if __name__ == "__main__":
  rospy.init_node('robot_calibration')
  main(rospy.myargv(argv=sys.argv)[1:])

