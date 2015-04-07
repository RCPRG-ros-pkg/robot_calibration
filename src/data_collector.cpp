/* Copyright (c) 2015, Warsaw University of Technology
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
       * Redistributions of source code must retain the above copyright
         notice, this list of conditions and the following disclaimer.
       * Redistributions in binary form must reproduce the above copyright
         notice, this list of conditions and the following disclaimer in the
         documentation and/or other materials provided with the distribution.
       * Neither the name of the Warsaw University of Technology nor the
         names of its contributors may be used to endorse or promote products
         derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL Warsaw University of Technology BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
#include <map>
#include <vector>
#include <utility>

#include <ros/ros.h>
#include <robot_calibration/CalibrationMarker.h>
#include <sensor_msgs/JointState.h>

#include <kdl_parser/kdl_parser.hpp>

#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <boost/circular_buffer.hpp>

#include <matio.h>

class DataCollector {
  typedef std::pair<std::vector<double>, robot_calibration::CalibrationMarker> Msr;
  typedef std::pair<std::string, KDL::Frame> CalibPair;

  ros::NodeHandle nh_;
  ros::Subscriber marker_sub_;
  ros::Subscriber joint_state_sub_;
  boost::circular_buffer<robot_calibration::CalibrationMarker> marker_buffer_;
  boost::circular_buffer<sensor_msgs::JointState> joint_state_buffer_;
  std::vector<std::string> joints_ordering_;
  std::vector<std::string> calibration_joints_;
  std::vector<std::string> marker_links_;
  std::vector<CalibPair> initial_calib_joints_;
  std::vector<CalibPair> initial_calib_marker_;
  std::map<std::string, std::vector<Msr> > measurements_;
  std::map<std::string, std::vector<double> > measurements_sigma_;
  std::map<std::string, sensor_msgs::CameraInfo> camera_info_;

  ros::Duration cam_offset_;

  double min_distance_;
  double vel_mul_;

  KDL::Tree robot_tree_;

 public:
  DataCollector() : marker_buffer_(10),
                    joint_state_buffer_(1000),
                    cam_offset_(0.0),
                    min_distance_(0.01),
                    vel_mul_(1.0) {
    marker_sub_ = nh_.subscribe<robot_calibration::CalibrationMarker>("marker", 10, &DataCollector::markerCB, this);
    joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 10, &DataCollector::jointStateCB, this);

    nh_.getParam("/calibration/joints_ordering", joints_ordering_);
    nh_.getParam("/calibration/calibration_joints", calibration_joints_);
    nh_.getParam("/calibration/marker_links", marker_links_);
    nh_.getParam("/calibration/minimum_distance", min_distance_);
    nh_.getParam("/calibration/vel_multiplier", vel_mul_);
  }

  ~DataCollector() {
  }

  void markerCB(const robot_calibration::CalibrationMarker::ConstPtr& msg) {
    marker_buffer_.push_back(*msg);
  }

  void jointStateCB(const sensor_msgs::JointState::ConstPtr& msg) {
    if (msg->name.size() >= joints_ordering_.size()) {
      bool found_all = true;
      for (int i = 0; i < joints_ordering_.size(); i++) {
        bool found_tmp = false;
        for (int j = 0; j < msg->name.size(); j++) {
          if (joints_ordering_[i] == msg->name[j]) {
            found_tmp = true;
            break;
          }
        }
        if (!found_tmp) {
          found_all = false;
          break;
        }
      }

      if (found_all) {
        joint_state_buffer_.push_back(*msg);
      }
    }
  }

  KDL::Frame getJoint(const KDL::Tree& tree, const std::string& name) {
    KDL::SegmentMap segments = tree.getSegments();

    for (KDL::SegmentMap::iterator iter = segments.begin(); iter != segments.end(); ++iter) {
      KDL::Segment seg = iter->second.segment;

      if ((seg.getJoint().getName() == name) || seg.getName() == name) {
        return seg.getFrameToTip();
      }
    }
    return KDL::Frame();
  }

  double getMarkerVel(const std::string &camera_link,
                      const std::string &marker_link,
                      const sensor_msgs::JointState& jnt_state,
                      const sensor_msgs::CameraInfo camera_info) {
    KDL::JntArray jnt_pos;
    KDL::JntArray jnt_vel;
    KDL::Chain chain;

    robot_tree_.getChain(camera_link, marker_link, chain);

    jnt_pos.resize(chain.getNrOfJoints());
    jnt_vel.resize(chain.getNrOfJoints());

    size_t k = 0;

    for (size_t i = 0; i < chain.getNrOfSegments(); i++) {
      KDL::Joint jnt = chain.getSegment(i).getJoint();

      if (jnt.getType() != KDL::Joint::None) {
        size_t n;
        for (n = 0; n < jnt_state.name.size(); n++) {
          if (jnt_state.name[n] == jnt.getName()) {
            break;
          }
        }

        jnt_pos(k) = jnt_state.position[n];
        jnt_vel(k) = jnt_state.velocity[n];
        ++k;
      }
    }

    KDL::ChainJntToJacSolver jac_solver(chain);

    KDL::Jacobian jac(chain.getNrOfJoints());
    jac_solver.JntToJac(jnt_pos, jac);
    KDL::Twist vel;
    KDL::MultiplyJacobian(jac, jnt_vel, vel);

    KDL::ChainFkSolverPos_recursive fk(chain);

    KDL::Frame pos;
    fk.JntToCart(jnt_pos, pos);

    KDL::Vector2 cvel;

    cvel(0) = (camera_info.P[0] / pos.p.z()) * vel.vel.x();
    cvel(1) = (camera_info.P[5] / pos.p.z()) * vel.vel.x();

    return sqrt(cvel(0) * cvel(0) + cvel(1) * cvel(1));
  }

  void run() {
    KDL::Tree my_tree;
    std::string robot_desc_string;
    nh_.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, robot_tree_)) {
      ROS_ERROR("Failed to construct kdl tree");
      return;
    }

    for (size_t i = 0; i < calibration_joints_.size(); i++) {
      std::cout << calibration_joints_[i] << std::endl;
      CalibPair pair = CalibPair(calibration_joints_[i],
                                 getJoint(robot_tree_, calibration_joints_[i]));
      initial_calib_joints_.push_back(pair);
    }

    for (size_t i = 0; i < marker_links_.size(); i++) {
      CalibPair pair = CalibPair(marker_links_[i],
                                 getJoint(robot_tree_, marker_links_[i]));
      initial_calib_marker_.push_back(pair);
    }

    while (ros::ok()) {
      if (!marker_buffer_.empty()) {
        for (int i = 1; i < joint_state_buffer_.size(); i++) {
          if ((marker_buffer_[0].header.stamp + cam_offset_) < joint_state_buffer_[i].header.stamp) {
            bool vel = true;

            camera_info_[marker_buffer_[0].header.frame_id] = marker_buffer_[0].camera_info;

            std::string msr_id = marker_buffer_[0].header.frame_id + "_" +
                                 marker_buffer_[0].marker_id;

            double velx = getMarkerVel(marker_buffer_[0].header.frame_id,
                                       marker_buffer_[0].marker_id,
                                       joint_state_buffer_[i], marker_buffer_[0].camera_info);

            double vel_lim = 0.1 / 0.002 * vel_mul_;

            if ((marker_buffer_[0].header.frame_id == "head_kinect_rgb_optical_frame") ||
                (marker_buffer_[0].header.frame_id == "left_hand_camera_optical_frame")) {
              vel_lim = 0.1 / 0.4 * vel_mul_;
            }

            vel = velx < vel_lim;

            std::vector<double> jnt_msr_act;
            remapJoints(joints_ordering_, joint_state_buffer_[i], &jnt_msr_act);

            bool pos = false;

            if (measurements_[msr_id].size() > 0) {
              int last = measurements_[msr_id].size() - 1;
              for (int c = 0; c < jnt_msr_act.size(); c++) {
                pos = pos || (fabs(jnt_msr_act[c] - measurements_[msr_id][last].first[c]) > min_distance_);
              }
            } else {
              pos = true;
            }

            if (vel && pos) {
              std::vector<double> jnt_msr;
              std::vector<double> jnt_msr_prev;
              remapJoints(joints_ordering_, joint_state_buffer_[i - 1], &jnt_msr_prev);

              jnt_msr.resize(joints_ordering_.size());

              for (int x = 0; x < jnt_msr_act.size(); x++) {
                jnt_msr[x] = interpolate(jnt_msr_prev[x],
                                         jnt_msr_act[x],
                                         joint_state_buffer_[i - 1].header.stamp,
                                         joint_state_buffer_[i].header.stamp,
                                         (marker_buffer_[0].header.stamp + cam_offset_));
              }

              measurements_[msr_id].push_back(Msr(jnt_msr, marker_buffer_[0]));
              measurements_sigma_[msr_id].push_back(velx);
              std::cout << "new data : " << msr_id << " [" << measurements_[msr_id].size() << "]" << std::endl;
            }
            marker_buffer_.pop_front();
            break;
          }
        }
      }

      ros::spinOnce();
      ros::Duration(0.01).sleep();
    }

    writeData();
  }

  double interpolate(double p0, double p1, ros::Time t0, ros::Time t1, ros::Time t) {
    return (p0 + (p1 - p0) * (t - t0).toSec() / (t1 - t0).toSec());
  }

  void remapJoints(const std::vector<std::string>& joint_names,
                   const sensor_msgs::JointState& jnt_state,
                   std::vector<double> *joint_out) {
    (*joint_out).resize(joint_names.size());
    for (int i = 0; i < joint_names.size(); i++) {
      for (int j = 0; j < jnt_state.name.size(); j++) {
        if (jnt_state.name[j] == joint_names[i]) {
          (*joint_out)[i] = jnt_state.position[j];
          break;
        }
      }
    }
  }

  void writeData() {
    mat_t *matfp;
    matvar_t *matvar;
    std::vector<size_t> dims(2, 0);
    matfp = Mat_CreateVer("/tmp/calibration.mat", NULL, MAT_FT_DEFAULT);

    if ( NULL == matfp ) {
      std::cout << "Error creating MAT file \"test.mat\"\n";
      return;
    }

    for (size_t i = 0; i < initial_calib_joints_.size(); i++) {
      dims[0] = 4;
      dims[1] = 4;

      std::vector<double> data(4 * 4, 0);
      initial_calib_joints_[i].second.Make4x4(&data[0]);

      matvar = Mat_VarCreate(initial_calib_joints_[i].first.c_str(), MAT_C_DOUBLE, MAT_T_DOUBLE,
                             2, &dims[0], &data[0], 0);
      if (matvar) {
        Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_NONE);
        Mat_VarFree(matvar);
      }
    }

    for (size_t i = 0; i < initial_calib_marker_.size(); i++) {
      dims[0] = 3;
      dims[1] = 1;

      matvar = Mat_VarCreate(initial_calib_marker_[i].first.c_str(), MAT_C_DOUBLE, MAT_T_DOUBLE, 2, &dims[0],
                             initial_calib_marker_[i].second.p.data, 0);
      if (matvar) {
        Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_NONE);
        Mat_VarFree(matvar);
      }
    }

    typedef std::map<std::string, std::vector<Msr> >::iterator it_type;
    for (it_type iterator = measurements_.begin(); iterator != measurements_.end(); ++iterator) {
      std::vector<double> joint_arr;
      joint_arr.reserve(joints_ordering_.size() * iterator->second.size());

      for (int i = 0; i < iterator->second.size(); i++) {
        for (int j = 0; j < iterator->second[i].first.size(); j++) {
          joint_arr.push_back(iterator->second[i].first[j]);
        }
      }

      dims[0] = joints_ordering_.size();
      dims[1] = iterator->second.size();
      std::string link_name = iterator->first;
      if (link_name[0] == '/') {
        link_name.erase(0, 1);
      }

      matvar = Mat_VarCreate((link_name + "_joints").c_str(), MAT_C_DOUBLE, MAT_T_DOUBLE, 2, &dims[0],
                             &joint_arr[0], 0);
      if (matvar) {
        Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_NONE);
        Mat_VarFree(matvar);
      }

      std::vector<double> points_arr;
      points_arr.reserve(2 * iterator->second.size());

      for (int i = 0; i < iterator->second.size(); i++) {
        points_arr.push_back(iterator->second[i].second.x);
        points_arr.push_back(iterator->second[i].second.y);
      }

      dims[0] = 2;
      dims[1] = iterator->second.size();

      matvar = Mat_VarCreate((link_name + "_points").c_str(), MAT_C_DOUBLE, MAT_T_DOUBLE, 2, &dims[0],
                             &points_arr[0], 0);
      if (matvar) {
        Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_NONE);
        Mat_VarFree(matvar);
      }

      dims[0] = 1;
      dims[1] = measurements_sigma_[iterator->first].size();

      matvar = Mat_VarCreate((link_name + "_sigmas").c_str(), MAT_C_DOUBLE, MAT_T_DOUBLE, 2, &dims[0],
                             &measurements_sigma_[iterator->first][0], 0);
      if (matvar) {
        Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_NONE);
        Mat_VarFree(matvar);
      }
    }

    typedef std::map<std::string, sensor_msgs::CameraInfo>::iterator ci_it_type;
    for (ci_it_type iterator = camera_info_.begin(); iterator != camera_info_.end(); ++iterator) {
      sensor_msgs::CameraInfo ci = iterator->second;

      dims[0] = 3;
      dims[1] = 3;

      matvar = Mat_VarCreate((iterator->first + "_K").c_str(),
                             MAT_C_DOUBLE,
                             MAT_T_DOUBLE,
                             2,
                             &dims[0],
                             &ci.K[0],
                             0);
      if (matvar) {
        Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_NONE);
        Mat_VarFree(matvar);
      }

      dims[0] = 1;
      dims[1] = 4;

      matvar = Mat_VarCreate((iterator->first + "_D").c_str(),
                             MAT_C_DOUBLE,
                             MAT_T_DOUBLE,
                             2,
                             &dims[0],
                             &ci.D[0],
                             0);
      if (matvar) {
        Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_NONE);
        Mat_VarFree(matvar);
      }
    }

    Mat_Close(matfp);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "data_collector");
  DataCollector ic;
  ic.run();
  return 0;
}



