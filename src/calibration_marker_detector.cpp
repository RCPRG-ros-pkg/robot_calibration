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
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <robot_calibration/CalibrationMarker.h>

class MarkerDetector {
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber image_sub_;
  image_geometry::PinholeCameraModel cam_model_;
  tf::TransformListener tf_listener_;
  ros::Publisher point_pub_;
  bool debug_;

  std::vector<std::string> markers_link_;

  double roi_ratio_;
  double marker_size_;

 public:
  MarkerDetector()
    : nhp_("~"), it_(nh_), tf_listener_(ros::Duration(30.0)) {
    std::string image_topic = nh_.resolveName("image");
    image_sub_ = it_.subscribeCamera(image_topic, 1, &MarkerDetector::imageCb, this);
    point_pub_ = nh_.advertise<robot_calibration::CalibrationMarker>("marker", 10);

    nh_.getParam("/calibration/marker_links", markers_link_);
    nhp_.param("roi_ratio", roi_ratio_, 2.0);
    nhp_.param("marker_size", marker_size_, 0.025);
    debug_ = true;
  }

  ~MarkerDetector() {
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cam_model_.fromCameraInfo(info_msg);

    cv::Mat debug_image;
    if (debug_) {
      debug_image = cv_ptr->image.clone();
    }

    for (int m = 0; m < markers_link_.size(); m++) {
      tf::StampedTransform transform;

      try {
        ros::Time acquisition_time = info_msg->header.stamp;
        ros::Duration timeout(1.0 / 30);
        tf_listener_.waitForTransform(cam_model_.tfFrame(), markers_link_[m],
                                      acquisition_time, timeout);
        tf_listener_.lookupTransform(cam_model_.tfFrame(), markers_link_[m],
                                     acquisition_time, transform);
      } catch (tf::TransformException& ex) {
        ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        return;
      }

      double dot = transform.getBasis().tdotz(tf::Vector3(0.0, 0.0, 1.0));

      if (dot > -0.5) {
        continue;
      }

      tf::Point pt = transform.getOrigin();
      cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
      cv::Point2d uv;
      uv = cam_model_.project3dToPixel(pt_cv);

      double marker_radius = cam_model_.getDeltaU(marker_size_, pt.z());

      if (marker_radius < 23.0) {
        continue;
      }

      int roi_size = static_cast<int>(marker_radius * roi_ratio_);

      if ((uv.x < roi_size / 2) ||
          (uv.x > (cv_ptr->image.cols - roi_size / 2)) ||
          (uv.y < roi_size / 2) ||
          (uv.y > (cv_ptr->image.rows - roi_size / 2))) {
        continue;
      }

      uv = cam_model_.unrectifyPoint(uv);

      if ((uv.x < roi_size / 2) ||
          (uv.x > (cv_ptr->image.cols - roi_size / 2)) ||
          (uv.y < roi_size / 2) ||
          (uv.y > (cv_ptr->image.rows - roi_size / 2))) {
        continue;
      }

      if (debug_) {
        int RADIUS = marker_radius / 2;
        cv::circle(debug_image, uv, RADIUS, CV_RGB(255, 0, 0), 2);
      }

      cv::Rect rect;
      rect.x = uv.x - roi_size / 2;
      rect.y = uv.y - roi_size / 2;

      rect.width = roi_size;
      rect.height = roi_size;

      cv::Mat image_roi(cv_ptr->image, rect);

      if (debug_) {
        cv::rectangle(debug_image, rect, CV_RGB(255, 0, 0));
      }

      cv::Mat src_mat_8U1;

      cv::cvtColor(image_roi, src_mat_8U1, CV_RGB2GRAY);

      int minus_c = 0;
      int half_kernel_size = 15;

      cv::Mat contours_image;

      cv::adaptiveThreshold(src_mat_8U1, contours_image, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                            cv::THRESH_BINARY, 2 * half_kernel_size + 1, minus_c);
      // ------------ Contour extraction --------------------------------------
      std::vector<std::vector<cv::Point> > contours;
      cv::findContours(contours_image, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

      // ------------ Ellipse extraction --------------------------------------
      int min_ellipse_size = static_cast<int>(marker_radius * 0.8);
      int max_ellipse_aspect_ratio = 3;

      std::vector<cv::RotatedRect> ellipses;
      for (size_t i = 0; i < contours.size(); i++) {
        size_t count = contours[i].size();
        if ( count < 6 )
          continue;
        cv::Mat pointsf;
        cv::Mat(contours[i]).convertTo(pointsf, CV_32F);
        cv::RotatedRect box = cv::fitEllipse(pointsf);
        // Plausibility checks
        double box_max = std::max(box.size.width, box.size.height);
        double box_min = std::min(box.size.width, box.size.height);
        if ( box_max > box_min * max_ellipse_aspect_ratio )
          continue;
        if (box_max > std::min(cv_ptr->image.rows, cv_ptr->image.cols) * 0.2)
          continue;
        if (box_min < 0.5 * min_ellipse_size)
          continue;
        if (box_max < min_ellipse_size)
          continue;
        if (box.center.x < 0 || box.center.x >= src_mat_8U1.cols ||
            box.center.y < 0 || box.center.y >= src_mat_8U1.rows)
          continue;
        bool add_ellipse = true;

        double carea = cv::contourArea(contours[i]);

        double earea = M_PI * box.size.width / 2 * box.size.height / 2;

        if (fabs(carea - earea) > 5.) {
          add_ellipse = false;
        }

        for (size_t j = 0; j < ellipses.size(); j++) {
          cv::Point2d diff = ellipses[j].center - box.center;
          if (norm(diff) < 1.0) {
            add_ellipse = false;
          }
        }

        if (add_ellipse) {
          ellipses.push_back(box);
        }
      }

      // find center subpixel
      if (ellipses.size() == 1) {
        std::vector<cv::Point2f> center(1);
        center[0] = ellipses[0].center;

        cv::cornerSubPix(src_mat_8U1, center, cv::Size(marker_radius * 0.2, marker_radius*0.2), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, 0.1));
        cv::Point2d p = center[0];

        robot_calibration::CalibrationMarker marker;

        marker.header = msg->header;
        marker.marker_id = markers_link_[m];
        marker.x = p.x + rect.x;
        marker.y = p.y + rect.y;
        marker.camera_info = *info_msg;

        point_pub_.publish(marker);

        if (debug_) {
          cv::RotatedRect ellipse = ellipses[0];

          ellipse.center.x += rect.x;
          ellipse.center.y += rect.y;

          cv::circle(debug_image, ellipse.center, 8, cv::Scalar(0, 255, 0), 1);

          cv::Point2d pt = center[0];

          pt.x += rect.x;
          pt.y += rect.y;

          cv::circle(debug_image, pt, 2, cv::Scalar(0, 0, 255), 2);

          cv::Point2d err = pt - uv;
          std::string err_str = boost::lexical_cast<std::string>(sqrt(err.x*err.x + err.y*err.y));

          cv::putText(debug_image, "error : " + err_str + "px",
                      cv::Point(rect.x, rect.y - 5),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5,
                      cv::Scalar(0, 0, 255), 2, 0, 0);
        }
      }
    }

    if (debug_) {
      cv::imshow("40 Ellipses", debug_image);
      cv::waitKey(3);
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_converter");
  MarkerDetector md;
  ros::spin();
  return 0;
}

