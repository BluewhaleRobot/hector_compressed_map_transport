//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <opencv2/core/core.hpp>

using namespace std;

/**
 * @brief This node provides occupancy grid maps as images via image_transport, so the transmission consumes less bandwidth.
 * The provided code is a incomplete proof of concept.
 */
class MapAsImageProvider
{
public:
  MapAsImageProvider()
    : pn_("~")
  {

    image_transport_ = new image_transport::ImageTransport(n_);
    image_transport_publisher_full_ = image_transport_->advertise("map_image/full", 1);
    image_transport_publisher_tile_ = image_transport_->advertise("map_image/tile", 1);

    odom_sub_ = n_.subscribe("/odom", 1, &MapAsImageProvider::odomCallback, this);
    map_sub_ = n_.subscribe("/map", 1, &MapAsImageProvider::mapCallback, this);
    //Which frame_id makes sense?
    cv_img_full_.header.frame_id = "map_image";
    cv_img_full_.encoding = "bgr8";
    map_mat = new cv::Mat(1, 1, CV_8U);

    //Fixed cell width for tile based image, use dynamic_reconfigure for this later
    p_size_tiled_map_image_x_ = 64;
    p_size_tiled_map_image_y_ = 64;

    ROS_INFO("Map to Image node started.");

    mCurrentPos_ = geometry_msgs::PoseStamped();
    mPoseUpdate_ = false;

  	pose_points = new cv::Point[4];
  	pose_points[0] = cv::Point(-6, 6);
  	pose_points[1] = cv::Point(10, 0);
  	pose_points[2] = cv::Point(-6, 6);
  	pose_points[3] = cv::Point(0, 0);
  	pose_npts = 4;

     mat_point0 = cv::Mat::eye(4, 4, CV_32F);
     mat_point0.at<float>(0,3) = -6;
     mat_point0.at<float>(1,3) = -6;

     mat_point1 = cv::Mat::eye(4, 4, CV_32F);
     mat_point1.at<float>(0,3) = 10;
     mat_point1.at<float>(1,3) = 0;

     mat_point2 = cv::Mat::eye(4, 4, CV_32F);
     mat_point2.at<float>(0,3) = -6;
     mat_point2.at<float>(1,3) = 6;

     mat_point3 = cv::Mat::eye(4, 4, CV_32F);
     mat_point3.at<float>(0,3) = 0;
     mat_point3.at<float>(1,3) = 0;
  }

  ~MapAsImageProvider()
  {
    delete image_transport_;
  }

  //We assume the robot position is available as a PoseStamped here (querying tf would be the more general option)
  void odomCallback(const nav_msgs::OdometryConstPtr& odom)
  {
    mCurrentPos_.header = odom->header;
    mCurrentPos_.pose = odom->pose.pose;
    mPoseUpdate_ = true;
  }

  //The map_ptr->image conversion runs every time a new map is received at the moment
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
  {
    map_ptr = map;
  }

  void drawMap(){
    // Only if someone is subscribed to it, do work and publish full map image
    if (image_transport_publisher_full_.getNumSubscribers() > 0){
      if(!(map_ptr))
        return;
      int size_x = map_ptr->info.width;
      int size_y = map_ptr->info.height;

      if ((size_x < 3) || (size_y < 3) ){
        ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
        return;
      }

      cv::Mat *map_color = &cv_img_full_.image;
      // resize cv image if it doesn't have the same dimensions as the map
      if ( (map_mat->rows != size_y) || (map_mat->cols != size_x)){
        *map_mat = cv::Mat(size_y, size_x, CV_8U);
        *map_color = cv::Mat(size_y, size_x, CV_8UC3);
      }

      const std::vector<int8_t>& map_data (map_ptr->data);

      unsigned char *map_mat_data_p=(unsigned char*) map_mat->data;

      //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
      int size_y_rev = size_y-1;

      for (int y = size_y_rev; y >= 0; --y){

        int idx_map_y = size_x * (size_y_rev -y);
        int idx_img_y = size_x * y;

        for (int x = 0; x < size_x; ++x){

          int idx = idx_img_y + x;
          int8_t mapGrid_data_temp = map_data[idx_map_y + x];

          if(mapGrid_data_temp>100) mapGrid_data_temp=100;

          switch (mapGrid_data_temp)
          {
          case -1:
            map_mat_data_p[idx] = 127;
            break;

          case 0:
            map_mat_data_p[idx] = 255;
            break;

          default:
            map_mat_data_p[idx] = (100 - mapGrid_data_temp)*255/100;
            break;
          }
        }
      }
      cvtColor(*map_mat, *map_color, CV_GRAY2BGR, 3);
      // draw pose
      if(mPoseUpdate_){

        geometry_msgs::PoseStamped pose_map;
        try{
          ros::Time now = mCurrentPos_.header.stamp;
          transform_listener.waitForTransform("/map", mCurrentPos_.header.frame_id,
                              now, ros::Duration(3.0));
          transform_listener.transformPose("/map", mCurrentPos_, pose_map);
        }catch(tf::TransformException ex){
           ROS_ERROR("transfrom exception : %s",ex.what());
        }

        float resolution = map_ptr->info.resolution;
        float map_right_pos = map_ptr->info.origin.position.x + map_ptr->info.width * resolution; //pose_ptr_->pose.position.x + map_ptr->info.width * resolution;
        float map_top_pos = map_ptr->info.origin.position.y + map_ptr->info.height* resolution; //pose_ptr_->pose.position.y + map_ptr->info.height * resolution;
        if(pose_map.pose.position.x > map_ptr->info.origin.position.x &&
          pose_map.pose.position.y > map_ptr->info.origin.position.y &&
          pose_map.pose.position.x < map_right_pos &&
          pose_map.pose.position.y < map_top_pos
        ){
          int pose_x = int((pose_map.pose.position.x - map_ptr->info.origin.position.x) / resolution);
          int pose_y = size_y - int((pose_map.pose.position.y - map_ptr->info.origin.position.y) / resolution);

          //transform points
          tf::Matrix3x3 Rbc(tf::Quaternion(pose_map.pose.orientation.x, pose_map.pose.orientation.y, pose_map.pose.orientation.z, pose_map.pose.orientation.w));
          cv::Mat Tbc = cv::Mat::eye(4, 4, CV_32F);
          for (int i = 0; i < 3; i++)
          {
              tf::Vector3 v = Rbc.getColumn(i);
              Tbc.at<float>(0, i) = v.getX();
              Tbc.at<float>(1, i) = v.getY();
              Tbc.at<float>(2, i) = v.getZ();
          }
          cv::Mat point0 = Tbc*mat_point0;
          cv::Mat point1 = Tbc*mat_point1;
          cv::Mat point2 = Tbc*mat_point2;
          cv::Mat point3 = Tbc*mat_point3;

        	pose_points[0].x = pose_x+int(point0.at<float>(0,3));
          pose_points[0].y = pose_y-int(point0.at<float>(1,3));

          pose_points[1].x = pose_x+int(point1.at<float>(0,3));
          pose_points[1].y = pose_y-int(point1.at<float>(1,3));

          pose_points[2].x = pose_x+int(point2.at<float>(0,3));
          pose_points[2].y = pose_y-int(point2.at<float>(1,3));

          pose_points[3].x = pose_x+int(point3.at<float>(0,3));
          pose_points[3].y = pose_y-int(point3.at<float>(1,3));

          //cv::fillConvexPoly(*map_color, pose_points, pose_npts, cv::Scalar(0, 0,255));
          // draw
          cv::Point p(pose_x , pose_y);
          cv::circle(*map_color, p, 5, cv::Scalar(0, 0,255), -1);

        }
      }
      image_transport_publisher_full_.publish(cv_img_full_.toImageMsg());
    }
  }

  ros::Subscriber map_sub_;
  ros::Subscriber odom_sub_;

  image_transport::Publisher image_transport_publisher_full_;
  image_transport::Publisher image_transport_publisher_tile_;

  image_transport::ImageTransport* image_transport_;

  geometry_msgs::PoseStamped mCurrentPos_;
  bool mPoseUpdate_;
  nav_msgs::OccupancyGridConstPtr map_ptr;

  cv_bridge::CvImage cv_img_full_;
  cv::Mat *map_mat;

  ros::NodeHandle n_;
  ros::NodeHandle pn_;

  int p_size_tiled_map_image_x_;
  int p_size_tiled_map_image_y_;

  tf::TransformListener transform_listener;

  int pose_npts;
  cv::Point * pose_points;
  cv::Mat mat_point0;
  cv::Mat mat_point1;
  cv::Mat mat_point2;
  cv::Mat mat_point3;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_to_image_node");
  MapAsImageProvider mapAsImageProvider;
  ros::Rate r(30);

  while (ros::ok()) {
    ros::spinOnce();
    mapAsImageProvider.drawMap();
    r.sleep();
  }
  return 0;
}
