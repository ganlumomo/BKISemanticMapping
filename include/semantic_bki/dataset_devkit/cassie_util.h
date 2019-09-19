#pragma once

#include <fstream>
#include <math.h>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

using namespace std::chrono;

class CassieData {
  public:
    CassieData(ros::NodeHandle& nh,
             double resolution, double block_depth, int num_class,
             double sf2, double ell, float prior,
             float var_thresh, double free_thresh, double occupied_thresh,
             double ds_resolution, double free_resolution, double max_range,
             std::string map_topic, bool visualize)
      : nh_(nh)
      , resolution_(resolution)
      , ds_resolution_(ds_resolution)
      , free_resolution_(free_resolution)
      , max_range_(max_range)
      , map_topic_(map_topic)
      , visualize_(visualize) {
        map_ = new semantic_bki::SemanticBKIOctoMap(resolution, block_depth, num_class, sf2, ell, prior, var_thresh, free_thresh, occupied_thresh);
      }

    // Data preprocess
    void PointCloudCallback(const sensor_msgs::PointCloudConstPtr& cloud_msg) {
      auto start = high_resolution_clock::now();
      long long cloud_msg_time = (long long)(round((double)cloud_msg->header.stamp.toNSec() / 1000.0) + 0.1);

      // Save pcd files
      semantic_bki::PCLPointCloud cloud;
      semantic_bki::point3f origin;
      
      for (int i = 0; i < cloud_msg->points.size(); ++i) {
        pcl::PointXYZL pt;
        pt.x = cloud_msg->points[i].x;
        pt.y = cloud_msg->points[i].y;
        pt.z = cloud_msg->points[i].z;
        pt.label = cloud_msg->channels[0].values[i];
  
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))
          continue;
        if (pt.label == 0 || pt.label == 13)  // Note: don't project background and sky
          continue;
        cloud.push_back(pt);
      }

      // Fetch the tf transform and write to a file
      tf::StampedTransform transform;
      try {
        listener_.lookupTransform("/odom",
                                  cloud_msg->header.frame_id,
                                  cloud_msg->header.stamp,
                                  transform);
      } catch (tf::TransformException ex) {
        std::cout<<"tf look for failed\n";
        ROS_ERROR("%s",ex.what());
        return;
      }

      Eigen::Affine3d t_eigen;
      tf::transformTFToEigen(transform, t_eigen);

      // Transform point cloud
      pcl::transformPointCloud(cloud, cloud, t_eigen);
      origin.x() = t_eigen.matrix()(0, 3);
      origin.y() = t_eigen.matrix()(1, 3);
      origin.z() = t_eigen.matrix()(2, 3);
      map_->insert_pointcloud(cloud, origin, ds_resolution_, free_resolution_, max_range_);
      auto stop = high_resolution_clock::now();
      auto duration = duration_cast<milliseconds>(stop - start);
      std::cout << "Finished in " << duration.count() << " ms." << std::endl;
      
      if (visualize_)
        publish_map();
    }

    void publish_map() {
      semantic_bki::MarkerArrayPub m_pub(nh_, map_topic_, resolution_);
      for (auto it = map_->begin_leaf(); it != map_->end_leaf(); ++it) {
        if (it.get_node().get_state() == semantic_bki::State::OCCUPIED) {
          semantic_bki::point3f p = it.get_loc();
          m_pub.insert_point3d_semantics(p.x(), p.y(), p.z(), it.get_size(), it.get_node().get_semantics(), 3);
        }
      }
      m_pub.publish();
    }

  private:
    ros::NodeHandle nh_;
    double resolution_;
    double ds_resolution_;
    double free_resolution_;
    double max_range_;
    std::string map_topic_;
    bool visualize_;
    semantic_bki::SemanticBKIOctoMap* map_;
    tf::TransformListener listener_;
};
