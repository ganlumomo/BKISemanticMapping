#pragma once

#include <fstream>
#include <math.h>
#include <unordered_set>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <nav_msgs/OccupancyGrid.h>

using namespace std::chrono;

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

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
        m_pub_ = new semantic_bki::MarkerArrayPub(nh_, map_topic_, resolution_);
        
        // for occupancy grid
        occupancy_grid_ptr_ = std::make_shared<nav_msgs::OccupancyGrid>(nav_msgs::OccupancyGrid());
        occupancy_grid_ptr_->info.resolution = 0.1;
        occupancy_grid_ptr_->info.width = 2000;
        occupancy_grid_ptr_->info.height = 2000;
        occupancy_grid_ptr_->info.origin.position.x = -100.0;
        occupancy_grid_ptr_->info.origin.position.y = -100.0;
        occupancy_grid_ptr_->info.origin.position.z = 0.0;
        occupancy_grid_ptr_->info.origin.orientation.x = 0.0;
        occupancy_grid_ptr_->info.origin.orientation.y = 0.0;
        occupancy_grid_ptr_->info.origin.orientation.z = 0.0;
        occupancy_grid_ptr_->info.origin.orientation.w = 1.0;
        occupancy_grid_ptr_->data.resize(occupancy_grid_ptr_->info.width * occupancy_grid_ptr_->info.height);
        std::fill(occupancy_grid_ptr_->data.begin(), occupancy_grid_ptr_->data.end(), -1);
        occupancy_grid_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 10);
        
        for (float i = -5; i <= 15; i += 0.1) {
          xs.push_back(i);
          ys.push_back(i);
        //zs.push_back(i);
        }
        for (float i = -3; i < 0; i += 0.1)
          zs.push_back(i);
      }

    // Data preprocess
    void PointCloudCallback(const sensor_msgs::PointCloudConstPtr& cloud_msg) {
      //if (cloud_msg->header.frame_id != "/velodyne_actual")
        //return;       

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
        std::cout << cloud_msg->header.frame_id << std::endl;
      } catch (tf::TransformException ex) {
        //std::cout<<"tf look for failed\n";
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

      // for occupancy grid
      std::unordered_set<int> target_labels{2,3};
      build_occupancy_grid(cloud, origin,target_labels);
    }

    void publish_map() {
      m_pub_->clear_map(resolution_);
      for (auto it = map_->begin_leaf(); it != map_->end_leaf(); ++it) {
        if (it.get_node().get_state() == semantic_bki::State::OCCUPIED) {
          semantic_bki::point3f p = it.get_loc();
          m_pub_->insert_point3d_semantics(p.x(), p.y(), p.z(), it.get_size(), it.get_node().get_semantics(), 3);
        }
      }
      m_pub_->publish();
    }

    void build_occupancy_grid(const semantic_bki::PCLPointCloud& cloud, const semantic_bki::point3f origin, std::unordered_set<int>& target_labels) {
      
      //int map_index = MAP_IDX(occupancy_grid_ptr_->info.width,
                              //int((origin.x() - occupancy_grid_ptr_->info.origin.position.x) / occupancy_grid_ptr_->info.resolution ),
                              //int((origin.y() - occupancy_grid_ptr_->info.origin.position.y) / occupancy_grid_ptr_->info.resolution));
      //occupancy_grid_ptr_->data[map_index] = 100;
      
      
      //for (int i = 0; i < cloud.points.size(); ++i) {
      for (auto x : xs) {
        x += origin.x();
        for (auto y : ys) {
        y += origin.y();
          for(auto z : zs) {
        //float x = cloud.points[i].x;
        //float y = cloud.points[i].y;
        //float z = cloud.points[i].z;
        z += origin.z();

        int map_index = MAP_IDX(occupancy_grid_ptr_->info.width,
                                int((x - occupancy_grid_ptr_->info.origin.position.x) / occupancy_grid_ptr_->info.resolution ),
                                int((y - occupancy_grid_ptr_->info.origin.position.y) / occupancy_grid_ptr_->info.resolution));
        if (map_index >= occupancy_grid_ptr_->info.width * occupancy_grid_ptr_->info.height) {
          continue;
        }
        if (occupancy_grid_ptr_->data[map_index] == 0) {
          continue;
        }

        // search semantics
        semantic_bki::SemanticOcTreeNode node = map_->search(x, y, z);
        if (node.get_state() == semantic_bki::State::OCCUPIED) {
          int label = node.get_semantics();
          if (target_labels.find(label) != target_labels.end())
            occupancy_grid_ptr_->data[map_index] = 0; // free
          else
            occupancy_grid_ptr_->data[map_index] = 100;
        }
      }
        }
      }

        occupancy_grid_ptr_->header.frame_id = "/map";
        occupancy_grid_publisher_.publish(*occupancy_grid_ptr_); 
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
    semantic_bki::MarkerArrayPub* m_pub_;
    tf::TransformListener listener_;
    // for occupancy grid
    std::shared_ptr<nav_msgs::OccupancyGrid> occupancy_grid_ptr_;
    ros::Publisher occupancy_grid_publisher_;
        std::vector<float> xs;
        std::vector<float> ys;
        std::vector<float> zs;

    };
