#pragma once

#include <fstream>
#include <math.h>
#include <unordered_set>
#include <queue>
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
             std::string map_topic, std::string static_frame, bool visualize)
      : nh_(nh)
      , resolution_(resolution)
      , ds_resolution_(ds_resolution)
      , free_resolution_(free_resolution)
      , max_range_(max_range)
      , map_topic_(map_topic)
      , static_frame_(static_frame)
      , visualize_(visualize) {
        // semantic map
        map_ = new semantic_bki::SemanticBKIOctoMap(resolution, block_depth, num_class, sf2, ell, prior, var_thresh, free_thresh, occupied_thresh);
        m_pub_ = new semantic_bki::MarkerArrayPub(nh_, map_topic_, resolution_, static_frame_);

        // occupancy grid
        occupancy_grid_ptr_ = std::make_shared<nav_msgs::OccupancyGrid>(nav_msgs::OccupancyGrid());
        occupancy_grid_ptr_->info.resolution = resolution;
        occupancy_grid_ptr_->info.width = 1000 * int(0.2/resolution);
        occupancy_grid_ptr_->info.height = 1000 * int(0.2/resolution);
        occupancy_grid_ptr_->info.origin.position.x = -100.0;
        occupancy_grid_ptr_->info.origin.position.y = -100.0;
        occupancy_grid_ptr_->info.origin.position.z = 0.0;
        occupancy_grid_ptr_->info.origin.orientation.x = 0.0;
        occupancy_grid_ptr_->info.origin.orientation.y = 0.0;
        occupancy_grid_ptr_->info.origin.orientation.z = 0.0;
        occupancy_grid_ptr_->info.origin.orientation.w = 1.0;
        occupancy_grid_ptr_->data.resize(occupancy_grid_ptr_->info.width * occupancy_grid_ptr_->info.height);
        std::fill(occupancy_grid_ptr_->data.begin(), occupancy_grid_ptr_->data.end(), -1);  // unknown
        occupancy_grid_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 10);
        for (float i = -20; i <= 20; i += resolution) {
          xs_.push_back(i);
          ys_.push_back(i);
        }
        for (float i = -2; i < -1; i += resolution)
          zs_.push_back(i);
        target_labels_ = std::unordered_set<int>{2,3};

        // cost map
        cost_map_ptr_ = std::make_shared<nav_msgs::OccupancyGrid>(nav_msgs::OccupancyGrid());
        cost_map_ptr_->info = occupancy_grid_ptr_->info;
        cost_map_ptr_->data.resize(cost_map_ptr_->info.width * cost_map_ptr_->info.height);
        std::fill(cost_map_ptr_->data.begin(), cost_map_ptr_->data.end(), 126);
        cost_map_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("cost_map", 10);
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
        listener_.lookupTransform(static_frame_,
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

      build_occupancy_grid(origin);
      update_cost_map();
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

    void build_occupancy_grid(const semantic_bki::point3f& origin) {
      for (auto x : xs_) {
        x += origin.x();
        for (auto y : ys_) {
          y += origin.y();
          int map_index = MAP_IDX(occupancy_grid_ptr_->info.width,
                                  int((x - occupancy_grid_ptr_->info.origin.position.x) / occupancy_grid_ptr_->info.resolution),
                                  int((y - occupancy_grid_ptr_->info.origin.position.y) / occupancy_grid_ptr_->info.resolution));
          if (map_index >= occupancy_grid_ptr_->info.width * occupancy_grid_ptr_->info.height)
            continue;
          //if (occupancy_grid_ptr_->data[map_index] == 0)
              //continue;

          for(auto z : zs_) {
            z += origin.z();
            // search semantics
            semantic_bki::SemanticOcTreeNode node = map_->search(x, y, z);
            if (node.get_state() == semantic_bki::State::OCCUPIED) {
              int label = node.get_semantics();
              if (target_labels_.find(label) != target_labels_.end())
                occupancy_grid_ptr_->data[map_index] = 0; // free
              else
                occupancy_grid_ptr_->data[map_index] = 100;
            }
          }
        }
      }
      occupancy_grid_ptr_->header.frame_id = static_frame_;
      occupancy_grid_publisher_.publish(*occupancy_grid_ptr_); 
    }

    void update_cost_map() {
      std::queue<int> grids_queue;
      std::vector<float> distance;
      distance.resize(cost_map_ptr_->info.width * cost_map_ptr_->info.height);

      // Initialize distance
      for (int i = 0; i < occupancy_grid_ptr_->info.width * occupancy_grid_ptr_->info.height; ++i) {
        if (occupancy_grid_ptr_->data[i] > 0)  // occupied
          distance[i] = 0.0f;
        else if (occupancy_grid_ptr_->data[i] < 0)  // unknown
          distance[i] = occupancy_grid_ptr_->data[i];
        else  // free
          distance[i] = std::numeric_limits<float>::infinity();
      }
      
      for (int i = 0; i < occupancy_grid_ptr_->info.width * occupancy_grid_ptr_->info.height; ++i) {
        if (distance[i] == 0) {
          std::vector<int> neighbors = find_neighbors(i, occupancy_grid_ptr_->info.width, occupancy_grid_ptr_->info.height);
          for (auto it = neighbors.begin(); it != neighbors.end(); ++it) {
            if (distance[*it] == std::numeric_limits<float>::infinity())
              grids_queue.push(*it);
          }
        }
      }

      while(!grids_queue.empty()) {
        int grid = grids_queue.front();
        grids_queue.pop();
        if (distance[grid] == std::numeric_limits<float>::infinity()) {
          std::vector<int> neighbors = find_neighbors(grid, occupancy_grid_ptr_->info.width, occupancy_grid_ptr_->info.height);
          float min = std::numeric_limits<float>::infinity();
          bool found_min = false;
          for (auto it = neighbors.begin(); it != neighbors.end(); ++it) {
            if (distance[*it] >= 0 && distance[*it] < min) {
              min = distance[*it];
              found_min = true;
            }
            if (distance[*it] == std::numeric_limits<float>::infinity())
              grids_queue.push(*it);
          }

          //std::cout << "queue size: " << grids_queue.size() << std::endl;
          if (found_min)
            distance[grid] = 1 + min;
        }
      }
      
      // Write distance to cost map
      for (int i = 0; i < cost_map_ptr_->info.width * cost_map_ptr_->info.height; ++i) {
        if (distance[i] <= 126)
          cost_map_ptr_->data[i] = (int8_t) distance[i];
        else
          cost_map_ptr_->data[i] = 126;
      }
      cost_map_ptr_->header.frame_id = static_frame_;
      cost_map_publisher_.publish(*cost_map_ptr_);
    }

  private:
    ros::NodeHandle nh_;
    double resolution_;
    double ds_resolution_;
    double free_resolution_;
    double max_range_;
    std::string map_topic_;
    std::string static_frame_;
    bool visualize_;
    // semantic map
    semantic_bki::SemanticBKIOctoMap* map_;
    semantic_bki::MarkerArrayPub* m_pub_;
    tf::TransformListener listener_;
    // occupancy grid
    std::shared_ptr<nav_msgs::OccupancyGrid> occupancy_grid_ptr_;
    ros::Publisher occupancy_grid_publisher_;
    std::vector<float> xs_;
    std::vector<float> ys_;
    std::vector<float> zs_;
    std::unordered_set<int> target_labels_;
    // cost map
    std::shared_ptr<nav_msgs::OccupancyGrid> cost_map_ptr_;
    ros::Publisher cost_map_publisher_;
    
    std::vector<int> find_neighbors(int index, int width, int height) {
      std::vector<int> neighbors{index-width-1, index-width, index-width+1,index-1, index+1, index+width-1, index+width, index+width+1};
      for (auto it = neighbors.begin(); it != neighbors.end(); ) {
        if ((*it >= 0) && ( *it < width * height))
          ++it;
        else
          it = neighbors.erase(it);
      }
      return neighbors;
    }
};
