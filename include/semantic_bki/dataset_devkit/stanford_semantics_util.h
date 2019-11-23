#pragma once

#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

class StanfordSemanticsData {
  public:
    StanfordSemanticsData(ros::NodeHandle& nh,
             double resolution, double block_depth,
             double sf2, double ell,
             int num_class, double free_thresh,
             double occupied_thresh, float var_thresh, 
	           double ds_resolution,
             double free_resolution, double max_range,
             std::string map_topic,
             float prior)
      : nh_(nh)
      , resolution_(resolution)
      , ds_resolution_(ds_resolution)
      , free_resolution_(free_resolution)
      , max_range_(max_range) {
        map_ = new semantic_bki::SemanticBKIOctoMap(resolution, block_depth, num_class, sf2, ell, prior, var_thresh, free_thresh, occupied_thresh);
        m_pub_ = new semantic_bki::MarkerArrayPub(nh_, map_topic, resolution);
      	//init_trans_to_ground_ << 1, 0, 0, 0,
          //                       0, 0, 1, 0,
            //                     0,-1, 0, 1,
              //                   0, 0, 0, 1;
              //
        init_trans_to_ground_ << 1, 0, 0, 0,
                                 0, 1, 0, 0,
                                 0, 0, 1, 0,
                                 0, 0, 0, 1;
      }

    bool read_lidar_poses(const std::string lidar_pose_name) {
      if (std::ifstream(lidar_pose_name)) {
        std::ifstream fPoses;
        fPoses.open(lidar_pose_name.c_str());
        while (!fPoses.eof()) {
          std::string s;
          std::getline(fPoses, s);
          if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            Eigen::Matrix4d t_matrix = Eigen::Matrix4d::Identity();
            for (int i = 0; i < 3; ++i)
              for (int j = 0; j < 4; ++j)
                ss >> t_matrix(i, j);
            lidar_poses_.push_back(t_matrix);
          }
        }
        fPoses.close();
        return true;
        } else {
         ROS_ERROR_STREAM("Cannot open evaluation list file " << lidar_pose_name);
         return false;
      }
    } 

    bool process_scans(std::string input_data_dir, std::string input_label_dir, int scan_num, bool query, bool visualize) {
      semantic_bki::point3f origin;

      std::vector<int> semantic_labels = {1, 2, 2, 3, 3, 3, 3, 3, 3, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 8, 9, 10, 10, 10, 10, 10, 10};
      
      auto start = std::chrono::high_resolution_clock::now();
      for (int scan_id  = 1; scan_id <= scan_num; ++scan_id) {
        char scan_id_c[256];
        sprintf(scan_id_c, "%06d", scan_id);
        std::string scan_name = input_data_dir + std::string(scan_id_c) + ".pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile<pcl::PointXYZ> (scan_name, *cloud);
        pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud(new pcl::PointCloud<pcl::PointXYZL>);
        for (int i = 0; i < cloud->points.size(); ++i) {
          pcl::PointXYZL point;
          point.x = cloud->points[i].x;
          point.y = cloud->points[i].y;
          point.z = cloud->points[i].z;
          point.label = semantic_labels[scan_id - 1];
          labeled_cloud->push_back(point);
        }

      	auto start = std::chrono::high_resolution_clock::now();
	      map_->insert_pointcloud_csm(*labeled_cloud, origin, ds_resolution_, free_resolution_, max_range_);
      	auto finish = std::chrono::high_resolution_clock::now();
	      std::chrono::duration<double> elapsed = finish - start;
      	std::cout << "Elapsed time for scan " << scan_name << " is: " << elapsed.count() << " s\n";
        
  
        if (visualize)
	        publish_map();
      }
      auto finish = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = finish - start;
      std::cout << "Elapsed time: " << elapsed.count() << " s\n";
      
      if (query) {
        query_scan(scan_num);
      }

      
      return 1;
    }

    void publish_map() {
      m_pub_->clear_map(resolution_);
      for (auto it = map_->begin_leaf(); it != map_->end_leaf(); ++it) {
        if (it.get_node().get_state() == semantic_bki::State::OCCUPIED) {
          semantic_bki::point3f p = it.get_loc();
          m_pub_->insert_point3d_semantics(p.x(), p.y(), p.z(), it.get_size(), it.get_node().get_semantics(), 2);
        }
      }
      m_pub_->publish();
    }

    void set_up_evaluation(const std::string query_data_dir, const std::string evaluation_result_dir) {
      query_data_dir_ = query_data_dir;
      evaluation_result_dir_ = evaluation_result_dir;
    }

    void query_scan(int scan_num) {
      for (int scan_id  = 1; scan_id <= scan_num; ++scan_id) {
        char scan_id_c[256];
        sprintf(scan_id_c, "%06d", scan_id);
        std::string scan_name = query_data_dir_ + std::string(scan_id_c) + ".pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile<pcl::PointXYZ> (scan_name, *cloud1);
        
        std::string result_name = evaluation_result_dir_ + std::string(scan_id_c) + ".txt";
        
        sprintf(scan_id_c, "%06d", scan_id + scan_num);
        scan_name = query_data_dir_ + std::string(scan_id_c) + ".pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile<pcl::PointXYZ> (scan_name, *cloud2);
 
        std::ofstream result_file;
        result_file.open(result_name);
        for (int i = 0; i < cloud1->points.size(); ++i) {
          int occupancy = 0;
          for (float x = cloud1->points[i].x+0.01; x < cloud2->points[i].x; x += resolution_) {
            for (float y = cloud1->points[i].y+0.01; y < cloud2->points[i].y; y += resolution_) {
              for (float z = cloud1->points[i].z+0.01; z < cloud2->points[i].z; z += resolution_) {
                semantic_bki::SemanticOcTreeNode node = map_->search(x, y, z);
                if (node.get_state() == semantic_bki::State::OCCUPIED)
                  occupancy = 1;
              }
            }
          }
          result_file << occupancy << "\n";
        }
        result_file.close();
       }
    }

  
  private:
    ros::NodeHandle nh_;
    double resolution_;
    double ds_resolution_;
    double free_resolution_;
    double max_range_;
    semantic_bki::SemanticBKIOctoMap* map_;
    semantic_bki::MarkerArrayPub* m_pub_;
    //ros::Publisher color_octomap_publisher_;
    tf::TransformListener listener_;
    std::ofstream pose_file_;
    std::vector<Eigen::Matrix4d> lidar_poses_;
    std::string query_data_dir_;
    std::string evaluation_result_dir_;
    Eigen::Matrix4d init_trans_to_ground_;

    int check_element_in_vector(const long long element, const std::vector<long long>& vec_check) {
      for (int i = 0; i < vec_check.size(); ++i)
        if (element == vec_check[i])
          return i;
      return -1;
    }

};
