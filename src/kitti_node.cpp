#include <string>
#include <iostream>
#include <ros/ros.h>

#include "bkioctomap.h"
#include "markerarray_pub.h"
#include "kitti_util.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "kitti_node");
    ros::NodeHandle nh("~");

    std::string map_topic("/occupied_cells_vis_array");
    int block_depth = 4;
    double sf2 = 1.0;
    double ell = 1.0;
    float prior = 1.0f;
    float var_thresh = 1.0f;
    double free_thresh = 0.3;
    double occupied_thresh = 0.7;
    double resolution = 0.1;
    int num_class = 12;
    double free_resolution = 0.5;
    double ds_resolution = 0.1;
    int scan_num = 0;
    double max_range = -1;
    
    // KITTI 05
    std::string dir;
    std::string left_img_prefix;
    std::string depth_img_prefix;
    std::string label_bin_prefix;
    std::string camera_pose_file;
    std::string evaluation_list_file;
    std::string reproj_img_prefix;
    int image_width = 1226;
    int image_height = 370;
    float focal_x = 707.0912;
    float focal_y = 707.0912;
    float center_x = 601.8873;
    float center_y = 183.1104;
    float depth_scaling = 2000;
    bool reproject = false;
    bool visualize = false;

    nh.param<int>("block_depth", block_depth, block_depth);
    nh.param<double>("sf2", sf2, sf2);
    nh.param<double>("ell", ell, ell);
    nh.param<float>("prior", prior, prior);
    nh.param<float>("var_thresh", var_thresh, var_thresh);
    nh.param<double>("free_thresh", free_thresh, free_thresh);
    nh.param<double>("occupied_thresh", occupied_thresh, occupied_thresh);
    nh.param<double>("resolution", resolution, resolution);
    nh.param<int>("num_class", num_class, num_class);
    nh.param<double>("free_resolution", free_resolution, free_resolution);
    nh.param<double>("ds_resolution", ds_resolution, ds_resolution);
    nh.param<int>("scan_num", scan_num, scan_num);
    nh.param<double>("max_range", max_range, max_range);

    // KITTI
    nh.param<std::string>("dir", dir, dir);
    nh.param<std::string>("left_img_prefix", left_img_prefix, left_img_prefix);
    nh.param<std::string>("depth_img_prefix", depth_img_prefix, depth_img_prefix);
    nh.param<std::string>("label_bin_prefix", label_bin_prefix, label_bin_prefix);
    nh.param<std::string>("camera_pose_file", camera_pose_file, camera_pose_file);
    nh.param<std::string>("evaluation_list_file", evaluation_list_file, evaluation_list_file);
    nh.param<std::string>("reproj_img_prefix", reproj_img_prefix, reproj_img_prefix);
    nh.param<int>("image_width", image_width, image_width);
    nh.param<int>("image_height", image_height, image_height);
    nh.param<float>("focal_x", focal_x, focal_x);
    nh.param<float>("focal_y", focal_y, focal_y);
    nh.param<float>("center_x", center_x, center_x);
    nh.param<float>("center_y", center_y, center_y);
    nh.param<float>("depth_scaling", depth_scaling, depth_scaling);
    nh.param<bool>("reproject", reproject, reproject);
    nh.param<bool>("visualize", visualize, visualize);

    ROS_INFO_STREAM("Parameters:" << std::endl <<
      "block_depth: " << block_depth << std::endl <<
      "sf2: " << sf2 << std::endl <<
      "ell: " << ell << std::endl <<
      "prior: " << prior << std::endl <<
      "var_thresh: " << var_thresh << std::endl <<
      "free_thresh: " << free_thresh << std::endl <<
      "occupied_thresh: " << occupied_thresh << std::endl <<
      "resolution: " << resolution << std::endl <<
      "num_class: " << num_class << std::endl <<
      "free_resolution: " << free_resolution << std::endl <<
      "ds_resolution: " << ds_resolution << std::endl <<
      "scan_sum: " << scan_num << std::endl <<
      "max_range: " << max_range << std::endl <<

      "KITTI:" << std::endl <<
      "dir: " << dir << std::endl <<
      "left_img_prefix: " << left_img_prefix << std::endl <<
      "depth_img_prefix: " << depth_img_prefix << std::endl <<
      "label_bin_prefix: " << label_bin_prefix << std::endl <<
      "camera_pose_file: " << camera_pose_file << std::endl <<
      "evaluation_list_file: " << evaluation_list_file << std::endl <<
      "reproj_img_prefix: " << reproj_img_prefix << std::endl <<
      "image_width: " << image_width << std::endl <<
      "image_height: " << image_height << std::endl <<
      "focal_x: " << focal_x << std::endl <<
      "focal_y: " << focal_y << std::endl <<
      "center_x: " << center_x << std::endl <<
      "center_y: " << center_y << std::endl <<
      "depth_scaling: " << depth_scaling << std::endl <<
      "reproject: " << reproject << std::endl <<
      "visualize" << visualize
      );


    KITTIData kitti_data(image_width, image_height, focal_x, focal_y, center_x, center_y, depth_scaling, num_class);
    std::string camera_pose_name(dir + "/" + camera_pose_file);
    if (!kitti_data.read_camera_poses(camera_pose_name))
      return 0;
    if (reproject) {
      std::string evaluation_list_name(dir + "/" + evaluation_list_file);
      if (!kitti_data.read_evaluation_list(evaluation_list_name))
        return 0;
      std::string reproj_img_folder(dir + "/" + reproj_img_prefix + "/");
      kitti_data.set_up_reprojection(reproj_img_folder);
    }

    ///////// Build Map /////////////////////
    semantic_bki::SemanticBKIOctoMap map(resolution, block_depth, num_class, sf2, ell, prior, var_thresh, free_thresh, occupied_thresh);
    semantic_bki::MarkerArrayPub m_pub(nh, map_topic, 0.1f);
    ros::Time start = ros::Time::now();
    for (int scan_id = 0; scan_id <= scan_num; ++scan_id) {
      semantic_bki::PCLPointCloud cloud;
      semantic_bki::point3f origin;
        
      char scan_id_c[256];
      sprintf(scan_id_c, "%06d", scan_id);
      std::string scan_id_s(scan_id_c);
      std::string depth_img_name(dir + "/" + depth_img_prefix + "/" + scan_id_s + ".png");
      std::string label_bin_name(dir + "/" + label_bin_prefix + "/" + scan_id_s + ".bin");

    	cv::Mat depth_img = cv::imread(depth_img_name, CV_LOAD_IMAGE_ANYDEPTH);
    	kitti_data.read_label_prob_bin(label_bin_name);
      kitti_data.process_depth_img(scan_id, depth_img, cloud, origin, reproject);
      
      map.insert_pointcloud(cloud, origin, resolution, free_resolution, max_range);
      ROS_INFO_STREAM("Scan " << scan_id << " done");
     
      if (reproject)
        kitti_data.reproject_imgs(scan_id, map); 
      
      if (visualize) {
        m_pub.clear_map(resolution);
        for (auto it = map.begin_leaf(); it != map.end_leaf(); ++it) {
          if (it.get_node().get_state() == semantic_bki::State::OCCUPIED) {
            semantic_bki::point3f p = it.get_loc();
            m_pub.insert_point3d_semantics(p.x(), p.y(), p.z(), it.get_size(), it.get_node().get_semantics(), 1);
          }
        }
        m_pub.publish();
      }
    }
    ros::Time end = ros::Time::now();
    ROS_INFO_STREAM("Mapping finished in " << (end - start).toSec() << "s");
        
    ros::spin();
    return 0;
}
