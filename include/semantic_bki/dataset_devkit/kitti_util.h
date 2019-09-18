#pragma once

#include <fstream>

#include <Eigen/Dense>
#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/common/transforms.h>

#include "point3f.h"

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXf_row;

class KITTIData {
  public:
    KITTIData(int im_width, int im_height,
              float fx, float fy,
              float cx, float cy,
              float depth_scaling, int num_class)
      : im_width_(im_width)
      , im_height_(im_height)
      , fx_(fx)
      , fy_(fy)
      , cx_(cx)
      , cy_(cy)
      , depth_scaling_(depth_scaling)
      , num_class_(num_class) {

        label_prob_frame_.resize(im_width_ * im_height_, num_class_ - 1);  // NOTE: valid label starts from 0
        label_to_color_.resize(num_class_ - 1, 3);
        label_to_color_ << 128,	  0,	 0,  // 0 building
                           128, 128, 128,  // 1 sky
                           128,	 64, 128,  // 2 road
                           128,	128,	 0,  // 3 vegetation
                             0,	  0, 192,  // 4 sidewalk
                            64,	  0, 128,  // 5 car
                            64,	 64,	 0,  // 6 pedestrian
                             0,	128, 192,  // 7 cyclist
                           192,	128, 128,  // 8 signate
                            64,	 64, 128,  // 9 fence
                           192,	192, 128;  // 10 pole

        init_trans_to_ground_ << 1, 0, 0, 0,
                                 0, 0, 1, 0,
                                 0,-1, 0, 1,
                                 0, 0, 0, 1;

      }
    
    ~KITTIData() {}
    
    bool read_camera_poses(const std::string camera_pose_name) {
      if (std::ifstream(camera_pose_name)) {
        std::vector<std::vector<float>> camera_poses_v;
        std::ifstream fPoses;
        fPoses.open(camera_pose_name.c_str());
        int counter = 0;
        while (!fPoses.eof()) {
          std::vector<float> camera_pose_v;
          std::string s;
          std::getline(fPoses, s);
          if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            float t;
            for (int i = 0; i < 12; ++i) {
              ss >> t;
              camera_pose_v.push_back(t);
            }
            camera_poses_v.push_back(camera_pose_v);
            counter++;
          }
        }
        fPoses.close();
        camera_poses_.resize(counter, 12);
        for (int c = 0; c < counter; ++c) {
          for (int i = 0; i < 12; ++i)
            camera_poses_(c, i) = camera_poses_v[c][i];
        }
        return true;
     } else {
       ROS_ERROR_STREAM("Cannot open camera pose file " << camera_pose_name);
       return false;
     }
    }

    bool read_evaluation_list(const std::string evaluation_list_name) {
      if (std::ifstream(evaluation_list_name)) {
        std::vector<int> evaluation_list_v;
        std::ifstream fImgs;
        fImgs.open(evaluation_list_name.c_str());
        int counter = 0;
        while (!fImgs.eof()) {
          std::string s;
          std::getline(fImgs, s);
          if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            int t;
            ss >> t;
            evaluation_list_v.push_back(t);
            counter++;
          }
        }
        fImgs.close();
        evaluation_list_.resize(counter);
        for (int c = 0; c < counter; ++c)
          evaluation_list_(c) = evaluation_list_v[c];
        return true;
      } else {
        ROS_ERROR_STREAM("Cannot open evaluation list file " << evaluation_list_name);
        return false;
      }
    }

    void set_up_reprojection(const std::string reproj_img_folder) {
      reproj_img_folder_ = reproj_img_folder;
      int img_num = evaluation_list_.rows();
      depth_imgs_.resize(img_num);
      reproj_imgs_color_.resize(img_num);
      reproj_imgs_label_.resize(img_num);
      for (int i = 0; i < img_num; ++i) {
        reproj_imgs_color_[i] = cv::Mat(cv::Size(im_width_, im_height_), CV_8UC3, cv::Scalar(200,200,200));
        reproj_imgs_label_[i] = cv::Mat(cv::Size(im_width_, im_height_), CV_8UC1, cv::Scalar(255));
      }
    }

    bool read_label_prob_bin(const std::string label_bin_name) {
      if (std::ifstream(label_bin_name)) {
        std::ifstream fLables(label_bin_name.c_str(), std::ios::in|std::ios::binary);
        if (fLables.is_open()) {
            int mat_byte_size = sizeof(float) * label_prob_frame_.rows() * label_prob_frame_.cols();
            float *mat_field = label_prob_frame_.data();
            fLables.read((char*)mat_field, mat_byte_size);
            fLables.close(); 
        } else {
          ROS_ERROR_STREAM("Cannot open label binary file " << label_bin_name);
          return false;
        }
        return true;
      } else
        return false;    
    }
   
    void process_depth_img(const int scan_id, const cv::Mat& depth_img,
                           pcl::PointCloud<pcl::PointXYZL>& cloud, semantic_bki::point3f& origin, bool reproject) {
      // Save depth images for reprojection
      if (reproject) {
        int reproj_id = check_element_in_vector(scan_id, evaluation_list_);
        if (reproj_id >= 0)
          depth_imgs_[reproj_id] = depth_img.clone();
      }

      Eigen::Matrix4f transform = get_current_pose(scan_id);
      for (int32_t i = 0; i < im_width_ * im_height_; ++i) {
        int ux = i % im_width_;
        int uy = i / im_width_;
        float pix_depth = (float) depth_img.at<uint16_t>(uy, ux);
        pix_depth = pix_depth / depth_scaling_;
        int pix_label;
        label_prob_frame_.row(i).maxCoeff(&pix_label);

        if (pix_label == 1)  // NOTE: don't project sky label
          continue;

        if (pix_depth > 20.0)
          continue;

        if (pix_depth > 0.1) {
          pcl::PointXYZL pt;
          pt.x = (ux - cx_) * (1.0 / fx_) * pix_depth;
          pt.y = (uy - cy_) * (1.0 / fy_) * pix_depth;
          pt.z = pix_depth;
          pt.label = pix_label + 1;  // NOTE: valid label starts from 0
          transform_pt_to_global(transform, pt);
          cloud.points.push_back(pt);
        }
      }
      cloud.width = (uint32_t) cloud.points.size();
      cloud.height = 1;

      // Set sensor origin
      origin.x() = transform(0, 3);
      origin.y() = transform(1, 3);
      origin.z() = transform(2, 3);
    }

    void reproject_imgs(const int current_scan_id, semantic_bki::SemanticBKIOctoMap& map) {
      if (check_element_in_vector(current_scan_id, evaluation_list_) < 0)
        return;
      for (int reproj_id = 0; reproj_id < evaluation_list_.rows(); ++reproj_id) {
        int scan_id = evaluation_list_[reproj_id];
        if (scan_id <= current_scan_id)
          reproject_img(scan_id, reproj_id, map);
      }
    }

    void reproject_img(const int scan_id, const int reproj_id, semantic_bki::SemanticBKIOctoMap& map) {
      Eigen::Matrix4f transform = get_current_pose(scan_id);
      for (int32_t i = 0; i < im_width_ * im_height_; ++i) {
        int ux = i % im_width_;
        int uy = i / im_width_;
        if (ux >= im_width_-1 ||  ux <= 0 || uy >= im_height_-1 ||  uy<= 0)
          continue;
        
        float pix_depth = (float) depth_imgs_[reproj_id].at<uint16_t>(uy, ux);
        pix_depth = pix_depth / depth_scaling_;

        if (pix_depth > 40.0)
          continue;

        if (pix_depth > 0.1) {
          pcl::PointXYZL pt;
          pt.x = (ux - cx_) * (1.0 / fx_) * pix_depth;
          pt.y = (uy - cy_) * (1.0 / fy_) * pix_depth;
          pt.z = pix_depth;
          transform_pt_to_global(transform, pt);
          
          semantic_bki::SemanticOcTreeNode node = map.search(pt.x, pt.y, pt.z);
          if (node.get_state() == semantic_bki::State::OCCUPIED){
            int pix_label = node.get_semantics();
            reproj_imgs_label_[reproj_id].at<uint8_t>(uy, ux) = (uint8_t) pix_label - 1;  // Note: valid label starts from 0 for evaluation
            reproj_imgs_color_[reproj_id].at<cv::Vec3b>(uy, ux)[0] = (uint8_t) label_to_color_(pix_label - 1, 2);
            reproj_imgs_color_[reproj_id].at<cv::Vec3b>(uy, ux)[1] = (uint8_t) label_to_color_(pix_label - 1, 1);
            reproj_imgs_color_[reproj_id].at<cv::Vec3b>(uy, ux)[2] = (uint8_t) label_to_color_(pix_label - 1, 0);
          }
        }
      }

      char scan_id_c[256];
      sprintf(scan_id_c, "%06d", scan_id);
      std::string scan_id_s(scan_id_c);
      std::string reproj_img_color_name(reproj_img_folder_ + "/" + scan_id_s + "_color.png");
      std::string reproj_img_label_name(reproj_img_folder_ + "/" + scan_id_s + "_bw.png");
      cv::imwrite(reproj_img_color_name, reproj_imgs_color_[reproj_id]);
      cv::imwrite(reproj_img_label_name, reproj_imgs_label_[reproj_id]);
    }

  private:
    int im_width_;
    int im_height_;
    float fx_;
    float fy_;
    float cx_;
    float cy_;
    float depth_scaling_;
    int num_class_;

    Eigen::MatrixXi label_to_color_;
    Eigen::Matrix4f init_trans_to_ground_; 
    Eigen::MatrixXf camera_poses_;
    MatrixXf_row label_prob_frame_;

    // Reprojection
    Eigen::VectorXi evaluation_list_;
    std::string reproj_img_folder_;
    std::vector<cv::Mat> depth_imgs_;
    std::vector<cv::Mat> reproj_imgs_color_;
    std::vector<cv::Mat> reproj_imgs_label_;

    int check_element_in_vector(const int element, const Eigen::VectorXi& vec_check) {
      for (int i = 0; i < vec_check.rows(); ++i)
        if (element == vec_check(i) )
          return i;
      return -1;
    }

    Eigen::Matrix4f get_current_pose(const int scan_id) {
      Eigen::VectorXf curr_pose_v = camera_poses_.row(scan_id);
      Eigen::MatrixXf curr_pose = Eigen::Map<MatrixXf_row>(curr_pose_v.data(), 3, 4);
      Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
      transform.block(0, 0, 3, 4) = curr_pose;
      Eigen::Matrix4f new_transform = init_trans_to_ground_ * transform;
      return new_transform;
    }

    void transform_pt_to_global(const Eigen::Matrix4f& transform, pcl::PointXYZL& pt) {
      Eigen::Vector4f global_pt_4 = transform * Eigen::Vector4f(pt.x, pt.y, pt.z, 1);
      Eigen::Vector3f global_pt_3 = global_pt_4.head(3) / global_pt_4(3);
      pt.x = global_pt_3(0);
      pt.y = global_pt_3(1);
      pt.z = global_pt_3(2);
    }
};
