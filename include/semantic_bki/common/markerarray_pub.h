#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include <cmath>
#include <string>

namespace semantic_bki {

    double interpolate( double val, double y0, double x0, double y1, double x1 ) {
        return (val-x0)*(y1-y0)/(x1-x0) + y0;
    }

    double base( double val ) {
        if ( val <= -0.75 ) return 0;
        else if ( val <= -0.25 ) return interpolate( val, 0.0, -0.75, 1.0, -0.25 );
        else if ( val <= 0.25 ) return 1.0;
        else if ( val <= 0.75 ) return interpolate( val, 1.0, 0.25, 0.0, 0.75 );
        else return 0.0;
    }

    double red( double gray ) {
        return base( gray - 0.5 );
    }
    double green( double gray ) {
        return base( gray );
    }
    double blue( double gray ) {
        return base( gray + 0.5 );
    }
    
    std_msgs::ColorRGBA JetMapColor(float gray) {
      std_msgs::ColorRGBA color;
      color.a = 1.0;

      color.r = red(gray);
      color.g = green(gray);
      color.b = blue(gray);
      return color;
    }

    std_msgs::ColorRGBA SemanticMapColor(int c) {
      std_msgs::ColorRGBA color;
      color.a = 1.0;

      switch (c) {
        case 1:
          color.r = 1;
          color.g = 0;
          color.b = 0;
          break;
        case 2:
          color.r = 70.0/255;
          color.g = 130.0/255;
          color.b = 180.0/255;
          break;
        case 3:
          color.r = 218.0/255;
          color.g = 112.0/255;
          color.b = 214.0/255;
          break;
        default:
          color.r = 1;
          color.g = 1;
          color.b = 1;
          break;
      }

      return color;
    }

    std_msgs::ColorRGBA SemanticKITTISemanticMapColor(int c) {
      std_msgs::ColorRGBA color;
      color.a = 1.0;

      switch (c) {
        case 1:  // car
          color.r = 245.0 / 255;
          color.g = 150.0 / 255;
          color.b = 100.0 / 255;
          break;
        case 2:  // bicycle
          color.r = 245.0 / 255;
          color.g = 230.0 / 255;
          color.b = 100.0 / 255;
          break;
        case 3:  // motorcycle
          color.r = 150.0 / 255;
          color.g = 60.0 / 255;
          color.b = 30.0 / 255;
          break;
        case 4:  // truck
          color.r = 180.0 / 255;
          color.g = 30.0 / 255;
          color.b = 80.0 / 255;
          break;
        case 5:  // other-vehicle
          color.r = 255.0 / 255;
          color.g = 80.0 / 255;
          color.b = 100.0 / 255;
          break;
        case 6:  // person
          color.r = 30.0 / 255;
          color.g = 30.0 / 255;
          color.b = 1;
          break;
        case 7:  // bicyclist
          color.r = 200.0 / 255;
          color.g = 40.0 / 255;
          color.b = 1;
          break;
        case 8:  // motorcyclist
          color.r = 90.0 / 255;
          color.g = 30.0 / 255;
          color.b = 150.0 / 255;
          break;
        case 9:  // road
          color.r = 1;
          color.g = 0;
          color.b = 1;
          break;
        case 10: // parking
          color.r = 1;
          color.g = 150.0 / 255;
          color.b = 1;
          break;
        case 11: // sidewalk
          color.r = 75.0 / 255;
          color.g = 0;
          color.b = 75.0 / 255;
          break;
        case 12: // other-ground
          color.r = 75.0 / 255;
          color.g = 0;
          color.b = 175.0 / 255;
          break;
        case 13: // building
          color.r = 0;
          color.g = 200.0 / 255;
          color.b = 1;
          break;
        case 14: // fence
          color.r = 50.0 / 255;
          color.g = 120.0 / 255;
          color.b = 1;
          break;
        case 15: // vegetation
          color.r = 0;
          color.g = 175.0 / 255;
          color.b = 0;
          break;
        case 16: // trunk
          color.r = 0;
          color.g = 60.0 / 255;
          color.b = 135.0 / 255;
          break;
        case 17: // terrain
          color.r = 80.0 / 255;
          color.g = 240.0 / 255;
          color.b = 150.0 / 255;
          break;
        case 18: // pole
          color.r = 150.0 / 255;
          color.g = 240.0 / 255;
          color.b = 1;
          break;
        case 19: // traffic-sign
          color.r = 0;
          color.g = 0;
          color.b = 1;
          break;
        default:
          color.r = 1;
          color.g = 1;
          color.b = 1;
          break;
      }
      return color;
    }

    std_msgs::ColorRGBA NCLTSemanticMapColor(int c) {
      std_msgs::ColorRGBA color;
      color.a = 1.0;

      switch (c) {
        case 1:  // water
          color.r = 30.0 / 255;
          color.g = 144.0 / 255;
          color.b = 250.0 / 255;
          break;
        case 2:  // road
          color.r = 250.0 / 255;
          color.g = 250.0 / 255;
          color.b = 250.0 / 255;
          break;
        case 3:  // sidewalk
          color.r = 128.0 / 255;
          color.g = 64.0 / 255;
          color.b = 128.0 / 255;
          //color.r = 250.0/255;
          //color.g = 250.0/255;
          //color.b = 250.0/255;
          break;
        case 4:  // terrain
          color.r = 128.0 / 255;
          color.g = 128.0 / 255;
          color.b = 0;
          break;
        case 5:  // building
          color.r = 250.0 / 255;
          color.g = 128.0 / 255;
          color.b = 0;
          break;
        case 6:  // vegetation
          color.r = 107.0 / 255;
          color.g = 142.0/ 255;
          color.b = 35.0 / 255;
          break;
        case 7:  // car
          color.r = 0;
          color.g = 0;
          color.b = 142.0 / 255;
          break;
        case 8:  // person
          color.r = 220.0 / 255;
          color.g = 20.0 / 255;
          color.b = 60.0 / 255;
          //color.r = 250.0 / 255;
          //color.g = 250.0 / 255;
          //color.b = 250.0 / 255;
          break;
        case 9:  // bike
          color.r = 119.0 / 255;
          color.g = 11.0 / 255;
          color.b = 32.0/ 255;
          break;
        case 10:  // pole
          color.r = 192.0 / 255;
          color.g = 192.0 / 255;
          color.b = 192.0 / 255;
          break;
        case 11:  // stair
          color.r = 123.0 / 255;
          color.g = 104.0 / 255;
          color.b = 238.0 / 255;
          break;
        case 12:  // traffic sign
          color.r = 250.0 / 255;
          color.g = 250.0 / 255;
          color.b = 0;
          break;
        case 13:  // sky
          color.r = 135.0 / 255;
          color.g = 206.0 / 255;
          color.b = 235.0 / 255;
          break;
        default:
          color.r = 1;
          color.g = 1;
          color.b = 1;
          break;
      }
      return color;
    }

    std_msgs::ColorRGBA KITTISemanticMapColor(int c) {
      std_msgs::ColorRGBA color;
      color.a = 1.0;
      
      switch (c) {
        case 1:  // building
          color.r = 128.0 / 255;
          color.g = 0;
          color.b = 0;
          break;
        case 2:  // sky
          color.r = 128.0 / 255;
          color.g = 128.0 / 255;
          color.b = 128.0 / 255;
          break;
        case 3:  // road
          color.r = 128.0 / 255;
          color.g = 64.0  / 255;
          color.b = 128.0 / 255;
          break;
        case 4:  // vegetation
          color.r = 128.0 / 255;
          color.g = 128.0 / 255;
          color.b = 0;
          break;
        case 5:  // sidewalk
          color.r = 0;
          color.g = 0;
          color.b = 192.0 / 255;
          break;
        case 6:  // car
          color.r = 64.0 / 255;
          color.g = 0;
          color.b = 128.0 / 255;
          break;
        case 7:  // pedestrain
          color.r = 64.0 / 255;
          color.g = 64.0 / 255;
          color.b = 0;
          break;
        case 8:  // cyclist
          color.r = 0;
          color.g = 128.0 / 255;
          color.b = 192.0 / 255;
          break;
        case 9:  // signate
          color.r = 192.0 / 255;
          color.g = 128.0 / 255;
          color.b = 128.0 / 255;
          break;
        case 10: // fense
          color.r = 64.0  / 255;
          color.g = 64.0  / 255;
          color.b = 128.0 / 255;
          break;
        case 11: // pole
          color.r = 192.0 / 255;
          color.g = 192.0 / 255;
          color.b = 128.0 / 255;
          break;
        default:
          color.r = 1;
          color.g = 1;
          color.b = 1;
          break;
      }

      return color;
    }


    std_msgs::ColorRGBA heightMapColor(double h) {

        std_msgs::ColorRGBA color;
        color.a = 1.0;
        // blend over HSV-values (more colors)

        double s = 1.0;
        double v = 1.0;

        h -= floor(h);
        h *= 6;
        int i;
        double m, n, f;

        i = floor(h);
        f = h - i;
        if (!(i & 1))
            f = 1 - f; // if i is even
        m = v * (1 - s);
        n = v * (1 - s * f);

        switch (i) {
            case 6:
            case 0:
                color.r = v;
                color.g = n;
                color.b = m;
                break;
            case 1:
                color.r = n;
                color.g = v;
                color.b = m;
                break;
            case 2:
                color.r = m;
                color.g = v;
                color.b = n;
                break;
            case 3:
                color.r = m;
                color.g = n;
                color.b = v;
                break;
            case 4:
                color.r = n;
                color.g = m;
                color.b = v;
                break;
            case 5:
                color.r = v;
                color.g = m;
                color.b = n;
                break;
            default:
                color.r = 1;
                color.g = 0.5;
                color.b = 0.5;
                break;
        }

        return color;
    }

    class MarkerArrayPub {
        typedef pcl::PointXYZ PointType;
        typedef pcl::PointCloud<PointType> PointCloud;
    public:
        MarkerArrayPub(ros::NodeHandle nh, std::string topic, float resolution) : nh(nh),
                                                                                  msg(new visualization_msgs::MarkerArray),
                                                                                  topic(topic),
                                                                                  resolution(resolution),
                                                                                  markerarray_frame_id("/map") {
            pub = nh.advertise<visualization_msgs::MarkerArray>(topic, 1, true);

            msg->markers.resize(2);
            for (int i = 0; i < 2; ++i) {
                msg->markers[i].header.frame_id = markerarray_frame_id;
                msg->markers[i].ns = "map";
                msg->markers[i].id = i;
                msg->markers[i].type = visualization_msgs::Marker::CUBE_LIST;
                msg->markers[i].scale.x = resolution * pow(2, i);
                msg->markers[i].scale.y = resolution * pow(2, i);
                msg->markers[i].scale.z = resolution * pow(2, i);
                std_msgs::ColorRGBA color;
                color.r = 0.0;
                color.g = 0.0;
                color.b = 1.0;
                color.a = 1.0;
                msg->markers[i].color = color;
            }
        }

        void insert_point3d(float x, float y, float z, float min_z, float max_z, float size) {
            geometry_msgs::Point center;
            center.x = x;
            center.y = y;
            center.z = z;

            int depth = 0;
            if (size > 0)
                depth = (int) log2(size / 0.1);

            msg->markers[depth].points.push_back(center);
            if (min_z < max_z) {
                double h = (1.0 - std::min(std::max((z - min_z) / (max_z - min_z), 0.0f), 1.0f)) * 0.8;
                msg->markers[depth].colors.push_back(heightMapColor(h));
            }
        }

        void clear_map(float size) {
          int depth = 0;
          if (size > 0)
            depth = (int) log2(size / 0.1);

          msg->markers[depth].points.clear();
          msg->markers[depth].colors.clear();
        }

        void insert_point3d_semantics(float x, float y, float z, float size, int c, int dataset) {
            geometry_msgs::Point center;
            center.x = x;
            center.y = y;
            center.z = z;

            int depth = 0;
            if (size > 0)
                depth = (int) log2(size / 0.1);

            msg->markers[depth].points.push_back(center);
            switch (dataset) {
              case 1:
                msg->markers[depth].colors.push_back(KITTISemanticMapColor(c));
                break;
              case 2:
                msg->markers[depth].colors.push_back(SemanticKITTISemanticMapColor(c));
                break;
              default:
                msg->markers[depth].colors.push_back(SemanticMapColor(c));
            }
        }

        void insert_point3d_variance(float x, float y, float z, float min_v, float max_v, float size, float var) {
            geometry_msgs::Point center;
            center.x = x;
            center.y = y;
            center.z = z;

            int depth = 0;
            if (size > 0)
                    depth = (int) log2(size / 0.1);

            float middle = (max_v + min_v) / 2;
            var = (var - middle) / (middle - min_v);
            //std::cout << var << std::endl; 
            msg->markers[depth].points.push_back(center);
            msg->markers[depth].colors.push_back(JetMapColor(var));

        }

        void insert_point3d(float x, float y, float z, float min_z, float max_z) {
            insert_point3d(x, y, z, min_z, max_z, -1.0f);
        }

        void insert_point3d(float x, float y, float z) {
            insert_point3d(x, y, z, 1.0f, 0.0f, -1.0f);
        }

        void insert_color_point3d(float x, float y, float z, double min_v, double max_v, double v) {
            geometry_msgs::Point center;
            center.x = x;
            center.y = y;
            center.z = z;

            int depth = 0;
            msg->markers[depth].points.push_back(center);

            double h = (1.0 - std::min(std::max((v - min_v) / (max_v - min_v), 0.0), 1.0)) * 0.8;
            msg->markers[depth].colors.push_back(heightMapColor(h));
        }

        void clear() {
            for (int i = 0; i < 10; ++i) {
                msg->markers[i].points.clear();
                msg->markers[i].colors.clear();
            }
        }

        void publish() const {
            msg->markers[0].header.stamp = ros::Time::now();
            pub.publish(*msg);
            ros::spinOnce();
        }

    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        visualization_msgs::MarkerArray::Ptr msg;
        std::string markerarray_frame_id;
        std::string topic;
        float resolution;
    };

}
