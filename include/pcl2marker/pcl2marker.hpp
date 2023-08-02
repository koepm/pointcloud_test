#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/passthrough.h"
#include <opencv2/calib3d.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>

//test
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class PointCloudROI : public rclcpp::Node
{
    public:
        PointCloudROI();
    
    private:
        //pub
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mark_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr park_area1_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr park_area2_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr park_area_pub_;

        //sub
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
        rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr test_sub_;

        //func
        void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
        void combined_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void test_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
        std::pair<double, double> findNearestPoint(double pixel_x, double pixel_y, double targetX, double targetY);

        //variable
        int img_height = 480;
        int img_width = 640;
        cv::Mat concatedMat;
        
        // CameraExtrinsicMat
        cv::Mat CameraExtrinsicMat = (cv::Mat_<double>(4, 4) << 
            0.01357418, -0.99990202, -0.00341862,  0.01607133,
            0.0375878 ,  0.00392679, -0.99928561,  0.04019551,
            0.99920113,  0.01343598,  0.03763742, -0.33372581,
            0.         ,  0.         ,  0.         ,  1.        );

        // CameraMat
        cv::Mat CameraMat = (cv::Mat_<double>(3, 3) << 
            496.7298126215774,   0.,   322.3650797270366,
            0.,  496.986275559556,  301.3556675479293, 
            0., 0.,   1. );

        // DistCoeff
        cv::Mat DistCoeff = (cv::Mat_<double>(1, 5) << 
            0.069668722452185, -0.180233541236036, 0.,   0.,   0.);

        cv::Mat transformMat;
        cv_bridge::CvImagePtr cv_ptr_;
        
        

        float max_FOV = CV_PI/4;    // 카메라의 최대 화각(라디안)

        rclcpp::Time lidar_timestamp_;
        rclcpp::Time image_timestamp_;

        double pixel_x;
        double pixel_y;

};


// Function to find the nearest point in the point cloud
std::pair<double, double> PointCloudROI::findNearestPoint(double pixel_x, double pixel_y, double targetX, double targetY) {
    double minDistance = 5;

    double distance = std::sqrt(std::pow(pixel_x - targetX, 2) + std::pow(pixel_y - targetY, 2));
    
    if (distance < minDistance) {
        //printf(" pixel_x : %f pixel_y : %f\n", pixel_x, pixel_y);
        return std::make_pair(pixel_x, pixel_y);
    }

    
}