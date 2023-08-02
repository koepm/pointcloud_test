#include "pcl2marker/pcl2marker.hpp"

pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);

PointCloudROI::PointCloudROI()
: Node("PointCloud_ROI")
{

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image1", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {image_callback(msg);});

    pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "velodyne_points", 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {lidar_callback(msg);});

    pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "velodyne_points_ROI",10);
    
    point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/vision/lane_points", 10, [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {point_callback(msg);});

    
}

void PointCloudROI::point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    pixel_x = msg->point.x;
    pixel_y = msg->point.y;
}


void PointCloudROI::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

    // ROS2 메세지를 PCL 포인트 클라우드로 변환
    pcl::fromROSMsg(*msg, *point_cloud);

    
    // crop 영역설정
    pcl::CropBox<pcl::PointXYZI> cropped_pcl;
    cropped_pcl.setInputCloud(point_cloud);
    //cropped_pcl.setMin(Eigen::Vector4f(-0, -7, -0.3, 0)); // x, y, z, min (m)
    //cropped_pcl.setMax(Eigen::Vector4f(15, 0, 1.0, 0));    // (-0,20) (-15,15) (-0.5,1.0)...............8/4 data
    cropped_pcl.setMax(Eigen::Vector4f(20.0, 5.0, 6.0, 1.0));    // 상자의 최대 좌표 설정
    cropped_pcl.setMin(Eigen::Vector4f(0.0, -5.0, -6.0, 1.0));  // 상자의 최소 좌표 설정    
    cropped_pcl.filter(*point_cloud);

    // Eigen::Matrix4f trans;
    // trans<< 1,   0,  0, 0.000,  //x축 양수->앞으로
    //         0,   1,  0, -0.070,  //y축 양수->왼쪽으로
    //         0,   0,  1, 0.090,  //z축 양수->위로
    //         0,   0,  0,     1;
    // pcl::transformPointCloud(*point_cloud, *point_cloud, trans);
}


void PointCloudROI::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv_ptr_ = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat undistorted_image = cv_ptr_->image;
        ///cv::undistort(cv_ptr_->image, undistorted_image, CameraMat, DistCoeff);
        //cv::resize(cv_ptr_->image, cv_ptr_->image, cv::Size(img_width,img_height));


        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
        int size = point_cloud->points.size(); 

        //std::cout<<"point cloud size: "<<size<< std::endl;;


        for(int i =0; i <size; i++)
        {
            // project get the photo coordinate
            pcl::PointXYZI temp;
            
            temp.x = point_cloud->points[i].x;
            temp.y = point_cloud->points[i].y;
            temp.z = point_cloud->points[i].z;
            temp.intensity = point_cloud->points[i].intensity;

            //printf("intensity : %f\n", temp.intensity);

            //float R_ = sqrt(pow(temp.x,2)+pow(temp.y,2)); // 수평거리
            float azimuth_ = abs(atan2(temp.y, temp.z)); // 각도
            //float elevation_ = abs(atan2(R_, temp.z));  // 수직방향

            //if(azimuth_ > max_FOV )
            //{
                // Construct transformMat
                cv::Mat Rlc = CameraExtrinsicMat(cv::Rect(0, 0, 3, 3));
                cv::Mat Tlc = CameraExtrinsicMat(cv::Rect(3, 0, 1, 3));
                cv::Mat concatedMat;
                
                cv::hconcat(Rlc, Tlc, concatedMat);
                cv::Mat transformMat = CameraMat * concatedMat;

                double a_[4] = { temp.x, temp.y, temp.z, 1.0 };
                cv::Mat pos(4, 1, CV_64F, a_);

                cv::Mat newpos(transformMat * pos);

                double x = (float)(newpos.at<double>(0, 0) / newpos.at<double>(2, 0));
                double y = (float)(newpos.at<double>(1, 0) / newpos.at<double>(2, 0));
                
                double final_x;
                double final_y;

                

                if (x >= 0 && x < 640 && y >= 0 && y < 480 && abs(x-pixel_x) < 5 && abs(y-pixel_y) < 5 )
                {
                    
                    
                    //printf("x : %f y : %f\n", x, y);

                    if (temp.intensity > 30)
                    {
                        cv::circle(undistorted_image, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
                    }
                    else
                    {
                        cv::circle(undistorted_image, cv::Point(x, y), 5, cv::Scalar(255, 0, 0), -1);
                        //cv::circle(undistorted_image, cv::Point(pixel_x, pixel_y), 2, cv::Scalar(255, 255, 0), -1);
                    }
                    
                }
            //}
            
        }

        sensor_msgs::msg::PointCloud2 cropped_point_cloud;
        pcl::toROSMsg(*point_cloud, cropped_point_cloud);
        pcl_pub_ -> publish(cropped_point_cloud);

        cv::imshow("combined_img", undistorted_image);
        cv::waitKey(1);

    }
    catch(cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // 이미지와 포인트 클라우드를 결합하여 combined_image에 저장
    //cv::resize(combined_image, combined_image, cv::Size(1920, 1080));
    //cv::circle(combined_image, cv::Point(z, x), 2, cv::Scalar(0, 0, 255), cv::FILLED);

    //cv::Mat gray_image;
    //cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
    
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudROI>());
  rclcpp::shutdown();
  return 0;
}