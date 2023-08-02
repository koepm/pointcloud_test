#include "pcl2marker/pcl2marker.hpp"

pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);

PointCloudROI::PointCloudROI()
: Node("PointCloud_ROI")
{

    pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "velodyne_points", 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {lidar_callback(msg);});

    pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "velodyne_points_test",10);

    park_area1_pub_= this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker1", 10);

    park_area2_pub_= this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker2", 10);

    
}

void PointCloudROI::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

    // ROS2 메세지를 PCL 포인트 클라우드로 변환
    pcl::fromROSMsg(*msg, *point_cloud);

    
    // crop 영역설정
    pcl::CropBox<pcl::PointXYZI> cropped_pcl;
    cropped_pcl.setInputCloud(point_cloud);
    cropped_pcl.setMin(Eigen::Vector4f(-0, -7, -0.3, 0)); // x, y, z, min (m)
    cropped_pcl.setMax(Eigen::Vector4f(15, 0, 1.0, 0));    // (-0,20) (-15,15) (-0.5,1.0)...............8/4 data
    //cropped_pcl.setMax(Eigen::Vector4f(20.0, 5.0, 6.0, 1.0));    // 상자의 최대 좌표 설정
    //cropped_pcl.setMin(Eigen::Vector4f(0.0, -5.0, -6.0, 1.0));  // 상자의 최소 좌표 설정    
    cropped_pcl.filter(*point_cloud);

    sensor_msgs::msg::PointCloud2 cropped_point_cloud;
    pcl::toROSMsg(*point_cloud, cropped_point_cloud);
    pcl_pub_ -> publish(cropped_point_cloud);


  

    float P_first_x = 1.3, P_first_y = -1.0;   //왼쪽 아래
    float P_second_x = 4.1, P_second_y = -7.0; //오른쪽 아래
    float P_third_x = 5.8, P_third_y = -6.2;   //오른쪽 위
    float P_fourth_x = 3.6, P_fourth_y = -1.0; //왼쪽 위

    float P_second_x1 = (P_first_x + 2.0*P_second_x)/3.0;
    float P_second_y1 = (P_first_y + 2.0*P_second_y)/3.0;
    float P_third_x1 = (P_fourth_x + 2.0*P_third_x)/3.0;
    float P_third_y1 = (P_fourth_y + 2.0*P_third_y)/3.0;

    float Pp_first_x = 3.8, Pp_first_y = -1.0;   //왼쪽 아래 (0.5,-1)
    float Pp_second_x = 6.6, Pp_second_y = -7.0; //오른쪽 아래 (3.3,-7)
    // float Pp_third_x = 6.6, Pp_third_y = -6.2;   //오른쪽 위 (2.3, -6.2)
    // float Pp_fourth_x = 4.2, Pp_fourth_y = -1.0; //왼쪽 위 (2.8,-1)
    float Pp_third_x = 7.8, Pp_third_y = -6.2;   //오른쪽 위 (2.3, -6.2)
    float Pp_fourth_x = 5.4, Pp_fourth_y = -1.0; //왼쪽 위 (2.8,-1)


    visualization_msgs::msg::Marker first_parking_area;
    first_parking_area.header.frame_id = "velodyne";  // 프레임 ID를 적절한 값으로 변경해야 합니다.
    first_parking_area.header.stamp = rclcpp::Clock().now();  // 현재 시간으로 설정하거나 적절한 타임스탬프로 설정해야 합니다.
    first_parking_area.ns = "parking_area1";
    first_parking_area.id = 0;
    first_parking_area.action = visualization_msgs::msg::Marker::ADD;
    first_parking_area.type = visualization_msgs::msg::Marker::LINE_STRIP;
    first_parking_area.pose.orientation.w = 1.0;

    // 포인트의 크기와 색상 설정 (선택적)
    first_parking_area.scale.x = 0.1;  // 포인트의 두께
    first_parking_area.color.r = 1.0;  // 빨간색
    first_parking_area.color.a = 1.0;  // 완전히 불투명한 색상

    // 점들을 추가
    geometry_msgs::msg::Point p1, p2, p3, p4;
    p1.x = P_first_x;
    p1.y = P_first_y;
    p1.z = 0.0;
    p2.x = P_second_x1;
    p2.y = P_second_y1;
    p2.z = 0.0;
    p3.x = P_third_x1;
    p3.y = P_third_y1;
    p3.z = 0.0;
    p4.x = P_fourth_x; 
    p4.y = P_fourth_y; 
    p4.z = 0.0;

    first_parking_area.points.push_back(p1);
    first_parking_area.points.push_back(p2);
    first_parking_area.points.push_back(p3);
    first_parking_area.points.push_back(p4);
    first_parking_area.points.push_back(p1);  // 시작점으로 돌아가기

    // 시각화를 위한 메시지 퍼블리시
    park_area1_pub_->publish(first_parking_area);


    visualization_msgs::msg::Marker second_parking_area;
    second_parking_area.header.frame_id = "velodyne";  // 프레임 ID를 적절한 값으로 변경해야 합니다.
    second_parking_area.header.stamp = rclcpp::Clock().now();  // 현재 시간으로 설정하거나 적절한 타임스탬프로 설정해야 합니다.
    second_parking_area.ns = "parking_area2";
    second_parking_area.id = 0;
    second_parking_area.action = visualization_msgs::msg::Marker::ADD;
    second_parking_area.type = visualization_msgs::msg::Marker::LINE_STRIP;
    second_parking_area.pose.orientation.w = 1.0;

    // 포인트의 크기와 색상 설정 (선택적)
    second_parking_area.scale.x = 0.1;  // 포인트의 두께
    second_parking_area.color.r = 1.0;  // 빨간색
    second_parking_area.color.a = 1.0;  // 완전히 불투명한 색상

    // 점들을 추가
    geometry_msgs::msg::Point p11, p22, p33, p44;
    p11.x = Pp_first_x;
    p11.y = Pp_first_y;
    p11.z = 0.0;
    p22.x = Pp_second_x;
    p22.y = Pp_second_y;
    p22.z = 0.0;
    p33.x = Pp_third_x;
    p33.y = Pp_third_y;
    p33.z = 0.0;
    p44.x = Pp_fourth_x; 
    p44.y = Pp_fourth_y; 
    p44.z = 0.0;

    second_parking_area.points.push_back(p11);
    second_parking_area.points.push_back(p22);
    second_parking_area.points.push_back(p33);
    second_parking_area.points.push_back(p44);
    second_parking_area.points.push_back(p11);  // 시작점으로 돌아가기

    // 시각화를 위한 메시지 퍼블리시
    park_area2_pub_->publish(second_parking_area);



    // Eigen::Matrix4f trans;
    // trans<< 1,   0,  0, 0.000,  //x축 양수->앞으로
    //         0,   1,  0, -0.070,  //y축 양수->왼쪽으로
    //         0,   0,  1, 0.090,  //z축 양수->위로
    //         0,   0,  0,     1;
    // pcl::transformPointCloud(*point_cloud, *point_cloud, trans);

    
    /*
    // 평행이동
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Matrix4f trans;
    trans<< 1,   0,  0, 0.000,  //x축 양수->앞으로
            0,   1,  0, 0.250,  //y축 양수->왼쪽으로
            0,   0,  1, 0.150,  //z축 양수->위로
            0,   0,  0,     1;
    pcl::transformPointCloud(*point_cloud, *ptr_transformed, trans);
    */
    
    /*
    // 회젼변환
    Eigen::Affine3f rotation_matrix = pcl::getTransformation(0.0f, 0.0f, 0.0f, -90.0f, 0.0f, -90.0f);

    // 포인트 클라우드를 회전 변환
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*ptr_transformed, *transformed_cloud, rotation_matrix);
   

    for (auto& point : ptr_transformed->points)
    {
        //point.x = 0.0f;
        printf("x: %f y: %f, z: %f \n", point.x, point.y, point.z);
    }
    */


    // pcl::PointCloud<pcl::PointXYZI>::Ptr pc_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
    // int size = point_cloud->points.size(); 

    // std::cout<<"point cloud size: "<<size<< std::endl;;


    // for(int i =0; i <size; i++)
    // {
    //     // project get the photo coordinate
    //     pcl::PointXYZ temp;
        
    //     temp.x = point_cloud->points[i].x;
    //     temp.y = point_cloud->points[i].y;
    //     temp.z = point_cloud->points[i].z;

    //     float R_ = sqrt(pow(temp.x,2)+pow(temp.y,2)); // 수평거리
    //     float azimuth_ = atan2(temp.y, temp.z); // 수평방향
    //     float elevation_ = abs(atan2(R_, temp.z));  // 수직방향

    //     if(azimuth_ > max_FOV )
    //     {
    //        // Construct transformMat
    //         cv::Mat Rlc = CameraExtrinsicMat(cv::Rect(0, 0, 3, 3));
    //         cv::Mat Tlc = CameraExtrinsicMat(cv::Rect(3, 0, 1, 3));
    //         cv::Mat concatedMat;
            
    //         cv::hconcat(Rlc, Tlc, concatedMat);
    //         cv::Mat transformMat = CameraMat * concatedMat;

    //         double a_[4] = { temp.x, temp.y, temp.z, 1.0 };
    //         cv::Mat pos(4, 1, CV_64F, a_);

    //         cv::Mat newpos(transformMat * pos);

    //         float x = (float)(newpos.at<double>(0, 0) / newpos.at<double>(2, 0));
    //         float y = (float)(newpos.at<double>(1, 0) / newpos.at<double>(2, 0));

    //         if (x >= 0 && x < 640 && y >= 0 && y < 480)
    //         {
    //             //printf("x : %f y : %f\n", x, y);
    //             cv::circle(cv_ptr_->image, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), -1);
                
    //         }
    //     }
        
    // }

    // sensor_msgs::msg::PointCloud2 cropped_point_cloud;
    // pcl::toROSMsg(*point_cloud, cropped_point_cloud);
    // pcl_pub_ -> publish(cropped_point_cloud);
    

    /*
    sensor_msgs::msg::PointCloud2 pc_transformed;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);

    Eigen::Matrix4f trans;
    trans<< 1,   0,  0, 0.000,  //x축 양수->앞으로
            0,   1,  0, 0.300,  //y축 양수->왼쪽으로
            0,   0,  1, 0.000,  //z축 양수->위로
            0,   0,  0,     1;
    pcl::transformPointCloud(*point_cloud, *ptr_transformed, trans);


    pcl::toROSMsg(*ptr_transformed, pc_transformed);
    pcl_pub_ -> publish(pc_transformed);
    */


    /*
    // Euler 각도를 회전 변환 행렬로 변환
    Eigen::Affine3f rotation_matrix = pcl::getTransformation(0.0f, 0.0f, 0.0f, -90.0f, 0.0f, -90.0f);

    // 포인트 클라우드를 회전 변환
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*point_cloud, *transformed_cloud, rotation_matrix);

    for (auto& point : transformed_cloud->points)
    {
        PointCloudROI::x = point.x; // 포인트 클라우드의 x 좌표
        PointCloudROI::y = point.y; // 포인트 클라우드의 y 좌표
        PointCloudROI::z = point.z;
    }
    sensor_msgs::msg::PointCloud2 transformed_pcl;
    pcl::toROSMsg(*transformed_cloud, transformed_pcl);
    pcl_pub_ -> publish(transformed_pcl);
    */
}



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudROI>());
  rclcpp::shutdown();
  return 0;
}