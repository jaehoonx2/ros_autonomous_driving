#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

// 영상을 받아오는 subscriber
ros::Subscriber sub_image;

void getImage(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr; Mat src; // 터틀봇에서 받은 이미지가 있을 시 저장

    try{ 
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e){ 
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return; 
    }

    // openCV에서 사용 가능하도록 Mat 형태로 변환
    src = cv_ptr->image;

    // 받아온 이미지 출력
    imshow("src", src);

    // 일정한 간격(frame)으로 영상을 보여주도록 설정
    waitKey(33);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtlebot3_opencv"); ros::NodeHandle nh; // Subscribe에서의 이벤트가 있을시 getImage가 작동하도록 설정
    sub_image = nh.subscribe("/usb_cam/image_raw", 100, getImage);

    // ros가 진행되는 동안 계속 돌아가도록 반복
    while(ros::ok()){
        ros::spinOnce();
    }
    
    return 0;
}