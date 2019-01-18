#include <string>
#include <stdio.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <termio.h>
#include <string>
#include <stdio.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <termio.h>
#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.hpp>
#include <iostream>

// kbhit() & getch() : Not implemented on linux.
int kbhit();
int getch();  

using namespace std;
using namespace cv;

geometry_msgs::Twist cmd;
double vx=0, vy=0, vz=0, vr=0; 
double linear_vel=1;
double angular_vel=1; 
int key=0;

const Point cameraCenter = Point(294, 64);  // center point of roi(roi_detected)
Point laneCenter;

// 영상을 받아오는 subscriber
ros::Subscriber sub_image;

Point getLaneCenter(Mat roi_detected) {
    Point lc, temp;
    int sum_left=0, avg_left=0, sum_right=0, avg_right=0;
    int tempValue, whiteNum = 0;

    // center coord of left lane
    for(int i = 0; i < 294; i++){
        temp = Point(i, 64);
        tempValue = roi_detected.at<uchar>(temp.y, temp.x);

        if(tempValue == 255){     // if tempValue is white
            sum_left += i;
            whiteNum++;
        }
    }

    if(whiteNum == 0){
        // exception
        return cameraCenter;
    } else {
        avg_left = sum_left / whiteNum;
    }

    whiteNum = 0;
    // center coord of right lane
    for(int j = 294; j < 589; j++){
        temp = Point(j, 64);
        tempValue = roi_detected.at<uchar>(temp.y, temp.x);

        if(tempValue == 255){
            sum_right += j;
            whiteNum++;
        }
    }

    if(whiteNum == 0){
        // exception
        return cameraCenter;
    } else {
        avg_right = sum_right / whiteNum;
    }

    lc = Point((avg_left + avg_right)/2, 64);

    return lc;
}

void getDirection(Point cameraCenter, Point laneCenter){
    int camera_x = cameraCenter.x;
    int lane_x = laneCenter.x;

    int diff = camera_x - lane_x;

    // turn left
    if(diff > 0){       
        if(diff < 10)   // diff is ignorable - just go straight
            vx=0.05, vy=0, vz=0, vr=0;
        else
            vx=0.05, vy=0, vz=0, vr=0.2;
    }
    // turn right
    else if(diff < 0){
            vx=0.01, vy=0, vz=0, vr=-0.2;
    }
    // go straight
    else
        vx = 0.05, vy=0, vz=0, vr=0;
}

void drawCross(Mat mat,Point point){
    line(mat, Point(point.x-8, 64), Point(point.x+8, 64), Scalar(255, 0, 0), 2, 8, 0);
    line(mat, Point(294, point.y-8), Point(294, point.y+8),  Scalar(255, 0, 0), 2, 8, 0);
}

void getImage(const sensor_msgs::CompressedImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    Mat src;                // 터틀봇에서 받은 이미지가 있을 시 저장
    Mat grayConverted;
    Mat roi;                // roi matrix
    Mat roi_detected;       // lane detected
    
    // 터틀봇에서 받은 이미지가 있을 시 저장
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e){ 
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return; 
    }

    // openCV에서 사용 가능하도록 Mat 형태로 변환
    src = cv_ptr->image;
    
    // covert GrayScale
    cvtColor(src, grayConverted, CV_BGR2GRAY);  
    
    // ROI setting - frontside
    Rect rect(50, 350, 589, 129);
    roi = grayConverted(rect);
    
    // threshold setting
    inRange(roi, Scalar(210), Scalar(255), roi_detected);
    laneCenter = getLaneCenter(roi_detected);
    cout << "laneCenter coord : " << laneCenter.x << ", " << laneCenter.y << endl;

    // draw Centers' on roi
    drawCross(roi, cameraCenter);
    line(roi, laneCenter, laneCenter, Scalar(255, 0, 0), 3, 8, 0);

    // show the roi'
    imshow("ROI", roi);
    imshow("ROI - White Line Only", roi_detected);

    // 일정한 간격(frame)으로 영상을 보여주도록 설정
    waitKey(10);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtlebot3_opencv");
    ros::NodeHandle nh; // Subscribe에서의 이벤트가 있을시 getImage가 작동하도록 설정
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    sub_image = nh.subscribe("/usb_cam/image_raw/compressed", 100, getImage);

    printf("\rESC to quit"); 
    ros::Rate loop_rate(10);

    // ros가 진행되는 동안 계속 돌아가도록 반복
    while(ros::ok()){
        if(kbhit()){
            key=getch();
        }

        if(key == 27){  // ESC
            vx=0, vy=0, vz=0, vr=0;
            cmd.linear.x=linear_vel*vx, cmd.linear.y=linear_vel*vy, cmd.linear.z=linear_vel*vz;
            cmd.angular.x=0, cmd.angular.y=0, cmd.angular.z=angular_vel*vr;
            // ROS_INFO("%f %f %f %f", vx, vy, vz, vr);
            pub.publish(cmd);
            break;
        }

        ros::spinOnce();

        // get the derection and move
        getDirection(cameraCenter, laneCenter);
        cmd.linear.x=linear_vel*vx, cmd.linear.y=linear_vel*vy, cmd.linear.z=linear_vel*vz;
        cmd.angular.x=0, cmd.angular.y=0, cmd.angular.z=angular_vel*vr;  
        pub.publish(cmd);

        loop_rate.sleep();
    }
  
    return 0;
}

// kbhit() & getch() : Not implemented on linux.
int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    
    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}
    
int getch(){
    int ch;
    struct termios buf, save;
    tcgetattr(0,&save);
    buf = save;
    buf.c_lflag &= ~(ICANON|ECHO); buf.c_cc[VMIN] = 1;
    buf.c_cc[VTIME] = 0;
    tcsetattr(0, TCSAFLUSH, &buf);
    ch = getchar();
    tcsetattr(0, TCSAFLUSH, &save);
    
    return ch;
}

/******************************************************************************************
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

// 영상을 받아오는 subscriber
ros::Subscriber sub_image;

int frame_num = 1;

void getImage(const sensor_msgs::CompressedImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr; Mat src; // 터틀봇에서 받은 이미지가 있을 시 저장
    Mat grayConverted;     // grayConverted matrix
    Mat RedConverted;

    // 터틀봇에서 받은 이미지가 있을 시 저장
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e){ 
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return; 
    }

    // openCV에서 사용 가능하도록 Mat 형태로 변환
    src = cv_ptr->image;

    putText(src, format("currunt frame number : %d", frame_num), Point(50,80), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,200,200), 4);
    // 프레임 프린팅 이미지 출력
    imshow("src (frmame number)", src);

    // covert GrayScale
    cvtColor(src, grayConverted, CV_BGR2GRAY);  
    imshow("grayCoverted", grayConverted);

    // ROI setting
    Rect rect(10, 10, 100, 100);
    Mat roi = src(rect);

    // threshold setting
    cvtColor(src, roi, CV_BGR2HSV);
    inRange(roi, Scalar(0,100,100), Scalar(10, 255, 255), RedConverted);

    imshow("ROI set and RedConverted", RedConverted);

    // increase frame_num
    frame_num++;
    
    // 일정한 간격(frame)으로 영상을 보여주도록 설정
    waitKey(10);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtlebot3_opencv");
    ros::NodeHandle nh; // Subscribe에서의 이벤트가 있을시 getImage가 작동하도록 설정
    sub_image = nh.subscribe("/usb_cam/image_raw/compressed", 100, getImage);

    // ros가 진행되는 동안 계속 돌아가도록 반복
    while(ros::ok()){
        ros::spinOnce();
    }
  
    return 0;
}
******************************************************************************************/