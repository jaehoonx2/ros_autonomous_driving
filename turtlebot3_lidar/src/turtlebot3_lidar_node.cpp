#include <string>
#include <stdio.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <termio.h>
#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
// kbhit() & getch() : Not implemented on linux.
int kbhit();
int getch();

using namespace std;

geometry_msgs::Twist cmd;
double vx=0, vy=0, vz=0, vr=0; 
double linear_vel=1;
double angular_vel=1; 
int key=0;

void msgCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    std::vector<float> ranges = msg->ranges;
     
    for(int i=70; i<110; i++){  
        if(ranges[i] != 0)
        {
        
            if(ranges[i] < 0.5)
                vx=0, vy=0, vz=0, vr=0; // if there is a sth on the leftside
            else
                vx=0.1, vy=0, vz=0, vr=0; // nothing is on the leftside
        
        } else
            vx=0.1, vy=0, vz=0, vr=0;

        cmd.linear.x=linear_vel*vx, cmd.linear.y=linear_vel*vy, cmd.linear.z=linear_vel*vz;
        cmd.angular.x=0, cmd.angular.y=0, cmd.angular.z=angular_vel*vr;   
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "turtlebot3_lidar");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    ros::Subscriber sub = nh.subscribe("/scan", 100, msgCallback);
    
    printf("\rESC to quit"); 
    ros::Rate loop_rate(10);

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

/************************** original code *************************/
// #include <ros/ros.h>
// #include <sensor_msgs/LaserScan.h>
// #include <geometry_msgs/Twist.h>

// using namespace std;

// void msgCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
//      std::vector<float> ranges = msg->ranges;
     
//     for(int i=0; i<ranges.size(); i++){
//         cout<<ranges[i]<<" ";
//     }

//     cout<<endl;
// }

// int main(int argc, char** argv){
//     ros::init(argc, argv, "turtlebot3_lidar");
//     ros::NodeHandle nh;
//     ros::Rate loop_rate(2);
//     ros::Subscriber sub = nh.subscribe("/scan", 100, msgCallback);
    
//     while(ros::ok()){
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     return 0;
// }