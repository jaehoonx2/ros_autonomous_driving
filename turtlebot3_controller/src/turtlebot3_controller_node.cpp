#include <string>
#include <stdio.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <termio.h>
#include <unistd.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// kbhit() & getch() : Not implemented on linux.
int kbhit();
int getch();

int main(int argc, char** argv){
    ros::init(argc, argv, "turtlebot3_controller"); 
    ros::NodeHandle nh; 
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    
    geometry_msgs::Twist cmd;
    double vx=0, vy=0, vz=0, vr=0; 
    double linear_vel=1;
    double angular_vel=1; 
    int key=0;
    
    ros::Rate loop_rate(10); 
    
    printf("\nControl Your Turtlebot3!\n"); 
    printf("---------------------------\n"); 
    printf("Moving around:\n");
    printf(" q w e\n");
    printf(" a s d\n");
    printf(" z x c\n\n");
    
    while(true){ 
        
        printf("\rESC to quit"); 
        
        if(kbhit()){
            key=getch();
        } else { 
                key=0; vx=0, vy=0, vz=0, vr=0; 
        }
        
        switch(key){
            case 27: return 0; break; 
            case 'q': vx=1, vy=0, vz=0, vr=1; break; 
            case 'w': vx=1, vy=0, vz=0, vr=0; break;
            case 'e': vx=1, vy=0, vz=0, vr=-1; break; 
            case 'a': vx=0, vy=0, vz=0, vr=1; break; 
            case 'd': vx=0, vy=0, vz=0, vr=-1; break; 
            case 'z': vx=-1, vy=0, vz=0, vr=-1; break; 
            case 'x': vx=-1, vy=0, vz=0, vr=0; break; 
            case 'c': vx=-1, vy=0, vz=0, vr=1; break;
        }

    cmd.linear.x=linear_vel*vx, cmd.linear.y=linear_vel*vy, cmd.linear.z=linear_vel*vz;
    cmd.angular.x=0, cmd.angular.y=0, cmd.angular.z=angular_vel*vr;
    // ROS_INFO("%f %f %f %f", vx, vy, vz, vr);
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