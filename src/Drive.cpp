
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <b228_miniproject/safety_msg.h> 
#include <stdlib.h> 
#include <time.h>

void drive(const b228_miniproject::safety_msg::ConstPtr& msg);

int PreCount = 0;
int Count = 0;
int Sidehit = 0;

ros::Publisher cmd_vel_pub;

geometry_msgs::Twist DriveMsg(float x, float z){
    geometry_msgs::Twist cmd_vel_message;
    cmd_vel_message.angular.z = z;
    cmd_vel_message.linear.x = x;
    return cmd_vel_message;
}

void safetyAction(int SideHit){
    srand (time(NULL));
    int Turns = rand() % 50 + 10;
        
    ros::Rate loop_rate(20);

    switch (SideHit){

        case 0:
        for(int i=0; i<=30; i++){
            cmd_vel_pub.publish(DriveMsg(-0.2, 0.0));
            loop_rate.sleep();
        }
        for(int i=0; i<=Turns; i++){ 
            cmd_vel_pub.publish(DriveMsg(0.0, -(0.75)));
            loop_rate.sleep();
        }
        break;
                    
        case 1:
        for(int i=0; i<=30; i++){ 
            cmd_vel_pub.publish(DriveMsg(-0.2, 0.0));
            loop_rate.sleep();
        }
        for(int i=0; i<=Turns; i++){
            cmd_vel_pub.publish(DriveMsg(0.0, (0.75)));
            loop_rate.sleep();
        }
        break;
                    
        case 2:
        for(int i=0; i<=30; i++){ 
            cmd_vel_pub.publish(DriveMsg(-0.2, 0.0));
            loop_rate.sleep();
        }
        for(int i=0; i<=Turns; i++){
            cmd_vel_pub.publish(DriveMsg(0.0, (0.75)));
            loop_rate.sleep();
        }
        break;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Drive");
        
    ros::NodeHandle n;     
   
    ros::Subscriber miniproject_sub = n.subscribe("/miniproject/safety", 1, drive);
    
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);

    while (ros::ok){          
        if (PreCount != Count){               
            safetyAction(Sidehit);
            PreCount = Count;
        } else {
            cmd_vel_pub.publish(DriveMsg(0.2, 0.0));
        }
        ros::spinOnce();
    }  
    ros::spin();
    return 0;
}

void drive(const b228_miniproject::safety_msg::ConstPtr& msg){
    Sidehit = msg->side;
    Count++;
}
