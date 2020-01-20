        // *** Author: Group B228    *** //
        // *** Project: Mini project *** //
        // *** Node: Drive node      *** //
        // *** Created: 31-12-2019   *** //

#include <ros/ros.h> // ros specific include file
#include <geometry_msgs/Twist.h> // A message that contains path to the angular and linear velocity
#include <b228_miniproject/safety_msg.h> // Own message creation for communication
#include <stdlib.h> // Include funtionallities for srand and rand
#include <time.h> // Include the time for srand

// A function without a return value(void), called drive
// When a essage is revieved on the topic "b228_miniproject" inside this topic a msg file with the name "safety_msg"
// will have the information published on the topic.
void drive(const b228_miniproject::safety_msg::ConstPtr& msg);

// All these variables are declared and initialized
// PreCount and count is used to determine if the bumper of cliff ha been activated
int PreCount = 0;
int Count = 0;
//Sidehit is used to store the value from the message to be used in this code
int Sidehit = 0;

// a ros specific publisher is declared 
ros::Publisher cmd_vel_pub;

// A function with the type Geometry_msgs::Twist is made.
// Parameters inputtet in this function is 2 float variables 
geometry_msgs::Twist DriveMsg(float x, float z){
    geometry_msgs::Twist cmd_vel_message;
    cmd_vel_message.angular.z = z;
    cmd_vel_message.linear.x = x;
    // Here it returns the value in x and z to the geometry_msgs::Twist
    return cmd_vel_message;
}

/*A function for calling when a safety action should be executet is declared.*/
void safetyAction(int SideHit){
    // Seeds the generator with with the given number
    // In this case time is the number used to calculate the random
    srand (time(NULL));
    // Turns is decladed and initialized with a random number between 10 and 59
    int Turns = rand() % 50 + 10;

    // ensures that the loop runs at 20 Hz
    ros::Rate loop_rate(20);

    // A switch expression the created with an parameter "sidehit" as input
    switch (SideHit){
        /*If a sensor on the left side is activated the robot 
        will first drive backwards and turn right*/
        case 0:
        // (initialization; condition; increment)
        for(int i=0; i<=30; i++){
            // Publish a message on the Twist topic with x and z 
            cmd_vel_pub.publish(DriveMsg(-0.2, 0.0));
            // sleeps for the remaning time in a loop (if there is remaining time)
            loop_rate.sleep();
        }
        for(int i=0; i<=Turns; i++){ 
            cmd_vel_pub.publish(DriveMsg(0.0, -(0.75)));
            loop_rate.sleep();
        }
        break;
        /*If a sensor on the middle is activated the robot 
        will drive backwards and turn left*/
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
        /*If a sensor on the right side is activated the robot 
        will drive backwards and turn left*/
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

// ARGument Count  - numbers of arguments from comand line input
// ARGument Vector - the string that has been inputtet through command line input
int main(int argc, char *argv[])
{
    /*The ROS node is initialized*/
    ros::init(argc, argv, "Drive");
    //  Node handle declaration for communication with ROS system
    ros::NodeHandle n;

    /*A ROS subscriber, and the publisher is assigned.*/

    //the subcriber  called miniproject_sub is declared 
    // it subcribes to the topic miniproject/safety, and passes the value onto drive, it has BUF_SIZE 1
    ros::Subscriber miniproject_sub = n.subscribe("/miniproject/safety", 1, drive);

    //the publisher is assiged a topic to publish to
    // it publishes to the teleop topic inside the twist message. BUF_SIZE 1
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    
    /*As long as ROS runs, check if the drive function has been called*/
    while (ros::ok){
        // This will run if precount is not equal to count
        if (PreCount != Count){
            /*If the drive function has been called, call safetyAction with safety as parameter, 
            and note this has been done, by Precount = Count*/
            safetyAction(Sidehit);
            PreCount = Count;
        } else {
            /*If the drive function has not been called, drive straight*/
            cmd_vel_pub.publish(DriveMsg(0.2, 0.0));
        }
        /*Run the while loop once. This ensures that the node also checks the topics it subscribes to*/
        ros::spinOnce();
    }  
    /*Spin the node*/
    ros::spin();
    return 0;
}

/*The drive function is assiged*/
void drive(const b228_miniproject::safety_msg::ConstPtr& msg){
    /*Set the global variable Sidehit as the side that was activated*/
    Sidehit = msg->side;
    /*Add one to count*/
    Count++;
}
