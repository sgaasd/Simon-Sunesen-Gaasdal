#include <ros/ros.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/BumperEvent.h>
#include <b228_miniproject/safety_msg.h>

using namespace std;
ros::Publisher miniproject_pub;
class Safety_CallBack {
    public: 
    
    void CliffCallback(const kobuki_msgs::CliffEvent::ConstPtr& msg){
        bool cliffs = msg->state;
        int sensors = msg->sensor;
 
        b228_miniproject::safety_msg safety_msg;
  
        if (cliffs == 1) {
            safety_msg.side = sensors;
            miniproject_pub.publish(safety_msg);
        }
    }
        
    void BumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
        bool hit = msg->state;
        int bump = msg->bumper;
      
        b228_miniproject::safety_msg safety_msg;
        
        if (hit == 1) {
            safety_msg.side = bump;
            miniproject_pub.publish(safety_msg);
        }
    }    
    private:
};

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "Safety");
    
    ros::NodeHandle n; 
        
    Safety_CallBack safetyClass;

    ros::Subscriber Cliff_sub = n.subscribe("/mobile_base/events/cliff",
     1, &Safety_CallBack::CliffCallback, &safetyClass);   
    ros::Subscriber Bumper_sub = n.subscribe("/mobile_base/events/bumper",
     1, &Safety_CallBack::BumperCallback, &safetyClass);

        ros::Publisher miniproject_pub = n.advertise<b228_miniproject::safety_msg>("/miniproject/safety", 1);
    ros::spin();
    return 0;
}
