// The node is a part of the emergency brake demo for the vehicle interface over CAN, 
// The node reieves the pointcloud data, creates region of interest, and within this region of interest
// if any object passes within a predefined threshold, a command message is sent to the CAN interface pkg
// to forward the message over CAN to the DBW controller which is programmed to actuate the brake actuator with
// the recieved actuation distance.
// The main purpose of the node is just testing the vehicle interface pipeline created. 

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <vector>
#include <raptor_dbw_msgs/BrakeCmd.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

class EmergencyBrakeLidar
{
     public:
        // Declaring publishers and subscribers
        ros::Publisher point_cloud_pub;
        ros::Publisher brake_cmd_pub;
        ros::Subscriber lidar_sub;    
        
        EmergencyBrakeLidar()
        {
            ros::NodeHandle n;
            
            point_cloud_pub = n.advertise<sensor_msgs::PointCloud>("test/point_cloud", 1000);
            brake_cmd_pub = n.advertise<raptor_dbw_msgs::BrakeCmd>("vehicle/brake_cmd",1000);
            lidar_sub = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1000, &EmergencyBrakeLidar::lidar_callback, this);
        }
        
    private:
        // Declaring intial values
        double position = 20;                   //default actuator position %
        bool brake_enable = false;              //default brake_enabling 
        
        double position_engaged = 100;          //engaged postion of the actuator %
        double position_disengaged = 20;        //disengaged position of the actuator %
        double front_distance_intialization = 1000;

        double front_threshold = 7;             //threshhold to activate emergency brake

        double min_x = 0.5;     double max_x = min_x; 
        double min_y = -2.5;    double max_y = 2.0;
        double min_z = -1.5;    double max_z = min_z;
        
        raptor_dbw_msgs::BrakeCmd brake_cmd_msg;  
    
        // Declaring point_cloud messages
        sensor_msgs::PointCloud2 point_cloud2;
        sensor_msgs::PointCloud point_cloud;

        void lidar_callback(sensor_msgs::PointCloud2 msg)
        {
            // converting the recieved Lidar message from point_cloud2 to point_cloud message type
            point_cloud2 = msg;
            sensor_msgs::convertPointCloud2ToPointCloud(point_cloud2, point_cloud);

            sensor_msgs::PointCloud point_cloud_roi;
            
            point_cloud_roi.header = point_cloud.header;
            point_cloud_roi.channels = point_cloud.channels;
            
            double front_min_distance = front_distance_intialization;           //setting the intial threshold high enough every loop

            //the main loop for (1) data ROI filtering and (2) actuation 
            for (int i=0; i<(point_cloud.points.size()); i++){        
                if (point_cloud.points[i].x>min_x){
                    if ((point_cloud.points[i].y<max_y) && (point_cloud.points[i].y>min_y)){
                        if((point_cloud.points[i].z>min_z)){
                            
                            // adding the points from ROI to the new point cloud of ROI
                            point_cloud_roi.points.push_back(point_cloud.points[i]);    
                            
                            // estimating the closest point in the front of the vehicle
                            if (point_cloud.points[i].x < front_min_distance)
                            {front_min_distance = point_cloud.points[i].x;}
                        } 
                    }
                }
            }
            
            point_cloud_pub.publish(point_cloud_roi); 
    
            EmergencyBrakeLidar::actuation(front_min_distance);

            //printing out the min value detected
            std::cout << front_min_distance << std::endl; 
        }

        void actuation(double front_min_distance){
            if (front_min_distance < front_threshold){
                brake_enable = true;
                position = position_engaged;
            }
            else{
                brake_enable = false;
                position = position_disengaged;
            }
            brake_cmd_msg.enable = brake_enable;  
            brake_cmd_msg.pedal_cmd = position; 
            //publishing the new topic
            brake_cmd_pub.publish(brake_cmd_msg);
        }  
};


int main(int argc, char **argv)
{  
    ros::init(argc, argv, "emergency_stop_demo_katech");
    EmergencyBrakeLidar emergency_brake_lidar;
    ros::Rate loop_rate(10);

    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
