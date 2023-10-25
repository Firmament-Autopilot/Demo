#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>

#define FLIGHT_ALTITUDE 1.0f
#define RATE            20   // loop rate hz
#define RADIUS          5.0 // radius of figure 8 in meters
#define CYCLE_S         30   // time to complete one figure 8 cycle in seconds
#define STEPS           (CYCLE_S * RATE)

int main(int argc, char** argv)
{
    ros::init(argc, argv, "offboard_figure_eight_demo");
    ros::NodeHandle nh;
    
    ROS_INFO("Offboard figure eight demo!");

    const float PI = 3.14159265359;
    const float dt = 1.0f / RATE;
    const float dadt = (2.0f * PI) / CYCLE_S;
    const float r = RADIUS;
    uint32_t i = 0;

    ros::Rate rate(RATE);

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    mavros_msgs::SetMode set_mode;

    set_mode.request.custom_mode = "OFFBOARD";
    // Set the mode to OFFBOARD using MAVROS service
    if (set_mode_client.call(set_mode) && set_mode.response.mode_sent) {
        ROS_INFO("Offboard mode enabled");
    }

    set_mode.request.custom_mode = "AUTO.TAKEOFF";
    // Set the mode to TAKEOFF using MAVROS service
    if (set_mode_client.call(set_mode) && set_mode.response.mode_sent) {
        ROS_INFO("Takeoff enabled");
    }

    // delay 10s to wait takeoff finish
    ros::Duration(10).sleep();

    mavros_msgs::PositionTarget setpoint_msg;
    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    while (ros::ok()) {
        // Your code here
        float a = (-PI / 2.0f) + i * (2.0f * PI / STEPS);
        float c = cos(a);
        float c2a = cos(2.0 * a);
        float c4a = cos(4.0 * a);
        float c2am3 = c2a - 3.0;
        float c2am3_cubed = c2am3 * c2am3 * c2am3;
        float s = sin(a);
        float cc = c * c;
        float ss = s * s;
        float sspo = (s * s) + 1.0;
        float ssmo = (s * s) - 1.0;
        float sspos = sspo * sspo;

        i = (i + 1) % STEPS;

        setpoint_msg.header.stamp = ros::Time::now();
        setpoint_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        setpoint_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE | mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFZ;

        // Set position
        setpoint_msg.position.x = -(r * c * s) / sspo;
        setpoint_msg.position.y = (r * c) / sspo;
        setpoint_msg.position.z = FLIGHT_ALTITUDE;
        
        // Set velocity
        setpoint_msg.velocity.x = dadt * r * (ss * ss + ss + (ssmo * cc)) / sspos;
        setpoint_msg.velocity.y = -dadt * r * s * (ss + 2.0 * cc + 1.0) / sspos;
        
        // Set acceleration
        setpoint_msg.acceleration_or_force.x = -dadt * dadt * 8.0 * r * s * c * ((3.0 * c2a) + 7.0) / c2am3_cubed;
        setpoint_msg.acceleration_or_force.y = dadt * dadt * r * c * ((44.0 * c2a) + c4a - 21.0) / c2am3_cubed;
        
        // Set yaw (in radians)
        setpoint_msg.yaw = atan2(setpoint_msg.velocity.y, setpoint_msg.velocity.x);

        setpoint_pub.publish(setpoint_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

