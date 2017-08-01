#include <cnt723/cnt723.h>

#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float64.h>

#include <cmath>
#include <string>

int
main(int argc, char* argv[]) {
    ros::init(argc, argv, "cnt723_node");
    ros::NodeHandle nh{"~"};

    // Get parameters
    std::string port;
    int baudrate;
    std::string topic_name_prefix;
    std::string topic_name_count;
    std::string topic_name_speed;
    float frequency;
    float wheel_diameter;
    float counts_per_rotation;

    nh.getParam("port", port);
    nh.param("baudrate", baudrate, 9600);
    nh.param("topic_name_prefix", topic_name_prefix, std::string{"encoder"});
    nh.param("topic_name_count", topic_name_count, std::string{"count"});
    nh.param("topic_name_speed", topic_name_speed, std::string{"speed"});
    nh.param("frequency", frequency, static_cast<float>(1000));
    nh.getParam("wheel_diameter", wheel_diameter);
    nh.getParam("counts_per_rotation", counts_per_rotation);

    if (topic_name_prefix != "") {
        topic_name_count = topic_name_prefix + "/" + topic_name_count;
        topic_name_speed = topic_name_prefix + "/" + topic_name_speed;
    }

    Cnt723 cnt723{port, static_cast<unsigned>(baudrate)};
    try {
        cnt723.connect();
        ROS_INFO("Connected to device:");
        ROS_INFO_STREAM("  PORT: " << port);
        ROS_INFO_STREAM("  BAUD: " << baudrate);
    }
    catch (const serial::PortNotOpenedException& e) {
        ROS_ERROR_STREAM(e.what());
        return 1;
    }
    catch (const serial::IOException& e) {
        ROS_ERROR_STREAM(e.what());
        return 1;
    }

    ros::Publisher count_pub = nh.advertise<std_msgs::UInt32>(topic_name_count, 1);
    ros::Publisher speed_pub = nh.advertise<std_msgs::Float64>(topic_name_speed, 1);
    ros::Rate r{frequency};

    std_msgs::UInt32 msg_count;
    std_msgs::Float64 msg_speed;
    unsigned long long int last_count;
    ros::Time last_count_time;
    bool not_the_first = false;
    while (ros::ok()) {
        auto current_count = cnt723.get_count();
        auto current_time = ros::Time::now();

        if (not_the_first) {
            auto dt = (current_time - last_count_time).toSec();
            // Note: When in reverse, becomes negative
            auto d_count = static_cast<long long int>(current_count - last_count);
            auto speed = (d_count / counts_per_rotation) * (wheel_diameter * M_PI) / dt;

            msg_count.data = current_count;
            count_pub.publish(msg_count);
            msg_speed.data = speed;
            speed_pub.publish(msg_speed);
        }

        last_count = current_count;
        last_count_time = current_time;
        not_the_first = true;

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
