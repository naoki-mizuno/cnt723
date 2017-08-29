#include <cnt723/cnt723.h>

#include <ros/ros.h>
#include <std_msgs/UInt32.h>

#include <string>

int
main(int argc, char* argv[]) {
    ros::init(argc, argv, "cnt723_node");
    ros::NodeHandle nh{"~"};

    // Get parameters
    std::string port;
    int baudrate;
    std::string topic_name;
    float frequency;

    nh.getParam("port", port);
    nh.param("baudrate", baudrate, 9600);
    nh.param("topic_name", topic_name, std::string{"encoder_count"});
    nh.param("frequency", frequency, static_cast<float>(100));

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

    ros::Publisher count_pub = nh.advertise<std_msgs::UInt32>(topic_name, 1);
    ros::Rate r{frequency};

    std_msgs::UInt32 msg_count;
    while (ros::ok()) {
        msg_count.data = cnt723.get_count();

        count_pub.publish(msg_count);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
