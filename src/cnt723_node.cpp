#include <cnt723/cnt723.h>

#include <ros/ros.h>
#include <coms_msgs/ComsEncoder.h>

#include <string>

int
main(int argc, char* argv[]) {
    ros::init(argc, argv, "cnt723_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p{"~"};

    // Get parameters
    std::string port;
    int baudrate;
    float frequency;

    nh_p.getParam("port", port);
    nh_p.param("baudrate", baudrate, 9600);
    nh_p.param("frequency", frequency, static_cast<float>(100));

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

    ros::Publisher count_pub = nh.advertise<coms_msgs::ComsEncoder>("cnt723/count", 1);
    ros::Rate r{frequency};

    unsigned seq = 0;
    coms_msgs::ComsEncoder msg;
    while (ros::ok()) {
        msg.header.seq = seq++;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "0";
        msg.count = cnt723.get_count();

        count_pub.publish(msg);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
