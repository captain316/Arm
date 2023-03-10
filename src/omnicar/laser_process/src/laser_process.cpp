#include <ros/ros.h>
#include <boost/bind.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
ros::Publisher laser_topic_pub;
ros::Subscriber sub;

void process_callback(const sensor_msgs::LaserScan old_scan)
{
    sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
    int scan_num = old_scan.ranges.size();
    scan->header.frame_id = old_scan.header.frame_id;
    
    scan->header.stamp = ros::Time::now();
    
    double time_error = scan->header.stamp.toSec() - old_scan.header.stamp.toSec();
    std::cout << "time_errorr = " << time_error << std::endl;
    scan->angle_min = old_scan.angle_min;
    scan->angle_max = old_scan.angle_max;
    scan->angle_increment = old_scan.angle_increment;
	
    scan->range_min = old_scan.range_min;
    scan->range_max = old_scan.range_max;
    scan->ranges.reserve(scan_num);
    scan->ranges.assign(old_scan.ranges.begin(), old_scan.ranges.end());
    scan->intensities.reserve(scan_num);
    // scan->intensities.assign(scan_num, std::numeric_limits<float>::infinity());
    scan->intensities.assign(old_scan.intensities.begin(), old_scan.intensities.end());

    for(int i = 1350; i < 1450; ++i) {
        scan->ranges[i] = 0.01;
    }

    for(int i = 555; i < 570; ++i) {
        scan->ranges[i] = 0.01;
    }
    // scan->header.stamp = ros::Time::now();
    
    laser_topic_pub.publish(scan);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserProcess");
    ros::NodeHandle n;
    laser_topic_pub = n.advertise<sensor_msgs::LaserScan>("scan", 100);  //发布laser的话题
    ros::Subscriber sub = n.subscribe("old_scan", 100, process_callback);
    std::cout << "waiting for old_laser!!!!" << std::endl;
    ros::spin();
    return 0;
}


