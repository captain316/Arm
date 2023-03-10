#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <boost/bind.hpp>
#include <math.h>

class pose2Dmsg {
    public:
        double x;
        double y;
        double theta;
        ros::Time cur_time;
        double last_x;
        double last_y;
        double last_theta;
        ros::Time last_time;
        pose2Dmsg();
};

pose2Dmsg:: pose2Dmsg()
{
    x = 0.;
    y = 0.;
    theta = 0.;
    cur_time = ros::Time::now();
    last_x = 0.;
    last_y = 0.;
    last_theta = 0.;
    last_time = ros::Time::now();
}