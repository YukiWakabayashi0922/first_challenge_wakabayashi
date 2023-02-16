#include "first_challenge_wakabayashi/first_challenge_wakabayashi.h"

FirstChallenge::FirstChallenge():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    sub_odom_ = nh_.subscribe("/roomba/odometry", 100, &FirstChallenge::odometry_callback, this);
    sub_laser_ = nh_.subscribe("/scan", 100, &FirstChallenge::laser_callback, this);
    pub_cmd_vel_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
}

void FirstChallenge::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odometry_ = *msg;
}

void FirstChallenge::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
}

float FirstChallenge::Getyaw()
{
    double roll, pitch, yaw;
    tf::Quaternion quat(odometry_.pose.pose.orientation.x, odometry_.pose.pose.orientation.y, odometry_.pose.pose.orientation.z, odometry_.pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    return float(yaw);
}

float FirstChallenge::Getrange_min()
{
    float range_min = 1e6;
    int size_medium = laser_.ranges.size()/2;
    int size_min = size_medium - 20;
    int size_max = size_medium + 20;
    for(int i=size_min; i<size_max; i++) {
        if(laser_.ranges[i] < range_min) {
            range_min = laser_.ranges[i];
        }
    }
    return float(range_min);
}

void FirstChallenge::run()
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0.2;
    cmd_vel_.cntl.angular.z = 0.0;

    pub_cmd_vel_.publish(cmd_vel_);
}

void FirstChallenge::turn()
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0.0;
    cmd_vel_.cntl.angular.z = M_PI/8;

    pub_cmd_vel_.publish(cmd_vel_);
}

void FirstChallenge::stop()
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0.0;
    cmd_vel_.cntl.angular.z = 0.0;

    pub_cmd_vel_.publish(cmd_vel_);
}

void FirstChallenge::show_odom()
{
    std::cout << "odom" << ": x:" << odometry_.pose.pose.position.x << " y:" << odometry_.pose.pose.position.y << " z:" << odometry_.pose.pose.position.z << std::endl;
}

void FirstChallenge::show_yaw()
{
    std::cout << "yaw:" << Getyaw() << std::endl;
}

void FirstChallenge::show_scan()
{
    std::cout << "scan: min:" << Getrange_min() << std::endl;
}

void FirstChallenge::process()
{
    int count = 0;
    int stage = 1;
    ros::Rate loop_late(hz_);
    while(ros::ok())
    {
        switch(stage) {
            case 1:
                run();
                show_odom();
                ros::spinOnce();
                loop_late.sleep();

                if(odometry_.pose.pose.position.x >= 1.0) {
                    stage = 2;
                }
                break;
            case 2:
                turn();
                show_yaw();
                ros::spinOnce();
                loop_late.sleep();
                count += 1;

                if((Getyaw() < 0.03) && (Getyaw() > -0.03) && (count > 50)) {
                    stage = 3;
                }
                break;
            case 3:
                run();
                show_scan();
                ros::spinOnce();
                loop_late.sleep();

                if(Getrange_min() <= 0.55) {
                    stage = 4;
                }
                break;
            case 4:
                stop();
                show_scan();
                ros::spinOnce();
                loop_late.sleep();
                break;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "first_challenge_wakabayashi");
    FirstChallenge first_challenge;
    first_challenge.process();
    ros::spin();
    return 0;
}
