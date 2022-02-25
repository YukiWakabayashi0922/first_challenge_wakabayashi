#include "first_challenge_Wakabayashi/first_challenge_Wakabayashi.h"

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

void FirstChallenge::run();
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0.1;
    cmd_vel_.cntl.angular.z = 0.0;

    pub_cmd_vel_.publish(cmd_vel_);
}

void FirstChallenge::turn();
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0.0;
    cmd_vel_.cntl.angular.z = M_PI/50;

    pub_cmd_vel_.publish(cmd_vel_);
}

void FirstChallenge::show_odom()
{
    std::cout << "odom" << ": x:" << odometry_.pose.pose.position.x << " y:" << odometry_.pose.pose.position.y << " z:" << odometry_.pose.pose.position.z << std::endl;
}

void FirstChallenge::show_scan
{
    float range_min = 1e6;
    for(int i=0; i<laser_.ranges.size(); i++) {
        if(laser_.ranges[i] < range_min) {
            range_min = laser_.ranges[i];
        }
    }
    std::cout << "scan: min:" << range_min << std::endl;
}

void FirstChallenge::process()
{
    ros::Rate loop_late(hz_);
    int count = 0;

    while(ros::ok())
    {
        run();
        show_odom();
//        show_scan();

        ros::spinOnce();
        loop_late.sleep();
        count += 1;

        if(count > 50) {
            for(int i=0; i<50; i++) {
                turn();

                ros::spinOnce();
                loop_late.sleep();
            }
            break;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "first_challenge_Wakabayashi");
    FirstChallenge first_challenge;
    first_challenge.process();
    ros::spin();
    return 0;
}
