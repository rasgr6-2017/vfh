#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32MultiArray.h"


class VFHfollowing
{
public:

    VFHfollowing();

    void run();

    void getScan(sensor_msgs::LaserScan);

    void thresholding();

    void computeheading();

private:
    double raw_scan[360];
    int scan_thr[360];
    double scan_mod[360];

    int countdown;

    double threshold;

    double para_a;
    double para_b;

    double target_heading;

    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Publisher scan_pub;
};

VFHfollowing::VFHfollowing()
{
    for(int i = 0; i < 360; i++)
    {
        raw_scan[i] = 1;
        scan_thr[i] = 0;
    }

    countdown = 1;
    threshold = 0.8;

    para_a = 1.0;
    para_b = 0.45;

    scan_sub = nh.subscribe("/scan", 1, &VFHfollowing::getScan, this);
    scan_pub = nh.advertise<std_msgs::Float32MultiArray>("histogram", 1);

}

void VFHfollowing::computeheading()
{
    // here you detect candidate valleys
    // pick the one closest to the target direction
    // do that wide and narrow check
    // finally decide the theta for robot's heading
}


void VFHfollowing::getScan(sensor_msgs::LaserScan scan_msg)
{
    if(--countdown == 0)
    {
      countdown = 10;
      for(int i = 0; i < 360; i++)
      {
          if(isinf(scan_msg.ranges[i]))
          {
             raw_scan[i] = 2.0;
          }
          else{
              raw_scan[i] = scan_msg.ranges[i];
              if (raw_scan[i] > 2.0)
              {
                  raw_scan[i] = 2.0;
              }
          }
      }
    }
}

void VFHfollowing::thresholding ()
{
    double temp;
    for(int i = 0; i < 360; i++)
    {
        temp = para_a -  para_b * raw_scan[i];
        scan_mod[i] = temp;
        if (temp >= threshold)
        {
            scan_thr[i] = 1;
        }
        else{
            scan_thr[i] = 0;
        }
    }
}

void VFHfollowing::run()
{
    ros::Rate r(5);

    while (ros::ok()) {

        if(countdown == 10)
        {
            thresholding ();

            std_msgs::Float32MultiArray msg;
            for(int i = 0; i < 360; i++)
            {
                msg.data.push_back((double)scan_mod[i]);
            }
            scan_pub.publish(msg);
        }

        ros::spinOnce();
        r.sleep();
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vfh_following");

  VFHfollowing follower;

  follower.run();

  return 0;
}

