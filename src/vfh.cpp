#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <sstream>
#include <math.h>
#include <vector>
#include <utility>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

class VFHfollowing
{
public:

    VFHfollowing();

    void run();

    void getScan(sensor_msgs::LaserScan);

    void getTarget(std_msgs::Float32MultiArray tar_msg);

    // helper function to transform between Euler angles and Quaternion
    void EulerQuaternion(geometry_msgs::PoseStamped odom);

    // get m for each beta (in each angle)
    void computeMag();

    // enlarge for obstacles
    // put m into sectors
    // so that we get the primary histogram, H_p_k
    void enlargeObs();

    // get binary Polar Histogram
    void getBPH();

    // get Masked Polar Histogram
    void getMPH();

    // select all candidate valleys from the MPH
    void findCandidate();

    // use selected angle to compute steering
    void computeSteering();

    // helper function to compute sector difference
    int sectordiff(int sec1, int sec2);

    // helper function to decide the direction relationship of two sectors
    int checkdirection(int sec1, int sec2);

    // helper function to extend a line between two point p1, p2 with a length Len
    std::pair<double, double> extendLine(std::pair<double, double> p1, std::pair<double, double> p2, double Len);

    // helper function to find point on circle
    std::pair<double, double> CirclePoint(std::pair<double, double> cen, double Len, double ang);

    int sector_operator(int sec, int change);

private:
    double raw_scan[360];
    double scan_mag[360];

    std::vector<double> ph;
    std::vector<int> bh;

    double certainty[360];

    int countdown;
    int TARGET_NUMBER;
    int command_number;

    int sector_size;
    double one_degree;

    double threshold;

    double para_a;
    double para_b;

    double robot_radius;
    double safety_distance;

    double range_min;
    double range_max;

    double upper_th;
    double lower_th;

    double m1, m2, m3;

    int prev_sector;

    double target_position[2]; // x, y
    std::vector< std::pair<double, double> > target_sequence;
    int current_tar;
    int target_sector;
    double tar_dist;

    double Euler[3]; // yaw-pitch-roll
    double Rob_Pos[3]; // x, y, z

    int current_orientation;

    int s_max;

    std::vector< std::vector<int> > valleys;
    std::vector< std::pair<double, double> > arc_points;
    std::vector<int> candidates;

    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber target_sub;
    ros::Publisher scan_pub;
    ros::Publisher vel_pub;
    ros::Publisher scan_pub_3;
};

VFHfollowing::VFHfollowing()
{
    for(int i = 0; i < 360; i++)
    {
        raw_scan[i] = 1;
        certainty[i] = 1;
    }

    countdown = 1;
    TARGET_NUMBER = 1;
    command_number = -1;
    threshold = 0.8;

    para_a = 1.0;
    para_b = 0.7;

    robot_radius = 0.125;
    safety_distance = 0.03;

    sector_size = 9;
    one_degree = 0.0174532923847;

    range_min = 0.1;
    range_max = 2.0;

    upper_th = 0.687; // 0.20cm
    lower_th = 0.616; // 0.30cm

    // the sector target direction is in
    target_position[0] = 0.2;
    target_position[1] = 0.6;
    current_tar = -1;
    target_sector = 0;

    // the se4ctor of current orientation(seems to be always 0, maybe useless here)
    current_orientation = 0;
    tar_dist = 0.2;

    s_max = 6;

    m1 = 3;
    m2 = 1;
    m3 = 1;

    prev_sector = -1;

    scan_sub = nh.subscribe("/scan", 1, &VFHfollowing::getScan, this);
    odom_sub = nh.subscribe("/localization/pose", 1, &VFHfollowing::EulerQuaternion, this);
    target_sub = nh.subscribe("/guider", 1, &VFHfollowing::getTarget, this);

    scan_pub = nh.advertise<std_msgs::Float32MultiArray>("histogram", 1);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
    scan_pub_3 = nh.advertise<std_msgs::Float32MultiArray>("histogram_3", 1);

    std::vector<int> valley_temp;
    valley_temp.push_back(1);
    valley_temp.push_back(2);
    valley_temp.push_back(3);

    valleys.push_back(valley_temp);
}

void VFHfollowing::getScan(sensor_msgs::LaserScan scan_msg)
{
    if(--countdown == 0)
    {
      countdown = 5;
      for(int i = 0; i < 150; i++)
      {
          if(isinf(scan_msg.ranges[i]))
          {
             if(i == 0)
             {
                 raw_scan[i] = range_max;
             }
             else{
                raw_scan[i] = raw_scan[i-1];
             }
          }
          else{
              raw_scan[i] = scan_msg.ranges[i];

              // maybe risky to set it like this !
              if (raw_scan[i] > range_max || raw_scan[i] < range_min)
              {
                  raw_scan[i] = range_max;
              }
          }
      }

      // to eliminate the points due to back corner
      for(int i = 150; i < 210; i++)
      {
          if(isinf(scan_msg.ranges[i]))
          {
             if(i == 0)
             {
                 raw_scan[i] = range_max;
             }
             else{
                raw_scan[i] = raw_scan[i-1];
             }
          }
          else{
              raw_scan[i] = scan_msg.ranges[i];

              // maybe risky to set it like this !
              if (raw_scan[i] > range_max)
              {
                  raw_scan[i] = range_max;
              }

              if (raw_scan[i] < 0.28 && (raw_scan[i-1] > 0.3 || scan_msg.ranges[i+1] > 0.3))
              {
                  raw_scan[i] = raw_scan[i-1];
              }

          }
      }

      for(int i = 210; i < 360; i++)
      {
          if(isinf(scan_msg.ranges[i]))
          {
             if(i == 0)
             {
                 raw_scan[i] = range_max;
             }
             else{
                raw_scan[i] = raw_scan[i-1];
             }
          }
          else{
              raw_scan[i] = scan_msg.ranges[i];

              // maybe risky to set it like this !
              if (raw_scan[i] > range_max || raw_scan[i] < range_min)
              {
                  raw_scan[i] = range_max;
              }
          }
      }

    }
}

void VFHfollowing::getTarget(std_msgs::Float32MultiArray tar_msg)
{
    // get the target position
    // and compute the target sector using odometry

    if(current_tar == -1)
        current_tar = 0;

    if (command_number != (int)tar_msg.data[tar_msg.data.size()-1])
    {
        command_number = (int)tar_msg.data[tar_msg.data.size()-1];
        current_tar = 0;
        ROS_INFO("new command!");
    }

    target_sequence.clear();
    TARGET_NUMBER = (tar_msg.data.size()-1)/2;
    // ROS_INFO("target number %d", TARGET_NUMBER);

    std::stringstream ss;
    ss << "target sequence: ";
    for(int i = 0; i < 2*TARGET_NUMBER; i++)
    {
        ss << "[ ";
        target_sequence.push_back(std::make_pair(tar_msg.data[i], tar_msg.data[i+1]));
        ss << tar_msg.data[i] << ", ";
        i++;
        ss << tar_msg.data[i] << " ], ";
    }
    std::cerr << ss.str();

    // transform the target point in the robot frame
    double hr = Euler[0];
    double xr = Rob_Pos[0];
    double yr = Rob_Pos[1];

    // target position in the robot frame
    double tar_x;
    double tar_y;

    while(true)
    {
        target_position[0] = target_sequence[current_tar].first;
        target_position[1] = target_sequence[current_tar].second;

        // ROS_INFO("target(%.3f, %.3f), robot(%.3f, %.3f)", target_position[0], target_position[1], xr, yr);

        double delta_x = target_position[0] - xr;
        double delta_y = target_position[1] - yr;
        tar_dist = sqrt(delta_x*delta_x + delta_y*delta_y);

        tar_x = (target_position[0] - xr) * sin(hr) - (target_position[1] - yr) * cos(hr);
        tar_y = (target_position[0] - xr) * cos(hr) + (target_position[1] - yr) * sin(hr);

        if( tar_dist < 0.11|| (tar_y < 0 && tar_dist < 0.13) )
        {
            ROS_INFO("we are here!");
            if(current_tar < TARGET_NUMBER-1)
                current_tar += 1;
            else{
                break;
            }
        }
        else
        {
            break;
        }
    }

    ROS_INFO("target in rob frame (%.3f, %.3f)", tar_x, tar_y);

    double theta = fmod(atan2(tar_y, tar_x) + 4.7124, 6.2832) ;

    int degree = (int)round(theta / 3.14159 * 180);

    target_sector = ((degree + 4)%360)/9;
}

void VFHfollowing::computeMag ()
{
    double temp;

    // when possible, update the certainty value here

    // preprocess the magnitude

    // double window;
    // window = raw_scan[0]+raw_scan[1]+raw_scan[2]+raw_scan[359]+raw_scan[358];
    int minus_num;
    int add_num;

    double average;


    double true_dist;
    double item1;

    // std::stringstream ss;
    // ss << "dist: ";
    for(int i = 0; i < 180; i++)
    {
        item1 = -17*cos((double)(i+1)*3.1415926/180.0);
        true_dist = raw_scan[i] - (item1+sqrt(item1*item1 + 961.32))/200.0;
        scan_mag[i] = certainty[i] * certainty[i] * (para_a -  para_b * sqrt(true_dist) );
        // ss << ' ' << (item1+sqrt(item1*item1 + 961.32))/200.0;
    }
    for(int i = 180; i < 360; i++)
    {
        item1 = 17*cos((double)(i-179)*3.1415926/180.0);
        true_dist = raw_scan[i] - (item1+sqrt(item1*item1 + 961.32))/200.0;
        scan_mag[i] = certainty[i] * certainty[i] * (para_a -  para_b * sqrt(true_dist) );
        // ss << ' ' << (item1+sqrt(item1*item1 + 961.32))/200.0;
    }
    // std::cerr << ss.str() << std::endl;


    // eliminate the outliers
    /*
    for(int i = 0; i < 360; i++)
    {
        if( fabs(raw_scan[i] - range_max) < 0.01 )
        {
            scan_mag[i] = certainty[i] * certainty[i] * 0.05;
            continue;
        }

        average = (window - raw_scan[i])/4.0;
        if( fabs(raw_scan[i] - average) > 0.5 )
        {
            ROS_INFO("outlier! %f, %f ", raw_scan[i], average);
            scan_mag[i] = certainty[i] * certainty[i] * (para_a -  para_b * average * average);
        }
        else{
            scan_mag[i] = certainty[i] * certainty[i] * (para_a -  para_b * raw_scan[i] * raw_scan[i]);
        }

        minus_num = ((i-2)+360)%360;
        add_num = ((i+3)+360)%360;
        window = window + raw_scan[add_num] - raw_scan[minus_num];
    }
    */
}

void VFHfollowing::enlargeObs()
{

    // fill in the sectors

    int start = (sector_size - 1) / 2;

    double max_temp = 0;

    ph.clear();

    for(int j = 0; j < start; j++)
    {
        if (scan_mag[j] > max_temp)
        {
            max_temp = scan_mag[j];
        }
    }
    for(int j = 359 - start; j < 360; j++)
    {
        if (scan_mag[j] > max_temp)
        {
            max_temp = scan_mag[j];
        }
    }

    ph.push_back(max_temp);

    for(int i = 1 ; i < 360/sector_size ; i++)
    {
        // using max let the follower be fragile to outlier !
        max_temp = 0;

        for (int j = start + (i-1)*sector_size; j < start + i*sector_size; j++)
        {
            if (scan_mag[j] > max_temp)
            {
                max_temp = scan_mag[j];
            }
        }

        ph.push_back(max_temp);
    }

    // then, ph(primary histogram is filled)

    // englarge obstacle, needs distance to compute how large the angle should be

}

void VFHfollowing::getBPH ()
{
    // use upper and lower threshold to get binary polar histogram

    double temp;

    bh.clear();

    int k = 0;
    while(true)
    {
        if( ph[k] > upper_th || ph[k] < lower_th )
        { break; }
        else
        { k++; }
    }

    int start_index = k;
    while(true)
    {
        temp = ph[k];
        if (temp > upper_th)
        {
            // higher than upper -> obstacle
            bh.push_back(1);
            k++;
        }
        else if(temp < lower_th)
        {
            // lower than lower -> free space
            bh.push_back(0);
            k++;
        }
        else
        {
            // between thresholds
            bh.push_back(bh[bh.size()-1]);
            k++;
        }

        if (k == ph.size())
        {
            k = 0;
        }

        if (k == start_index)
        { break; }
    }

    for (int i = 16; i < 25; i++)
    {
        bh[i] = 1;
    }

    if (bh.size() != ph.size())
    { ROS_INFO("binary histogram wrong size! bh: %d ph: %d ", (int)bh.size(), (int)ph.size() ); }

    // show the binary value in bh
    /*
    std::stringstream ss;
    for( int p = 0; p < bh.size(); p++ )
    {
        ss << ' ' << bh[p];
    }
    std::cerr << ss.str() << std::endl;
    */
}

int VFHfollowing::sector_operator(int sec, int change)
{
    return ( sec + change + 360/sector_size )%(360/sector_size);
}

void VFHfollowing::getMPH ()
{
    // it is necessary!

    // what we have now is a binary histogram showing the obstacles
    std::vector<int> temp_bh;
    double min_dist;
    double enlarge_angle=0.0;
    double angle_deg;
    int sector_range;

    for(int i = 0; i < bh.size(); i++)
    {
        temp_bh.push_back(0);
    }

    for(int i = 0; i < bh.size(); i++)
    {
        if(bh[i] == 0)
        {
            continue;
        }
        else{
            temp_bh[i] = 1;

            min_dist = pow( (para_a - ph[i])/para_b ,2.0) + 0.04;
            // ROS_INFO("No.%d sector: min_dist %f", i, min_dist );
            if( (safety_distance+robot_radius)/(min_dist+robot_radius) < 0.86 )
                enlarge_angle = asin((safety_distance+robot_radius)/(min_dist+robot_radius));
            else
                enlarge_angle = 1.05;

            angle_deg = enlarge_angle*180.0/3.14159;
            // ROS_INFO("No.%d sector: angle %f", i, angle_deg );

            if(angle_deg > 8.0 && angle_deg < 12.0)
            {
                sector_range = 1;
            }
            else{
                sector_range = (int)floor(angle_deg/12.0);
            }
            if (sector_range > 3)
                sector_range = 3;
            for(int k = -sector_range; k <= sector_range; k++)
            {
                temp_bh[sector_operator (i, k)] = 1;
            }
        }
    }
    bh = temp_bh;
    // ROS_INFO("finish");
}

void VFHfollowing::findCandidate ()
{
    // given the thresholded H, select the valleys for valid valleys
    int i = 0;
    while(true)
    {
        if ( bh[i] == 0 && i < bh.size())
        {
            i++;
        }
        else{
            break;
        }
    }

    if( i == (bh.size()-1) )
    {    ROS_INFO("empty!");  }

    // find all valleys
    valleys.clear();
    std::vector<int> valley_temp;

    int start_index = i;
    while(true)
    {
        if( bh[i] == 0 )
        {
            valley_temp.push_back(i);
        }
        else{
            if (valley_temp.size() != 0)
            {
                valleys.push_back(valley_temp);
                valley_temp.clear();
            }
        }
        i++;
        if (i == bh.size())
        {
            i = 0;
        }
        if (i == start_index)
        {
            if(valley_temp.size() != 0)
            {
                valleys.push_back(valley_temp);
                valley_temp.clear();
            }
            break;
        }
    }
    // ROS_INFO("valleys number %d", (int)valleys.size());

    std::stringstream ss;

    for (int p = 0; p < valleys.size(); p++)
    {
        ss << std::endl << "valley " << p << ": ";
        for (int q = 0; q < valleys[p].size(); q++)
        {
             ss << ' ' << valleys[p][q];
        }
    }

    // std::cerr << ss.str() << std::endl;

    // find the candidate direction sector
    candidates.clear();
    int valley_size=0;
    int cr, cl, cn;
    for(int j = 0; j < valleys.size(); j++)
    {


        valley_size = valleys[j].size();
        if(valley_size == 0)
        {
            continue;
        }

        // wide valley
        int swap;
        if (valley_size > s_max)
        {
            cr = valleys[j][s_max/2];
            cl = valleys[j][valley_size - 1 - s_max/2];

            candidates.push_back(cr);
            candidates.push_back(cl);

            if ( valleys[j][0] > valleys[j][valley_size-1] )
            {
                if( target_sector < cl || target_sector > cr )
                {
                    candidates.push_back(target_sector);
                }
            }

            if ( valleys[j][0] < valleys[j][valley_size-1] )
            {
                if( target_sector < cl && target_sector > cr )
                {
                    candidates.push_back(target_sector);
                }
            }

        }

        // narrow valley
        if (valley_size <= s_max)
        {
            cn = valleys[j][(int)valley_size/2];
            candidates.push_back(cn);

            if(sectordiff (target_sector, cn) == 1)
            {
                candidates.push_back(target_sector);
            }

        }

    }

    /*
    std::stringstream ssr;

    for (int p = 0; p < candidates.size(); p++)
    {
        ssr << " candidate: " << candidates[p] << " ||| ";
    }

    std::cerr << ssr.str() << std::endl;
    */

    // compute cost for each candidate direction
    double min_cost = 999999;
    double temp_cost = 0;
    int min_index = 0;

    for(int j = 0; j < candidates.size(); j++)
    {
        if (prev_sector == -1)
        {
            temp_cost = m1 * sectordiff(candidates[j], target_sector) + m2 * sectordiff(candidates[j], current_orientation);
        }
        else{
            temp_cost = m1 * sectordiff(candidates[j], target_sector) + m2 * sectordiff(candidates[j], current_orientation) + m3 * sectordiff(candidates[j], prev_sector) ;
        }

        if (temp_cost < min_cost)
        {
            min_cost = temp_cost;
            min_index = j;
        }
    }

    prev_sector = candidates[min_index];
}

int VFHfollowing::sectordiff(int sec1, int sec2)
{
    int a1 = abs(sec1 - sec2);
    int a2 = abs(sec1 - sec2 - 360/sector_size);
    int a3 = abs(sec1 - sec2 + 360/sector_size);

    if(a1 < a2 && a1 < a3)
        return a1;
    else if (a2 < a1 && a2 < a3)
        return a2;
    else
        return a3;

}

int VFHfollowing::checkdirection(int sec1, int sec2)
{
    // 0: sec1 is "right" and sec2 is "left"
    // 1: sec1 is "left"  and sec2 is "right"

}

void VFHfollowing::computeSteering()
{

    // compute for the ang_vel and lin_vel for robot to go
    double lin_vel, ang_vel;
    geometry_msgs::Twist vel_msg;
    lin_vel = 0.06; // set a fixed linear velocity first

    double delta_x = target_position[0] - Rob_Pos[0];
    double delta_y = target_position[1] - Rob_Pos[1];
    tar_dist = sqrt(delta_x*delta_x + delta_y*delta_y);


    if(tar_dist < 0.1 && ( current_tar == -1|| current_tar >= TARGET_NUMBER-1) )
    {
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;
        vel_pub.publish(vel_msg);
        return;
    }
    // special case, when it just needs to go straight
    else if(prev_sector == 0)
    {
        vel_msg.linear.x = lin_vel * 1.5;
        vel_msg.angular.z = 0.0;
        vel_pub.publish(vel_msg);
        arc_points.clear();
        for (int i = 0; i < 4; i++)
        {
            arc_points.push_back(std::make_pair(0.15+0.1*i, 180.0));
        }

        return;
    }
    // special case of turning too much, do a pure turning first
    else if(prev_sector > 8 && prev_sector < 32 )
    {
        vel_msg.linear.x = 0.0;
        int sec_err = sectordiff (prev_sector, current_orientation);
        vel_msg.angular.z = 0.2;
        vel_pub.publish(vel_msg);
        arc_points.clear();
        for (int i = 0; i < 4; i++)
        {
            arc_points.push_back(std::make_pair(0.5, ((current_orientation+i*sec_err/3)*9+180)%360 ));
        }
    }
    else{
        // first. compute goal point in robot frame
        ROS_INFO("distance: %f", tar_dist);

        double dist_list[8] = {0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40};
        // we don't want to go further than the target itself


        if( tar_dist > 0.20 )
            for(int i = 0; i < 8; i++)
            {
                if(dist_list[i] <= tar_dist)
                    continue;
                else{
                    dist_list[i] = tar_dist;
                }
            }

        double direction = ((double)prev_sector*9+90.0)*3.14159/180.0;

        // ROS_INFO("sector: %d, direction: %f", prev_sector, direction);

        std::pair<double, double> goal_points[8];

        double sind = sin(direction);
        double cosd = cos(direction);


        for(int i = 0; i < 8; i++)
        {
            goal_points[i] = std::make_pair(dist_list[i]*cosd , dist_list[i]*sind);
            // ROS_INFO("goal_x: %f, goal_y: %f", dist_list[i]*cosd, dist_list[i]*sind);
        }

        double tar_radius[8] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        // when radius < 0, center of circle is on the right
        // when radius > 0, center of circle is on the left
        for(int i = 0; i < 8; i++)
        {
            tar_radius[i] = -( dist_list[i]*dist_list[i]/( 2*(goal_points[i].first) ) ) ;
            // std::cerr << std::endl;
            // ROS_INFO("target_radius %d: %f", i, tar_radius[i]);
        }

        // center of that arc is at (radius, 0) in robot frame
        int zone_num = 8;
        double alpha;
        double ang_step;
        std::vector<double> check_ang;
        std::vector< std::pair<double, double> > check_point;

        double px, py;
        double Len;
        double rho, rho_ceil, rho_floor;
        double reach;

        int valid_max = 0;
        bool valid_arc;


        for(int i = 0; i < 8; i++)
        {
            alpha = atan2((goal_points[i].second), (goal_points[i].first)+tar_radius[i]);
            // ROS_INFO("alpha %d: %f", i, alpha);
            check_ang.clear();
            if(tar_radius[i] > 0) // center is on the left of lidar
            {
                ang_step = alpha / (double)zone_num;
                for(int j = 0; j < zone_num; j++)
                {
                    check_ang.push_back(0.0 + ang_step*(j+1));
                    // ROS_INFO("check angle %d: %f", j, ang_step*(j+1));
                }
            }
            else{ // center is on the right of lidar
                ang_step = (3.1415926 - alpha) / (double)zone_num;
                for(int j = 0; j < zone_num; j++)
                {
                    check_ang.push_back(alpha + ang_step*j);
                    // ROS_INFO("check angle %d: %f", j, alpha + ang_step*j);
                }
            }

            check_point.clear();
            valid_arc = true;
            for(int j = 0; j < zone_num; j++)
            {
                Len = fabs(tar_radius[i]) + robot_radius + 0.04; // 0.14 is the distance we want to guarrantee so that our corner won't hit the wall
                px = -tar_radius[i] + Len * cos(check_ang[j]);
                py = 0.0 + Len * sin(check_ang[j]);
                // ROS_INFO("px: %f, py: %f", px, py);

                rho = atan2(py, px) * 180.0/3.1415926;
                rho_ceil = ceil(rho);
                rho_floor = floor(rho);

                rho_ceil = fmod((rho_ceil + 360.0 - 91.0), 360);
                rho_floor = fmod((rho_ceil + 360.0 - 91.0), 360);

                // check whether obstacle is in range
                reach = sqrt( px*px + py*py );
                check_point.push_back(std::make_pair(reach, rho+90.0));
                // ROS_INFO("reach: %f, rho: %f", reach, rho);
                if ( (raw_scan[(int)rho_ceil] + raw_scan[(int)rho_floor]) / 2.0 <= reach )
                {
                    valid_arc = false;
                    break;
                }
            }
            if(valid_arc==true)
            {
                valid_max = i;
                arc_points = check_point;
                // ROS_INFO("chosen distance: %f ", dist_list[i]);
            }
        }
        // send out velocity
        vel_msg.linear.x = lin_vel;
        vel_msg.angular.z = lin_vel / tar_radius[valid_max];

        vel_pub.publish(vel_msg);
    }
}

std::pair<double, double> VFHfollowing::extendLine(std::pair<double, double> p1, std::pair<double, double> p2, double Len)
{
    std::pair<double, double> p3;

    double x1 = p1.first;
    double y1 = p1.second;
    double x2 = p2.first;
    double y2 = p2.second;
    double dist_12 = sqrt( (y2-y1)*(y2-y1) + (x2-x1)*(x2-x1) );
    double x3 = x2 + (x2-x1)*(Len/dist_12);
    double y3 = y2 + (y2-y1)*(Len/dist_12);

    p3 = std::make_pair(x3, y3);
    return p3;
}

std::pair<double, double> VFHfollowing::CirclePoint(std::pair<double, double> cen, double Len, double ang)
{
    std::pair<double, double> point;

    double px, py;
    px = (cen.first) + Len * cos(ang);
    py = (cen.second) + Len * sin(ang);

    point = std::make_pair(px, py);

    return point;
}

void VFHfollowing::EulerQuaternion(geometry_msgs::PoseStamped odom)
{
    // get the odometry - orientation
    double q0 = odom.pose.orientation.x;
    double q1 = odom.pose.orientation.y;
    double q2 = odom.pose.orientation.z;
    double q3 = odom.pose.orientation.w;

    // get the odometry - position
    Rob_Pos[0] = odom.pose.position.x;
    Rob_Pos[1] = odom.pose.position.y;
    Rob_Pos[2] = odom.pose.position.z;

    // double e1, e2, e3;
    // e1 = atan2(q2*q3 + q0*q1 , 0.5-(q1*q1+q2*q2));
    // e2 = asin(-2*(q1*q3-q0*q2));
    // e3 = atan2((q1*q2+q0*q3), 0.5-(q2*q2+q3*q3));

    double roll, pitch, yaw;
    tf::Quaternion quater;
    tf::quaternionMsgToTF(odom.pose.orientation, quater);
    tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);

    yaw = angles::normalize_angle_positive(yaw);

    Euler[0] = fmod(yaw, 6.2832); // yaw angle in radius
    Euler[1] = 0;
    Euler[2] = 0;

}

void VFHfollowing::run()
{
    ros::Rate r(5);

    while (ros::ok()) {

        if(countdown == 5)
        {
            // ROS_INFO("computing magnitude!");
            computeMag ();
            // ROS_INFO("enlarging obstacles!");
            enlargeObs ();
            // ROS_INFO("getting binary histogram!");
            getBPH ();

            getMPH ();

            // ROS_INFO("looking for candidates!");
            findCandidate ();

            computeSteering ();


            int j;
            std_msgs::Float32MultiArray msg;

            for(int i = 0; i < 360; i++)
            {
                msg.data.push_back((double)raw_scan[i]);
            }
            int cand[360];
            for(int i = 0; i < 360; i++)
            {
                msg.data.push_back((double)scan_mag[i]);
                cand[i] = 0;
            }
            for(int i = 0; i < 360; i++)
            {
                j = (i + 5)%360;
                msg.data.push_back((double)bh[ j/sector_size ]);
            }

            // ROS_INFO("candidate number %d: ", (int)candidates.size());
            for(int i = 0; i < candidates.size(); i++)
            {
                if(candidates[i] == 0)
                {
                    cand[359] = 1;
                }
                else{
                    cand[candidates[i]*9 - 1] = 1;
                }
            }

            for(int i = 0; i < 360; i++)
            {
                // msg.data.push_back((double)bh[ j/sector_size ]);
                msg.data.push_back((double)cand[i]);
            }

            for(int i = 0; i < 4; i++)
            {
                msg.data.push_back(arc_points[i].first);  // reach
                msg.data.push_back(arc_points[i].second);  // rho
            }
            msg.data.push_back((double)prev_sector*9);

            scan_pub.publish(msg);

            // Euler angle here is changed into degrees
            ROS_INFO("Robot angle: %f || Robot position: %f, %f || Target position %f, %f ", Euler[0]*(180.0/3.14159), Rob_Pos[0], Rob_Pos[1], target_position[0], target_position[1] );
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

