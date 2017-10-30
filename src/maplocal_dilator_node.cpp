/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, University of Michigan, Ann Arbor (ROAHM Lab)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Fan Bu
 */

// This ROS node reads in an occupency grid map and dilates the obstacle points, then publish out the contour of those dilated points.

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Int8.h"
#include <iostream> 
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include <sstream>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/foreach.hpp>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sys/time.h>
#include <time.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

using namespace cv;

ros::Publisher map_pub;
std::string odom_topic_name = "/segway/odometry/local_filtered";
std::string map_topic_name = "map";
std::string grid_obstacles_array_name = "/roahm/grid_obstacles/array";
std::string obstacles_pcd_topic_name = "/roahm/grid_obstacles/pcd";
std::string contour_topic_name = "/roahm/contour/array";
std::string map_dilated_contour_name = "/roahm/dilated_obstacles/array";
std::string map_dilated_img_name = "/roahm/dilated_obstacles/image";
std::string outer_contour_topic_name = "/roahm/outer_contour/array";
std::string map_outer_dilated_contour_name = "/roahm/outer_dilated_obstacles/array";
std::string map_outer_dilated_img_name = "/roahm/outer_dilated_obstacles/image";
std::string pose_on_map_name = "/roahm/pose_odom";
std::string odom_on_map_name = "/roahm/odom_on_map";
// std::string lidar_topic_name = "/first";
std::string lidar_pcd_name = "/scan_matched_points2";
std::string prediction_topic_name = "/forecast/output";

nav_msgs::OccupancyGrid now_map;
nav_msgs::Odometry now_odom;
std_msgs::Float32MultiArray now_obstacles_array;
std_msgs::Float32MultiArray now_dilated_obs_array;
std_msgs::Float32MultiArray now_contour_array;
std_msgs::Float32MultiArray now_outer_dilated_obs_array;
std_msgs::Float32MultiArray now_outer_contour_array;
pcl::PointCloud<pcl::PointXYZ> now_obs_cloud;



// sensor_msgs::LaserScan now_scan;

pcl::PointCloud<pcl::PointXYZ> now_lidar_cloud;
pcl::PointCloud<pcl::PointXYZRGB> now_pred;



bool use_range_filter = false;
double range_threshold = 5.0;   // meters
double localfilter_size = 16.0;  // meters (default localfilter is a 6x6 m^2 sqaure)
// bool use_range_filter;
// double range_threshold;   // meters
// double localfilter_size;  // meters (default localfilter is a 6x6 m^2 sqaure)

int now_obs_num;
int now_obs_contour_num;
int now_outer_obs_contour_num;
Mat now_obs_dilated_img;
Mat now_outer_obs_dilated_img;

int dilation_elem = 0;
// int dilation_elem;
 // if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
 //     else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
 //     else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
double buffer_size = 0.3;
// double buffer_size ;
// buffer_size is the obstacle buffer size. Its unit is in meters.
double outer_buffer_size = 0.35;
// Unlike buffer_size, outer_buffer_size is a larger buffer size for obstacles, it is designed for global waypoints planning. Its unit is in meters.
int publish_rate = 25;
double obs_confidence_threshold = 80;
int dilation_size;
int dilation_type;
bool map_processing = false;
double obs_reduce_distance = 0.2;
Mat element;
// double map_width;
// double map_height;
// float map_resolution;
// float map_origin_x;
// float map_origin_y;

double get_wall_time() {
    struct timeval time;
    if (gettimeofday(&time,NULL)) {
        //  Handle error
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * 1e-6;
    //return (double)time.tv_usec;
}


void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    map_processing = true;
    std_msgs::Header header = msg->header;
    nav_msgs::MapMetaData info = msg->info;

    ROS_INFO("Got map %d x %d", info.width, info.height);
    ROS_INFO("Map resolution is %f", info.resolution);
    ROS_INFO("Map origin is [%f], [%f], [%f]", info.origin.position.x, info.origin.position.y, info.origin.position.z);
    now_map = *msg;
    ROS_INFO("Now odometry position is -> x: [%f], y: [%f], z: [%f]", now_odom.pose.pose.position.x, now_odom.pose.pose.position.y, now_odom.pose.pose.position.z);
    // While reading grid map:
    // Unknow area == -1;
    // No obstacle == 0;
    // Obstacle == 100;
    // ROS_INFO("Cell number == %i", msg->data[1984*1984/2+1984/2]);
    map_processing = false;

}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    now_odom = *odom_msg;

    // ROS_INFO("Got map %d %d", now_map.info.width, now_map.info.height);
    // ROS_INFO("Seq: [%d]", msg->header.seq);
    // ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    // ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}

void lidarpcdCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& lidar_cloud)
{
    now_lidar_cloud = *lidar_cloud;
}

void predCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pred_cloud)
{
    now_pred = *pred_cloud;
}




int main(int argc, char **argv){
    // Initialize for the dilation algorithm.
    // if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
 //     else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
 //     else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
 //     element = getStructuringElement( dilation_type,
 //                                   Size( 2*dilation_size + 1, 2*dilation_size+1 ),
 //                                   Point( dilation_size, dilation_size ) );

    ros::init(argc, argv, "maplocal_dilator");
    ros::NodeHandle n;


    tf::TransformListener listener;
    geometry_msgs::PoseStamped pose_odom;
    geometry_msgs::PoseStamped pose_on_map;
    nav_msgs::Odometry odom_on_map;

    

    // map_pub = n.advertise<nav_msgs::OccupancyGrid>("map_out",10);
    ros::Publisher grid_obstacles_array_pub = n.advertise<std_msgs::Float32MultiArray>(grid_obstacles_array_name , 10);
    ros::Publisher obstacles_cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZI> >(obstacles_pcd_topic_name, 1);
    ros::Publisher contour_pub = n.advertise<std_msgs::Float32MultiArray>(contour_topic_name , 10);
    ros::Publisher map_dilated_contour_pub = n.advertise<std_msgs::Float32MultiArray>(map_dilated_contour_name , 10);
    ros::Publisher outer_contour_pub = n.advertise<std_msgs::Float32MultiArray>(outer_contour_topic_name , 10);
    ros::Publisher map_outer_dilated_contour_pub = n.advertise<std_msgs::Float32MultiArray>(map_outer_dilated_contour_name , 10);
    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>(pose_on_map_name, 1);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(odom_on_map_name, 1);
    
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher map_dilated_img_pub = it.advertise(map_dilated_img_name, 1);
    image_transport::Publisher map_outer_dilated_img_pub = it.advertise(map_outer_dilated_img_name, 1);


    ros::Subscriber odom_sub = n.subscribe(odom_topic_name, 0, odomCallback);   // Subscribe current odometry generated by segway encoders
    ros::Subscriber map_sub = n.subscribe(map_topic_name,0,mapCallback);        // Subscribe current map ganerated by SLAM algorithm
    ros::Subscriber lidar_pcd_sub = n.subscribe(lidar_pcd_name,0,lidarpcdCallback);      // Subscribe a pointcloud that ganerated by lidar signal, google Cartographer already transformed this pcd into "map" frame
    ros::Subscriber pred_cloud_sub = n.subscribe(prediction_topic_name,0,predCallback); // Subscribe prediction pointclouds for pedestrians

    // Update parameters from launch file
    ros::NodeHandle np("~");
    np.getParam("map", map_topic_name);
    np.getParam("/segway/odometry/local_filtered", odom_topic_name);
    np.getParam("/roahm/grid_obstacles/array", grid_obstacles_array_name);
    np.getParam("/roahm/grid_obstacles/pcd", obstacles_pcd_topic_name);
    np.getParam("/roahm/contour/array", contour_topic_name);
    np.getParam("/roahm/dilated_obstacles/array", map_dilated_contour_name);
    np.getParam("/roahm/dilated_obstacles/image", map_dilated_img_name);
    np.getParam("/roahm/pose_odom", pose_on_map_name);
    np.getParam("/roahm/odom_on_map", odom_on_map_name);
    np.getParam("/roahm/outer_contour/array", outer_contour_topic_name);
    np.getParam("/roahm/outer_dilated_obstacles/array", map_outer_dilated_contour_name);
    np.getParam("/roahm/outer_dilated_obstacles/image", map_outer_dilated_img_name);
    np.getParam("/scan_matched_points2", lidar_pcd_name);  // We are subscribing to a pointcloud because this pcd is alredy in "map" frame
    np.getParam("/forecast/output", prediction_topic_name);

    np.param("use_range_filter", use_range_filter, false);
    np.param("range_threshold", range_threshold, 5.0);
    np.param("localfilter_size", localfilter_size, 6.0);
    np.param("dilation_elem", dilation_elem, 0);
    np.param("buffer_size", buffer_size, 0.3);
    np.param("outer_buffer_size", outer_buffer_size, 0.5);
    np.param("publish_rate", publish_rate, 20);
    np.param("obs_confidence_threshold", obs_confidence_threshold, 80.0);
    np.param("obs_reduce_distance", obs_reduce_distance, 0.2);  // Instead of scanning the openCV map image, we insert points into the contour array so that the obstacle points for Fmincon would be less dense.  
    



    // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(publish_rate);

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    // int count = 0;
    while (ros::ok() && !map_processing)
    {
        // Before we publish anything, lets transform the odom data into map frame and store the transformed pose in pose_on_map
        
        pose_odom.header.frame_id = now_odom.header.frame_id;
        pose_odom.pose.position =  now_odom.pose.pose.position;
        pose_odom.pose.orientation =  now_odom.pose.pose.orientation;
        // ROS_INFO("Before transformation,pose[x,y] is [%f , %f]", pose_odom.pose.position.x, pose_odom.pose.position.y );

        tf::StampedTransform transform;
        try{
            listener.lookupTransform("odom", "map",
                                    ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("Get transform exception : %s",ex.what());
            // ros::Duration(1.0).sleep();
            continue;
        }

        // pose_on_map.header.frame_id = "map";
        try{

            listener.transformPose("map",pose_odom, pose_on_map); 
            // Publish pose into ROS
            ros::Time now_time = ros::Time::now();
            pose_on_map.header.stamp = now_time ;
            pose_pub.publish(pose_on_map);

            // Publish odometry into ROS
            odom_on_map.header.stamp = now_time ;
            odom_on_map.header.frame_id = "map";
            //set the position
            odom_on_map.pose.pose.position.x = pose_on_map.pose.position.x;
            odom_on_map.pose.pose.position.y = pose_on_map.pose.position.y;
            odom_on_map.pose.pose.position.z = 0.0;
            odom_on_map.pose.pose.orientation = pose_on_map.pose.orientation;
            //set the velocity
            odom_on_map.child_frame_id = "segway/base_link";
            odom_on_map.twist.twist.linear.x = now_odom.twist.twist.linear.x;
            odom_on_map.twist.twist.angular.z = now_odom.twist.twist.angular.z;
            //publish the message
            odom_pub.publish(odom_on_map);
        }
            catch( tf::TransformException ex)
        {
            ROS_ERROR("Transfroming exception : %s",ex.what());
            // continue;
        }
        // ROS_INFO("After transformation,pose[x,y] is [%f , %f]", pose_on_map.pose.position.x, pose_on_map.pose.position.y );

        if(now_map.info.resolution != 0){
            // Now, let's create a square filter (default: 6x6 m^2) around the robot
            ros::Time tic = ros::Time::now();
            int start_x;
            int start_y;
            int end_x;
            int end_y;
            start_x = int((pose_on_map.pose.position.x - now_map.info.origin.position.x)/now_map.info.resolution) - int(localfilter_size/2/now_map.info.resolution);
            end_x = int((pose_on_map.pose.position.x - now_map.info.origin.position.x)/now_map.info.resolution) + int(localfilter_size/2/now_map.info.resolution);
            start_y = int((pose_on_map.pose.position.y - now_map.info.origin.position.y)/now_map.info.resolution) - int(localfilter_size/2/now_map.info.resolution);
            end_y = int((pose_on_map.pose.position.y - now_map.info.origin.position.y)/now_map.info.resolution) + int(localfilter_size/2/now_map.info.resolution);
            if(start_x<0){
                start_x = 0;
            }
            if(start_x >= now_map.info.width){
                start_x = now_map.info.width-1;
            }
            if(end_x<0){
                end_x = 0;
            }
            if(end_x >= now_map.info.width){
                end_x = now_map.info.width-1;
            }
            if(start_y<0){
                start_y = 0;
            }
            if(start_y >= now_map.info.height){
                start_y = now_map.info.height-1;
            }
            if(end_y<0){
                end_y = 0;
            }
            if(end_y >= now_map.info.height){
                end_y = now_map.info.height-1;
            }

            double coord_x;
            double coord_y;
            int obs_cnt = 0;
            Mat obs_img(end_y-start_y+1, end_x-start_x+1, CV_8UC1);

            //  Initialize for publishing obs points into pcd format
            pcl::PointCloud<pcl::PointXYZ>::Ptr obs_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            std_msgs::Float32MultiArray grid_obstacles_array_msg;
            grid_obstacles_array_msg.data.clear();

            // Draw map obstacles on cv image
            for (int x = start_x; x <= end_x; x++){
                // if(x<0 || x>=now_map.info.width){
                //     continue;
                // }
                for (int y = start_y; y <= end_y; y++){
                    if(x<0 || x>=now_map.info.width){
                        ROS_INFO("Local window reaches out the map range, we'll only receive what's within the map." );
                        std::cout <<"Now, x = "<< x << ", y = "<< y <<std::endl;
                        std::cout <<"start_x = "<< start_x << std::endl;
                        std::cout <<"start_y = "<< start_y << std::endl;
                        std::cout <<"end_x = "<< end_x << std::endl;
                        std::cout <<"end_y = "<< end_y << std::endl;
                        std::cout <<"cheking now_map.data["<< x << "+" << now_map.info.width << "*" << y << "]" << std::endl;
                        continue;
                    }

                    // Draw obstacles from map to cv image if confidence is larger than threshold.
                    if (now_map.data[x+ now_map.info.width * y] > obs_confidence_threshold){      //map value is 0 ~ 100, if blank, value = -1
                        // ROS_INFO("Obstacle position x: [%i], y: [%i]", x, y);
                        // ROS_INFO("Value: [%i]", now_map.data[x+ now_map.info.width * y]);
                        coord_x = double(now_map.info.origin.position.x) + double(now_map.info.resolution)*double(x+0.5);
                        coord_y = double(now_map.info.origin.position.y) + double(now_map.info.resolution)*double(y+0.5);
                        // ROS_INFO("x,y is: [%f],[%f]", coord_x, coord_y);
                        grid_obstacles_array_msg.data.push_back(coord_x);
                        grid_obstacles_array_msg.data.push_back(coord_y);
                        obs_img.at<uchar>(y-start_y, x-start_x)= 255; 
                        obs_cloud->points.push_back (pcl::PointXYZ(coord_x, coord_y, 0));
                        obs_cnt++;
                    }
                    else{
                        obs_img.at<uchar>(y-start_y, x-start_x)= 0; 
                    }

                }
            }

            // Draw lidar poinclouds on obs_img, google Cartographer already transformed this pcd into "map" frame.
            float filter_min_x = pose_on_map.pose.position.x - localfilter_size/2;
            float filter_max_x = pose_on_map.pose.position.x + localfilter_size/2;
            float filter_min_y = pose_on_map.pose.position.y - localfilter_size/2;
            float filter_max_y = pose_on_map.pose.position.y + localfilter_size/2;
            if(now_lidar_cloud.width>0){
                // ROS_INFO("Lidar pcd[0] is:%f,%f,%f", now_lidar_cloud.points[0].x, now_lidar_cloud.points[0].y, now_lidar_cloud.points[0].z);
                for(int i=0; i < now_lidar_cloud.width; i++){
                    if(now_lidar_cloud.points[i].x > filter_min_x && now_lidar_cloud.points[i].x < filter_max_x &&
                      now_lidar_cloud.points[i].y > filter_min_y && now_lidar_cloud.points[i].y < filter_max_y){
                        int x = int((now_lidar_cloud.points[i].x-now_map.info.origin.position.x)/now_map.info.resolution);
                        int y = int((now_lidar_cloud.points[i].y-now_map.info.origin.position.y)/now_map.info.resolution);
                        if (obs_img.at<uchar>(y-start_y, x-start_x) != 255){
                            obs_img.at<uchar>(y-start_y, x-start_x)= 255; 
                            obs_cloud->points.push_back (pcl::PointXYZ(now_lidar_cloud.points[i].x, now_lidar_cloud.points[i].y, 0));
                            obs_cnt++;
                        }
                        
                    }
                }
            }
            else{
                ROS_INFO("No lidar pointcloud exist!");
            }

            // Draw pedestrian prediction poinclouds on obs_img, which are already in "map" frame.
            if(now_pred.width>0){
                // ROS_INFO("now_pred.width is: %i", now_pred.width);
                for(int i=0; i < now_pred.width; i++){
                    if(now_pred.points[i].x > filter_min_x && now_pred.points[i].x < filter_max_x &&
                      now_pred.points[i].y > filter_min_y && now_pred.points[i].y < filter_max_y){
                        int x = int((now_pred.points[i].x-now_map.info.origin.position.x)/now_map.info.resolution);
                        int y = int((now_pred.points[i].y-now_map.info.origin.position.y)/now_map.info.resolution);
                        if (obs_img.at<uchar>(y-start_y, x-start_x) != 255){
                            obs_img.at<uchar>(y-start_y, x-start_x)= 255; 
                            obs_cloud->points.push_back (pcl::PointXYZ(now_pred.points[i].x, now_pred.points[i].y, 0));
                            obs_cnt++;
                        }
                    }
                }
            }

            // cv::namedWindow("ADSA");
            // cv::imshow("ADSA",obs_img);
            // cv::waitKey(0);
            // ros::Time toc = ros::Time::now();
            // ros::Duration diff = toc - tic;
            // std::cout <<"Finding SLAM grid(with local filter) took: "<< diff <<"seconds" << std::endl;

            now_obstacles_array = grid_obstacles_array_msg;
            now_obs_cloud = *obs_cloud;
            now_obs_num = obs_cnt;
            
            ROS_INFO("There are %i original obstacle points on the map.", obs_cnt);


            // Dilate the local map image (only for obstacles)
            // tic = ros::Time::now();
            dilation_size = buffer_size/double(now_map.info.resolution);
            // std::cout <<"  buffer_size: "<< buffer_size << " meters." << std::endl;
            // std::cout <<"  now_map.info.resolution: "<< now_map.info.resolution << " meters/col." << std::endl;
            // std::cout <<"  dilation_size = buffer_size/double(now_map.info.resolution) =  "<< dilation_size << " columns." << std::endl;

            if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
            else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
            else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
            element = getStructuringElement( dilation_type,
                                           Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                           Point( dilation_size, dilation_size ) );

            Mat obs_dilated_img;
            dilate(obs_img, obs_dilated_img, element);      //This is where the error popped out!!!!!
            // toc = ros::Time::now();
            // diff = toc - tic;
            // std::cout <<"Dilation took: "<< diff <<"seconds" << std::endl;

            // Dilate the local map image (use larger buffer for global path planning)
            // tic = ros::Time::now();
            dilation_size = outer_buffer_size/double(now_map.info.resolution);
            // std::cout <<"  outer_buffer_size: "<< outer_buffer_size << " meters." << std::endl;
            // std::cout <<"  now_map.info.resolution: "<< now_map.info.resolution << " meters/col." << std::endl;
            // std::cout <<"  dilation_size = buffer_size/double(now_map.info.resolution) =  "<< dilation_size << " columns." << std::endl;
            element = getStructuringElement( dilation_type,
                                           Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                           Point( dilation_size, dilation_size ) );
            Mat outer_obs_dilated_img;
            dilate(obs_img, outer_obs_dilated_img, element);      //This is where the error popped out!!!!!
            // toc = ros::Time::now();
            // diff = toc - tic;
            // std::cout <<"Dilation took: "<< diff <<"seconds" << std::endl;





            // Find contour of the dialated image (only for obstacles)
            // tic = ros::Time::now();
            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;
            findContours(obs_dilated_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

            // Read contour points coordinates and publish contour arrays(First contour point repeated!!)
            std_msgs::Float32MultiArray contour_array;
            contour_array.data.clear();
            int contour_cnt = 0;
            double coord_x_first;
            double coord_y_first;
            double coord_x_prev;
            double coord_y_prev;

            double dist;
            std_msgs::Float32MultiArray dilated_obs_contour_array;
            dilated_obs_contour_array.data.clear();

            for (int i = 0; i < contours.size(); i++){
                if (i > 0){
                    contour_array.data.push_back(NAN);
                    contour_array.data.push_back(NAN);
                }
                for (int j = 0; j < contours[i].size(); j++){
                    coord_x = double(now_map.info.origin.position.x) + double(now_map.info.resolution)*(double(contours[i][j].x+start_x)+0.5);
                    coord_y = double(now_map.info.origin.position.y) + double(now_map.info.resolution)*(double(contours[i][j].y+start_y)+0.5); 
                    if (j == 0){                         //Save the first contour point!!
                        coord_x_first = coord_x;  //Save the first point for each obstacle
                        coord_y_first = coord_y;  //Save the first point for each obstacle
                        coord_x_prev = coord_x;  //Save coord_x_prev for dilated_obs_contour_array
                        coord_y_prev = coord_y;  //Save coord_y_prev for dilated_obs_contour_array
                    }
                    else{
                        dist = sqrt(pow(double(coord_x - coord_x_prev),2) + pow(double(coord_y - coord_y_prev),2));
                        if (dist > obs_reduce_distance){    // Insert points into the dilated_obs_contour_array if two consequtive contour points are too far.
                            int num_insert = int(dist/obs_reduce_distance);
                            for(int k = 0; k<num_insert ; k++){
                                double coord_x_insert = (k*(coord_x - coord_x_prev)*obs_reduce_distance/dist) + coord_x_prev;
                                double coord_y_insert = (k*(coord_y - coord_y_prev)*obs_reduce_distance/dist) + coord_y_prev;
                                dilated_obs_contour_array.data.push_back(coord_x_insert);
                                dilated_obs_contour_array.data.push_back(coord_y_insert);
                            }
                        }
                        coord_x_prev = coord_x;
                        coord_y_prev = coord_y;
                    }
                    contour_array.data.push_back(coord_x);
                    contour_array.data.push_back(coord_y);

                    dilated_obs_contour_array.data.push_back(coord_x);
                    dilated_obs_contour_array.data.push_back(coord_y);


                    if (j == contours[i].size() - 1){    //Repeat the first contour point!!
                        contour_array.data.push_back(coord_x_first);
                        contour_array.data.push_back(coord_y_first);
                    }
                    contour_cnt++;
                }
            }
            now_contour_array = contour_array;
            now_dilated_obs_array = dilated_obs_contour_array;
            now_obs_contour_num = contour_cnt;
            // ROS_INFO("There are %i sharp contour points on the map.", now_obs_contour_num);
            // toc = ros::Time::now();
            // diff = toc - tic;
            // std::cout <<"Publish contour arrays took: "<< diff <<"seconds" << std::endl;

            // Draw the contours out
            // tic = ros::Time::now();
            int linewidth = 1;
            Mat result(obs_dilated_img.size() , CV_8UC1 , cv::Scalar(255)) ;  
            drawContours(result , contours , -1 , cv::Scalar(0) , linewidth) ; 
            // Draw the robot as a little cross
            int x = int((pose_on_map.pose.position.x-now_map.info.origin.position.x)/now_map.info.resolution);
            int y = int((pose_on_map.pose.position.y-now_map.info.origin.position.y)/now_map.info.resolution);
            result.at<uchar>(y-start_y, x-start_x)= 100; 
            result.at<uchar>(y-start_y-1, x-start_x)= 100; 
            result.at<uchar>(y-start_y+1, x-start_x)= 100; 
            result.at<uchar>(y-start_y, x-start_x-1)= 100; 
            result.at<uchar>(y-start_y, x-start_x+1)= 100; 

            cv::flip(result, now_obs_dilated_img, 0);
            // toc = ros::Time::now();
            // diff = toc - tic;
            // std::cout <<"Draw contours took: "<< diff <<"seconds" << std::endl;

            // Find the coordinates of the contour points(with local filter used).
            // The idea is to draw out the contour we have in now_contour_array data(in color white), and then scan the output image row by row. If a pixel is white, then this pixel is a contour point.
            // tic = ros::Time::now();
            // contour_cnt = 0;
            // double dist;
            // std_msgs::Float32MultiArray dilated_obs_contour_array;
            // dilated_obs_contour_array.data.clear();

            // for (int x = start_x; x <= end_x; x++){
            //     for (int y = start_y; y <= end_y; y++){
            //         if (result.at<uchar>(y-start_y, x-start_x) < 100){ // Pick out the useful points on the image

            //             coord_x = double(now_map.info.origin.position.x) + double(now_map.info.resolution)*double(x+0.5);
            //             coord_y = double(now_map.info.origin.position.y) + double(now_map.info.resolution)*double(y+0.5); 
            //             if (use_range_filter){
            //                 dist = sqrt(pow(double(pose_on_map.pose.position.x - coord_x),2) + pow(double(pose_on_map.pose.position.y - coord_y),2));
            //                 if (dist<range_threshold){
            //                     dilated_obs_contour_array.data.push_back(coord_x);
            //                     dilated_obs_contour_array.data.push_back(coord_y);
            //                     contour_cnt++;
            //                 }
            //             }
            //             else{
            //                 dilated_obs_contour_array.data.push_back(coord_x);
            //                 dilated_obs_contour_array.data.push_back(coord_y);
            //                 contour_cnt++;
            //             }
                        
            //         }
            //     }
            // }
            // toc = ros::Time::now();
            // diff = toc - tic;
            // std::cout <<"Finding contour points took: "<< diff <<"seconds" << std::endl;
            // now_dilated_obs_array = dilated_obs_contour_array;
            // now_obs_contour_num = contour_cnt;
            // ROS_INFO("There are %i contour points on the map.", contour_cnt);



            // Find contour of the outer dialated image (use larger buffer for global path planning)
            // tic = ros::Time::now();
            // vector<vector<Point> > contours; //declared before
            // vector<Vec4i> hierarchy; //declared before
            findContours(outer_obs_dilated_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
            // Read contour points coordinates and publish contour arrays(First contour point repeated!!)
            // std_msgs::Float32MultiArray contour_array;
            contour_array.data.clear();
            contour_cnt = 0; //declared before
            // double coord_x_first; //declared before
            // double coord_y_first; //declared before
            dilated_obs_contour_array.data.clear();
            for (int i = 0; i < contours.size(); i++){
                if (i > 0){
                    contour_array.data.push_back(NAN);
                    contour_array.data.push_back(NAN);
                }
                for (int j = 0; j < contours[i].size(); j++){
                    coord_x = double(now_map.info.origin.position.x) + double(now_map.info.resolution)*(double(contours[i][j].x+start_x)+0.5);
                    coord_y = double(now_map.info.origin.position.y) + double(now_map.info.resolution)*(double(contours[i][j].y+start_y)+0.5); 
                    if (j == 0){                         //Save the first contour point!!
                        coord_x_first = coord_x;  //Save the first point for each obstacle
                        coord_y_first = coord_y;  //Save the first point for each obstacle
                        coord_x_prev = coord_x;  //Save coord_x_prev for dilated_obs_contour_array
                        coord_y_prev = coord_y;  //Save coord_y_prev for dilated_obs_contour_array
                    }
                    else{
                        dist = sqrt(pow(double(coord_x - coord_x_prev),2) + pow(double(coord_y - coord_y_prev),2));
                        if (dist > obs_reduce_distance){    // Insert points into the dilated_obs_contour_array if two consequtive contour points are too far.
                            int num_insert = int(dist/obs_reduce_distance);
                            for(int k = 0; k<num_insert ; k++){
                                double coord_x_insert = (k*(coord_x - coord_x_prev)*obs_reduce_distance/dist) + coord_x_prev;
                                double coord_y_insert = (k*(coord_y - coord_y_prev)*obs_reduce_distance/dist) + coord_y_prev;
                                dilated_obs_contour_array.data.push_back(coord_x_insert);
                                dilated_obs_contour_array.data.push_back(coord_y_insert);
                            }
                        }
                        coord_x_prev = coord_x;
                        coord_y_prev = coord_y;
                    }
                    contour_array.data.push_back(coord_x);
                    contour_array.data.push_back(coord_y);

                    dilated_obs_contour_array.data.push_back(coord_x);
                    dilated_obs_contour_array.data.push_back(coord_y);


                    if (j == contours[i].size() - 1){    //Repeat the first contour point!!
                        contour_array.data.push_back(coord_x_first);
                        contour_array.data.push_back(coord_y_first);
                    }
                    contour_cnt++;
                }
            }
            now_outer_contour_array = contour_array;
            now_outer_dilated_obs_array = dilated_obs_contour_array;
            now_outer_obs_contour_num = contour_cnt;
            ROS_INFO("There are %i sharp outer contour points on the map.", contour_cnt);
            // toc = ros::Time::now();
            // diff = toc - tic;
            // std::cout <<"Publish outer contour arrays took: "<< diff <<"seconds" << std::endl;

            // Draw the contours out
            // tic = ros::Time::now();
            // int linewidth = 1; //declared before
            Mat outer_result(outer_obs_dilated_img.size() , CV_8UC1 , cv::Scalar(255)) ;  //declared before
            drawContours(outer_result , contours , -1 , cv::Scalar(0) , linewidth) ; 
            // Draw the robot as a little cross
            x = int((pose_on_map.pose.position.x-now_map.info.origin.position.x)/now_map.info.resolution);
            y = int((pose_on_map.pose.position.y-now_map.info.origin.position.y)/now_map.info.resolution);
            outer_result.at<uchar>(y-start_y, x-start_x)= 100; 
            outer_result.at<uchar>(y-start_y-1, x-start_x)= 100; 
            outer_result.at<uchar>(y-start_y+1, x-start_x)= 100; 
            outer_result.at<uchar>(y-start_y, x-start_x-1)= 100; 
            outer_result.at<uchar>(y-start_y, x-start_x+1)= 100; 
            // toc = ros::Time::now();
            // diff = toc - tic;
            // std::cout <<"Draw outer contours took: "<< diff <<"seconds" << std::endl;
            // now_obs_dilated_img = outer_result;
            cv::flip(outer_result, now_outer_obs_dilated_img, 0);

            // Find the coordinates of the contour points(with local filter used).
            // The idea is to draw out the contour we have in now_outer_contour_array data(in color white), and then scan the output image row by row. If a pixel is white, then this pixel is a contour point.
            // tic = ros::Time::now();
            // contour_cnt = 0;
            // double dist; //declared before
            // std_msgs::Float32MultiArray dilated_obs_contour_array; // declared before
            // dilated_obs_contour_array.data.clear();

            // for (int x = start_x; x <= end_x; x++){
            //     for (int y = start_y; y <= end_y; y++){
            //         if (outer_result.at<uchar>(y-start_y, x-start_x) < 100){ // Pick out the useful points on the image

            //             coord_x = double(now_map.info.origin.position.x) + double(now_map.info.resolution)*double(x+0.5);
            //             coord_y = double(now_map.info.origin.position.y) + double(now_map.info.resolution)*double(y+0.5); 
            //             if (use_range_filter){
            //                 dist = sqrt(pow(double(pose_on_map.pose.position.x - coord_x),2) + pow(double(pose_on_map.pose.position.y - coord_y),2));
            //                 if (dist<range_threshold){
            //                     dilated_obs_contour_array.data.push_back(coord_x);
            //                     dilated_obs_contour_array.data.push_back(coord_y);
            //                     contour_cnt++;
            //                 }
            //             }
            //             else{
            //                 dilated_obs_contour_array.data.push_back(coord_x);
            //                 dilated_obs_contour_array.data.push_back(coord_y);
            //                 contour_cnt++;
            //             }
                        
            //         }
            //     }
            // }
            // toc = ros::Time::now();
            // diff = toc - tic;
            // std::cout <<"Finding outer contour points took: "<< diff <<"seconds" << std::endl;
            // now_outer_dilated_obs_array = dilated_obs_contour_array;
            // now_outer_obs_contour_num = contour_cnt;
            // ROS_INFO("There are %i outer contour points on the map.", contour_cnt);
            // std::cout << std::endl;







            // Now we can publish all the topics we want.
            ros::Time now = ros::Time::now();

            grid_obstacles_array_pub.publish(now_obstacles_array);
            pcl_conversions::toPCL(now, now_obs_cloud.header.stamp);
            now_obs_cloud.header.frame_id = map_topic_name;
            obstacles_cloud_pub.publish(now_obs_cloud);

            contour_pub.publish(now_contour_array);
            outer_contour_pub.publish(now_outer_contour_array);

            sensor_msgs::ImagePtr dilated_map_img = cv_bridge::CvImage(std_msgs::Header(), "mono8", now_obs_dilated_img).toImageMsg();
            dilated_map_img->header.stamp = now;
            map_dilated_img_pub.publish(dilated_map_img);
            map_dilated_contour_pub.publish(now_dilated_obs_array);

            sensor_msgs::ImagePtr outer_dilated_map_img = cv_bridge::CvImage(std_msgs::Header(), "mono8", now_outer_obs_dilated_img).toImageMsg();
            outer_dilated_map_img->header.stamp = now;
            map_outer_dilated_img_pub.publish(outer_dilated_map_img);
            map_outer_dilated_contour_pub.publish(now_outer_dilated_obs_array);

            
        }
        
        ros::spinOnce();

        loop_rate.sleep();
        // ++count;
    }

    ros::spin();
    return 0;
}
