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

namespace cv // OpenCV 3.2 is stupid, I have to add this block so that cv::vector is defined.
{
    using std::vector;
}

using namespace cv;

ros::Publisher map_pub;
std::string odom_topic_name = "/segway/odometry/local_filtered";
std::string map_topic_name = "map";
std::string grid_obstacles_array_name = "/roahm/grid_obstacles/array";
std::string obstacles_pcd_topic_name = "/roahm/grid_obstacles/pcd";
std::string contour_topic_name = "/roahm/contour/array";
std::string map_dilated_contour_name = "/roahm/dilated_obstacles/array";
std::string map_dilated_img_name = "/roahm/dilated_obstacles/image";
std::string pose_on_map_name = "/roahm/pose_odom";

nav_msgs::OccupancyGrid now_map;
nav_msgs::Odometry now_odom;
std_msgs::Float32MultiArray now_obstacles_array;
std_msgs::Float32MultiArray now_dilated_obs_array;
std_msgs::Float32MultiArray now_contour_array;
pcl::PointCloud<pcl::PointXYZI> now_obs_cloud;

geometry_msgs::PoseStamped pose_on_map;


bool use_range_filter = true;
double range_threshold = 5.0;
int now_obs_num;
int now_obs_contour_num;
Mat now_obs_dilated_img;

int dilation_elem = 0;
 // if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
 //  	else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
 //  	else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
double buffer_size = 0.3;
// buffer_size is the buffer size . Its unit is in meters.
int dilation_size;
int dilation_type;
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
    std_msgs::Header header = msg->header;
    nav_msgs::MapMetaData info = msg->info;
    std_msgs::Float32MultiArray grid_obstacles_array_msg;
    grid_obstacles_array_msg.data.clear();
    
    pcl::PointCloud<pcl::PointXYZI> obs_cloud;

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

    double coord_x;
    double coord_y;
    int obs_cnt = 0;
    Mat obs_img(info.height,info.width, CV_8UC1);

    ros::Time tic = ros::Time::now();

    for (unsigned int x = 0; x < info.width; x++){
        for (unsigned int y = 0; y < info.height; y++){
            if (msg->data[x+ info.width * y] > 0){
                // ROS_INFO("Obstacle position x: [%i], y: [%i]", x, y);
                // ROS_INFO("Value: [%i]", msg->data[x+ info.width * y]);
                coord_x = double(info.origin.position.x) + double(info.resolution)*double(x+0.5);
                coord_y = double(info.origin.position.y) + double(info.resolution)*double(y+0.5);
                // ROS_INFO("x,y is: [%f],[%f]", coord_x, coord_y);
                grid_obstacles_array_msg.data.push_back(coord_x);
                grid_obstacles_array_msg.data.push_back(coord_y);
                obs_img.at<uchar>(y,x)= 255; 
                obs_cnt++;
            }
            else{
            	obs_img.at<uchar>(y,x)= 0; 
            }
        }
    }

    // cv::namedWindow("ADSA");
    // cv::imshow("ADSA",obs_img);
    // cv::waitKey(0);
	ros::Time toc = ros::Time::now();
	ros::Duration diff = toc - tic;
    std::cout <<"Finding SLAM grid took: "<< diff <<"seconds" << std::endl;

    //  Initialize for publishing obs points into pcd format
    tic = ros::Time::now();
    obs_cloud = pcl::PointCloud<pcl::PointXYZI>();
    obs_cloud.width = obs_cnt;
    obs_cloud.height = 1;
    obs_cloud.points.resize(obs_cnt * 1);
    obs_cnt = 0;
    for (unsigned int x = 0; x < info.width; x++){
        for (unsigned int y = 0; y < info.height; y++){
            if (msg->data[x+ info.width * y] > 0){

                coord_x = double(info.origin.position.x) + double(info.resolution)*double(x+0.5);
                coord_y = double(info.origin.position.y) + double(info.resolution)*double(y+0.5);
                obs_cloud.points[obs_cnt].intensity =  msg->data[x+ info.width * y];
                obs_cloud.points[obs_cnt].y = coord_y;
                obs_cloud.points[obs_cnt].x = coord_x;
                obs_cloud.points[obs_cnt].z = 0; 
                obs_cnt++;
            }
        }
    }
    toc = ros::Time::now();
    diff = toc - tic;
    std::cout <<"Translating SLAM grid into pcd took: "<< diff <<"seconds" << std::endl;

 	// Dilate the map image
    dilation_size = buffer_size/double(info.resolution);
    std::cout <<"  buffer_size: "<< buffer_size << " meters." << std::endl;
    std::cout <<"  info.resolution: "<< info.resolution << " meters/col." << std::endl;
    std::cout <<"  dilation_size = buffer_size/double(info.resolution) =  "<< dilation_size << " columns." << std::endl;

	if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  	else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  	else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
  	element = getStructuringElement( dilation_type,
                                   Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                   Point( dilation_size, dilation_size ) );

  	tic = ros::Time::now();
  	Mat obs_dilated_img;
    dilate(obs_img, obs_dilated_img, element); 		//This is where the error popped out!!!!!
    toc = ros::Time::now();
    diff = toc - tic;
    std::cout <<"Dilation took: "<< diff <<"seconds" << std::endl;

 	// Find contour of the dialated image
  	vector<vector<Point> > contours;
  	vector<Vec4i> hierarchy;
    findContours(obs_dilated_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	// Read contour points coordinates and publish contour arrays
    tic = ros::Time::now();

    std_msgs::Float32MultiArray contour_array;
    contour_array.data.clear();
    int contour_cnt = 0;
    for (unsigned int i = 0; i < contours.size(); i++){
        if (i > 0){
            contour_array.data.push_back(NAN);
            contour_array.data.push_back(NAN);
        }
        for (unsigned int j = 0; j < contours[i].size(); j++){
                coord_x = double(info.origin.position.x) + double(info.resolution)*double(contours[i][j].x+0.5);
                coord_y = double(info.origin.position.y) + double(info.resolution)*double(contours[i][j].y+0.5); 
                contour_array.data.push_back(coord_x);
                contour_array.data.push_back(coord_y);
                contour_cnt++;
        }
    }
    now_contour_array = contour_array;
    toc = ros::Time::now();
    diff = toc - tic;
    std::cout <<"Publish contour arrays took: "<< diff <<"seconds" << std::endl;

    // Draw the contours out
    tic = ros::Time::now();
    int linewidth = 1;
    Mat result(obs_dilated_img.size() , CV_8UC1 , cv::Scalar(255)) ;  
    drawContours(result , contours , -1 , cv::Scalar(0) , linewidth) ; 
    toc = ros::Time::now();
    diff = toc - tic;
    std::cout <<"Draw contours took: "<< diff <<"seconds" << std::endl;

    now_obs_dilated_img = result;
    now_obstacles_array = grid_obstacles_array_msg;
    now_obs_cloud = obs_cloud;
    now_obs_num = obs_cnt;
    
    ROS_INFO("There are %i obstacle points on the map.", obs_cnt);

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




int main(int argc, char **argv){
	// Initialize for the dilation algorithm.
	// if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
 //  	else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
 //  	else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
 //  	element = getStructuringElement( dilation_type,
 //                                   Size( 2*dilation_size + 1, 2*dilation_size+1 ),
 //                                   Point( dilation_size, dilation_size ) );

    ros::init(argc, argv, "gridmap_obstacle_reader");
    ros::NodeHandle n;

    tf::TransformListener listener;
    geometry_msgs::PoseStamped pose_odom;

    // map_pub = n.advertise<nav_msgs::OccupancyGrid>("map_out",10);
    ros::Publisher grid_obstacles_array_pub = n.advertise<std_msgs::Float32MultiArray>(grid_obstacles_array_name , 10);
    ros::Publisher obstacles_cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZI> >(obstacles_pcd_topic_name, 1);
    ros::Publisher contour_pub = n.advertise<std_msgs::Float32MultiArray>(contour_topic_name , 10);
    ros::Publisher map_dilated_contour_pub = n.advertise<std_msgs::Float32MultiArray>(map_dilated_contour_name , 10);
    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>(pose_on_map_name, 1);
    
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher map_dilated_img_pub = it.advertise(map_dilated_img_name, 1);


    ros::Subscriber odom_sub = n.subscribe(odom_topic_name, 100, odomCallback);
    ros::Subscriber map_sub = n.subscribe(map_topic_name,1,mapCallback);

    ros::NodeHandle np("~");
    // Update parameters from launch file
    // np.param<bool>("max_group_distance", use_range_filter, false);

    // np.param<double>("range_threshold", range_threshold, 5.0);       // meters
    // np.param<int>("dilation_elem", dilation_elem, 0); 
    //  // if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
    //  //     else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
    //  //     else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
    // np.param<double>("buffer_size", buffer_size, 0.3); 


    np.getParam("use_range_filter", use_range_filter);
    np.getParam("range_threshold", range_threshold);
    np.getParam("dilation_elem", dilation_elem);
    np.getParam("buffer_size", buffer_size);



    // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    // int count = 0;
    while (ros::ok())
    {
    	// Before we publish anything, lets transform the odom data into map frame and store the transformed pose in pose_on_map
		
	    pose_odom.header.frame_id = now_odom.header.frame_id;
	    pose_odom.pose.position =  now_odom.pose.pose.position;
	    pose_odom.pose.orientation =  now_odom.pose.pose.orientation;
	    ROS_INFO("Before transformation,pose[x,y] is [%f , %f]", pose_odom.pose.position.x, pose_odom.pose.position.x );

		tf::StampedTransform transform;
	    try{
	        listener.lookupTransform("/odom", "/map",
	                                ros::Time(0), transform);
	    }
	    catch (tf::TransformException &ex) {
	        ROS_ERROR("Get trnsform exception : %s",ex.what());
	        ros::Duration(1.0).sleep();
	        // continue;
	    }

		try{
		    listener.transformPose("map",pose_odom, pose_on_map); 
		}
		    catch( tf::TransformException ex)
		{
		    ROS_ERROR("Transfroming exception : %s",ex.what());
		    // continue;
		}
		ROS_INFO("After transformation,pose[x,y] is [%f , %f]", pose_on_map.pose.position.x, pose_on_map.pose.position.x );


		// In order to use filters to the contour points, let's read full contour points:
		ros::Time tic = ros::Time::now();
	    int contour_cnt = 0;
	    double dist;
	    std_msgs::Float32MultiArray dilated_obs_contour_array;
    	dilated_obs_contour_array.data.clear();
    	double coord_x;
    	double coord_y;
	    for (unsigned int x = 0; x < now_map.info.width; x++){
	        for (unsigned int y = 0; y < now_map.info.height; y++){
	            if (now_obs_dilated_img.at<uchar>(y,x) < 100){ // Pick out he useful points on the image

	                coord_x = double(now_map.info.origin.position.x) + double(now_map.info.resolution)*double(x+0.5);
	                coord_y = double(now_map.info.origin.position.y) + double(now_map.info.resolution)*double(y+0.5);
	                if (use_range_filter){
	                	dist = sqrt(pow(double(pose_on_map.pose.position.x - coord_x),2) + pow(double(pose_on_map.pose.position.y - coord_y),2));
	                	if (dist<range_threshold){
	                		dilated_obs_contour_array.data.push_back(coord_x);
			                dilated_obs_contour_array.data.push_back(coord_y);
			                contour_cnt++;
	                	}
	                }
	                else{
		                dilated_obs_contour_array.data.push_back(coord_x);
		                dilated_obs_contour_array.data.push_back(coord_y);
		                contour_cnt++;
	                }
	                
	            }
	        }
	    }
	    ros::Time toc = ros::Time::now();
	    ros::Duration diff = toc - tic;
	    std::cout <<"Finding contour points took: "<< diff <<"seconds" << std::endl;
	    now_dilated_obs_array = dilated_obs_contour_array;
    	now_obs_contour_num = contour_cnt;
    	ROS_INFO("There are %i contour points on the map.", contour_cnt);



		// Now we can publish all the topics we want.
        ros::Time now = ros::Time::now();

        contour_pub.publish(now_contour_array);
        grid_obstacles_array_pub.publish(now_obstacles_array);
        pcl_conversions::toPCL(now, now_obs_cloud.header.stamp);
        now_obs_cloud.header.frame_id = map_topic_name;
        obstacles_cloud_pub.publish(now_obs_cloud);



        sensor_msgs::ImagePtr dilated_map_img = cv_bridge::CvImage(std_msgs::Header(), "mono8", now_obs_dilated_img).toImageMsg();
		dilated_map_img->header.stamp = now;
		map_dilated_img_pub.publish(dilated_map_img);
		map_dilated_contour_pub.publish(now_dilated_obs_array);

		pose_pub.publish(pose_on_map);

        ROS_INFO("Now, obstacle number is %i", now_obs_num);
        ROS_INFO("Now, contour points number is %i", now_obs_contour_num);
        ros::spinOnce();

        loop_rate.sleep();
        // ++count;
    }

    ros::spin();
    return 0;
}
