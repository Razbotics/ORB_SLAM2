/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../../../include/MapPoint.h"
#include"../../../include/Converter.h"

#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point32.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace tf;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, ros::Publisher pos_pub, ros::Publisher cloud_pub);

    ORB_SLAM2::System* mpSLAM;
    //sensor_msgs::PointCloud cloud;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    
    ros::Rate loop_rate(30);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/orb_slam/pointcloud", 1);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/orb_slam/pos", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2, pos_pub, cloud_pub));


    while (ros::ok()) {
	    // Spin
	    ros::spinOnce();

	    // Sleep
	    loop_rate.sleep();
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, ros::Publisher pos_pub, ros::Publisher cloud_pub)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat pose_TCW = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());


    // CAMERA POSE

    if (pose_TCW.empty()) {
            return;
    }
    // transform into right handed camera frame
    cv::Mat rotation_World =  (cv::Mat_<float>(4,4) <<  1,  0,  0, 0,
                                                        0,  0,  1, 0,
                                                        0, -1,  0, 0,
                                                        0,  0,  0, 1);
    cv::Mat pose_TWC = pose_TCW.inv();
    pose_TWC = rotation_World * pose_TWC;

    tf::Matrix3x3 rh_cameraPose(   pose_TWC.at<float>(0,0),   pose_TWC.at<float>(0,1),   pose_TWC.at<float>(0,2),
                                   pose_TWC.at<float>(1,0),   pose_TWC.at<float>(1,1),   pose_TWC.at<float>(1,2),
                                    pose_TWC.at<float>(2,0),  pose_TWC.at<float>(2,1),  pose_TWC.at<float>(2,2));

    tf::Vector3 rh_cameraTranslation( pose_TWC.at<float>(0,3), pose_TWC.at<float>(1,3),  pose_TWC.at<float>(2,3) );



    //publish right handed, x forward, y right, z down (NED)
    cv::Mat twc_trans = pose_TWC.rowRange(0,3).col(3);

    // Publish tf transform
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(twc_trans.at<float>(0), twc_trans.at<float>(1), twc_trans.at<float>(2)));
    vector<float> r = ORB_SLAM2::Converter::toQuaternion(pose_TWC.rowRange(0,3).colRange(0,3));
    tf::Quaternion qu(r[0], r[1], r[2], r[3]);
    transform.setRotation(qu);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera")); // camera is the parent frame


    tf::Quaternion q;
    rh_cameraPose.getRotation(q);
    geometry_msgs::PoseStamped p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = "world";
    p.pose.position.x = rh_cameraTranslation[0];
    p.pose.position.y = rh_cameraTranslation[1];
    p.pose.position.z = rh_cameraTranslation[2];
    p.pose.orientation.x = q[0];
    p.pose.orientation.y = q[1];
    p.pose.orientation.z = q[2];
    p.pose.orientation.w = q[3];

    pos_pub.publish(p);

    /* POINT CLOUD
    sensor_msgs::PointCloud cloud;
    cloud.points = [];
    cloud.header.frame_id = "world";
    cv::Mat rotation_World_pc =  (cv::Mat_<float>(3,3) <<  1,  0,  0,
                                                           0,  0,  1,
                                                           0, -1,  0);
    //std::vector<geometry_msgs::Point32> geo_points;
    std::vector<ORB_SLAM2::MapPoint*> points = mpSLAM->GetTrackedMapPoints();
    //cout << points.size() << endl;
    for (std::vector<int>::size_type i = 0; i != points.size(); i++) {
	    if (points[i]) {
		    cv::Mat coords = points[i]->GetWorldPos();
                    coords = rotation_World_pc * coords;
		    geometry_msgs::Point32 pt;
		    pt.x = coords.at<float>(0);
		    pt.y = coords.at<float>(1);
		    pt.z = coords.at<float>(2);
		    cloud.points.push_back(pt);
	    } 
            
    else {
	    }
    }
    //cout << geo_points.size() << endl;
    //cloud.points = geo_points;
    //cloud_pub.publish(cloud);
    cloud_pub.publish(cloud);
  */
}
