#include "ImageBuffer.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>

namespace ORB_SLAM
{

void ImageBuffer::Run()
{ 
    // Our count var
    count = 0;
    // Subscribe to the camera topic
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageBuffer::AddImage, this);
    // Hand this off to ros
    ros::spin();
}

void ImageBuffer::AddImage(const sensor_msgs::ImageConstPtr& msg)
{
    boost::mutex::scoped_lock lock(iMutex);
    // Increment count
    count++;
    // Only accept every 10 images
    if(count!=5)
        return;
    else
        count =0;
    // Copy the ros image message to cv::Mat. Convert to grayscale if it is a color image.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // Add to local buffer
    images.push_back(cv_ptr);
}

bool ImageBuffer::CheckNewImages()
{
    boost::mutex::scoped_lock lock(iMutex);
    return !images.empty();
}

cv_bridge::CvImageConstPtr ImageBuffer::GrabImage()
{
    boost::mutex::scoped_lock lock(iMutex);
    cv_bridge::CvImageConstPtr temp = images.front();
    images.pop_front();
    return temp;
}

int ImageBuffer::GetBufferCount()
{
    boost::mutex::scoped_lock lock(iMutex);
    return images.size();
}

}

