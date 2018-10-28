
/*The MIT License (MIT)
 *
 * Copyright (c) 2017, Scanse, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <scanse_sweep/sweep.hpp>

void publish_scan(ros::Publisher& pub, const sweep::scan& scan, const std::string& frame_id)
{
    pcl::PointCloud <pcl::PointXYZ> cloud;
    ros::Time ros_now = ros::Time::now();
    cloud.height = 1;
    cloud.width = scan.samples.size();
    cloud.points.resize(cloud.width * cloud.height);
    for (size_t idx = 0; idx < scan.samples.size(); idx++)
    {
        const sweep::sample& sample = scan.samples[idx];
        const int32_t range = sample.distance;
        const float angle = ((float)sample.angle / 1000); //millidegrees to degrees
        cloud.points[idx].x = (float)(range * cos(DEG2RAD(angle))) / 100;
        cloud.points[idx].y = (float)(range * sin(DEG2RAD(angle))) / 100;
        cloud.points[idx].z = 0.0;
    }
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.header.stamp = ros_now;
    ROS_DEBUG_NAMED(ros::this_node::getName(), "Publishing a full scan");
    pub.publish(cloud_msg);
}

int main(int argc, char** argv)
{
    // Initialize Node and handles
    ros::init(argc, argv, "sweep_node");
    ROS_INFO_NAMED(ros::this_node::getName(), "Starting sweep_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    //G et Serial Parameters
    const std::string serial_port = nhp.param(std::string("serial_port"), std::string("/dev/ttyUSB0"));
    // Get Scanner Parameters
    const int rotation_speed = nhp.param(std::string("rotation_speed"), 5);
    const int sample_rate = nhp.param(std::string("sample_rate"), 500);
    // Get frame_id Parameter
    const std::string frame_id = nhp.param(std::string("frame_id"), std::string("sweep_frame"));
    // Get scan topic
    const std::string scan_topic = nhp.param(std::string("scan_topic"), std::string("sweep_scan"));
    // Setup Publisher
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::PointCloud2>(scan_topic, 1, false);
    try
    {
        // Create Sweep Driver Object
        ROS_INFO_NAMED(ros::this_node::getName(), "Starting sweep device");
        sweep::sweep device{serial_port.c_str()};
        // Send Rotation Speed
        device.set_motor_speed(rotation_speed);
        // Send Sample Rate
        device.set_sample_rate(sample_rate);
        ROS_INFO_NAMED(ros::this_node::getName(), "Expected rotation frequency: %d (Hz)", rotation_speed);
        // Start Scan
        ROS_INFO_NAMED(ros::this_node::getName(), "Starting scan loop");
        device.start_scanning();
        while (ros::ok())
        {
            // Grab Full Scan
            const sweep::scan scan = device.get_scan();
            publish_scan(scan_pub, scan, frame_id);
            ros::spinOnce();
        }
        // Stop Scanning & Destroy Driver
        ROS_INFO_NAMED(ros::this_node::getName(), "Closing sweep device");
        device.stop_scanning();
    }
    catch (const sweep::device_error& error)
    {
        ROS_ERROR_NAMED(ros::this_node::getName(), "Sweep device error: %s", error.what());
    }
    return 0;
}
