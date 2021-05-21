/**
 * @file
 * @brief Example node to publish point clouds and imu topics
 */

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <memory>

#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/ros.h"

using PacketMsg = ouster_ros::PacketMsg;
using Cloud = ouster_ros::Cloud;
using Point = ouster_ros::Point;
namespace sensor = ouster::sensor;

// template <class ContainerAllocator>

typedef std::vector<uint8_t, std::allocator<void>::rebind<uint8_t>::other>::const_iterator pm_buf_iter;
typedef std::vector<uint8_t, std::allocator<void>::rebind<uint8_t>::other> pm_buf_type;

int main(int argc, char** argv) {
    ros::init(argc, argv, "os_cloud_node");
    ros::NodeHandle nh("~");

    auto tf_prefix = nh.param("tf_prefix", std::string{});
    if (!tf_prefix.empty() && tf_prefix.back() != '/') tf_prefix.append("/");
    auto sensor_frame = tf_prefix + "os_sensor";
    auto imu_frame = tf_prefix + "os_imu";
    auto lidar_frame = tf_prefix + "os_lidar";

    ouster_ros::OSConfigSrv cfg{};
    auto client = nh.serviceClient<ouster_ros::OSConfigSrv>("os_config");
    client.waitForExistence();
    if (!client.call(cfg)) {
        ROS_ERROR("Calling config service failed");
        return EXIT_FAILURE;
    }

    auto info = sensor::parse_metadata(cfg.response.metadata);
    auto info_sub = info; 
    
    uint32_t H = info.format.pixels_per_column;
    uint32_t W = info.format.columns_per_frame;//1024

     

    auto pf = sensor::get_format(info);

    uint32_t W_sub = pf.columns_per_packet; //16:  360 degree divided by 22.5 degree is 16 parts. 
    // uint32_t H_sub = info.format.pixels_per_column;
    // info_sub.format.columns_per_frame = W_sub; 

    auto lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("points", 10);
    auto subCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("sub_points", 10);
    auto imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);

    auto xyz_lut = ouster::make_xyz_lut(info);
    

    Cloud cloud{W, H};
    ouster::LidarScan ls{W, H};
    ouster::ScanBatcher batch(W, pf);



    auto lidar_handler = [&](const PacketMsg& pm) mutable {
        
        //for whole cloud
        
        if (batch(pm.buf.data(), ls)) {//TODO: packetmsg to lidar scan
            auto h = std::find_if(
                ls.headers.begin(), ls.headers.end(), [](const auto& h) {
                    return h.timestamp != std::chrono::nanoseconds{0};
                });
            if (h != ls.headers.end()) {
                scan_to_cloud(xyz_lut, h->timestamp, ls, cloud);
                lidar_pub.publish(ouster_ros::cloud_to_cloud_msg(
                    cloud, h->timestamp, sensor_frame));
            }
        }
        


        //for sub cloud
        ouster::LidarScan ls_sub{W_sub, H}; 
        ouster::ScanBatcher batch_sub(W_sub, pf);
        Cloud cloud_sub{W_sub, H};
        // convert the package msg into laser scan
        if (batch_sub(pm.buf.data(), ls_sub, true)) {

            auto xyz_lut_sub = ouster::make_xyz_lut(W_sub, info_sub.format.pixels_per_column,
                    sensor::range_unit, info_sub.lidar_origin_to_beam_origin_mm,
                    info_sub.lidar_to_sensor_transform, info_sub.beam_azimuth_angles,
                    info_sub.beam_altitude_angles, ls_sub.measure_id, W);

            auto h = std::find_if(
                ls_sub.headers.begin(), ls_sub.headers.end(), [](const auto& h) {
                    return h.timestamp != std::chrono::nanoseconds{0};
                });
            if (h != ls_sub.headers.end()) {
                scan_to_cloud(xyz_lut_sub, h->timestamp, ls_sub, cloud_sub);
                subCloud_pub.publish(ouster_ros::cloud_to_cloud_msg(
                    cloud_sub, h->timestamp, sensor_frame));
            }
        }
           

        
    };

    auto imu_handler = [&](const PacketMsg& p) {
        imu_pub.publish(ouster_ros::packet_to_imu_msg(p, imu_frame, pf));
    };

    auto lidar_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "lidar_packets", 2048, lidar_handler);
    auto imu_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "imu_packets", 100, imu_handler);


    // publish transforms
    tf2_ros::StaticTransformBroadcaster tf_bcast{};

    tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
        info.imu_to_sensor_transform, sensor_frame, imu_frame));

    tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
        info.lidar_to_sensor_transform, sensor_frame, lidar_frame));

    ros::spin();

    return EXIT_SUCCESS;
}
