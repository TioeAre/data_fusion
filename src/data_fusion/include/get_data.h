//
// Created by tioeare on 5/6/23.
//

#ifndef DATA_FUSION_GET_DATA_H
#define DATA_FUSION_GET_DATA_H

#include <ceres/ceres.h>
#include <ceres/internal/config.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "filter.h"
#include "read_OF.h"

#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_INFO

namespace fusion {

class Data {
public:
    // 读取光流
    read_OF OF_data;

    double t265_dpose[3] = {0.0, 0.0, 0.0};
    double OF_dpose[3] = {0.0, 0.0, 0.0};
    double gps_dpose[3] = {0.0, 0.0, 0.0};
    double t265_dvel[3] = {0.0, 0.0, 0.0};
    double OF_dvel[3] = {0.0, 0.0, 0.0};
    std::vector<double> after_filter_pose = {0., 0., 0.};
    std::vector<double> pub_pose = {0., 0., 0.};
    fusion::filter pose_filter;

public:
    Data();

    ~Data() = default;

    void start();

private:
    float OF_x_scale = 1.1;  /// TODO:调参补偿光流x轴速度
    float OF_y_scale = -2;   /// TODO:调参补偿光流y轴速度

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner;
    tf2_ros::StaticTransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tf_t265;
    geometry_msgs::TransformStamped tf_OF;
    geometry_msgs::TransformStamped tf_gps;

    geometry_msgs::PointStamped t265_UAV_point, OF_UAV_point, gps_UAV_point;
    geometry_msgs::PointStamped t265_point, OF_point, gps_point;
    geometry_msgs::PointStamped t265_UAV_last_point, OF_last_point, gps_last_point;
    geometry_msgs::PointStamped OF_UAV_last_point, gps_UAV_last_point;

    bool if_first_gps = true;
    float t2652UAV_pose[3] = {0.065, 0.0, -0.04};  // TODO: 调整相对飞控位移
    float OF2UAV_pose[3] = {0.0, 0.0, -0.08};
    float gps2UAV_pose[3] = {0.0, 0.0, 0.0};
    float t2652UAV_rotation[4] = {0.0, 0.0, 0.0, 1.0};
    float OF2UAV_rotation[4] = {0.0, 0.0, 0.0, 1.0};
    float gps2UAV_rotation[4] = {0.0, 0.0, 0.0, 1.0};
    std::mutex tf_mutex;

    // 订阅的话题名称
    std::string t265_pose_sub_topic, gps_sub_topic, OF_sub_topic, imu_sub_topic;
    std::string pose_pub_topic;

    // 订阅的话题队列长度
    int queueSize = 10;

    // 订阅话题变量
    typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, data_fusion::OpticalFlow_msg,
                                                            sensor_msgs::Imu>
        TimeSyncPolicy;
    message_filters::Subscriber<nav_msgs::Odometry> *t265_pose_sub;
    message_filters::Subscriber<data_fusion::OpticalFlow_msg> *OF_sub;
    message_filters::Subscriber<sensor_msgs::Imu> *imu_sub;
    message_filters::Synchronizer<TimeSyncPolicy> *syncTime;
    ros::Subscriber gps_sub;
    ros::Publisher pose_pub;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;

    std::deque<std::vector<double>> pose_x, pose_y, pose_z;  // 位移变化, 每一时刻包含t265, 光流和gps三个值
    std::deque<std::vector<double>> vel_x, vel_y, vel_z;

    std::deque<std::vector<double>> pose_gps_x, pose_gps_y,
        pose_gps_z;  // 有gps信号时的位移变化, 每一时刻包含t265, 光流和gps三个值
    std::deque<std::vector<double>> vel_gps_x, vel_gps_y, vel_gps_z;
    std::deque<std::vector<double>> state;
    std::deque<std::vector<double>> state_gps;
    std::vector<float> aPx, aPy, aPz;  // 位移系数
    std::vector<float> aVx, aVy, aVz;  // 速度系数
    double t265_last_speed[3] = {0.0, 0.0, 0.0};
    double OF_last_speed[2] = {0.0, 0.0};

    double dtime = 0;  // 时间间隔

    // ceres损失函数
    struct COST {
        COST(double x1, double x2, double x3, double y) : _x1(x1), _x2(x2), _x3(x3), _y(y) {}

        const double _x1, _x2, _x3, _y;

        template <typename T>
        bool operator()(const T *const abc, T *residual) const {
            residual[0] = T(_y) - (abc[0] * T(_x1) + abc[1] * T(_x2) + abc[2] * T(_x3));
            return true;
        }
    };

    ros::Publisher compensated_OF_pub;
    ros::Publisher OF_sum_pub, origin_dOF_pub, origin_dt265_pub, origin_dgps_pub;
    ros::Publisher fusion_displace_pub, oriUAV_t265_pub, oriUAV_gps_pub;

    std::string compensated_OF_pub_topic, OF_sum_pub_topic, origin_dOF_pub_topic, origin_dt265_pub_topic,
        origin_dgps_pub_topic, fusion_displace_pub_topic, oriUAV_t265_pub_topic, oriUAV_gps_pub_topic;

    geometry_msgs::PointStamped compensated_OF_pub_msg, origin_dOF_pub_msg, origin_dt265_pub_msg, origin_dgps_pub_msg,
        fusion_displace_pub_msg;

    double orientation[4] = {0., 0., 0., 0.};

private:
    /**
     * @brief 读取参数
     */
    void read_param();

    /**
     * @brief 回调函数
     * @param t265_pose_msg t265的位姿信息
     * @param OF_msg 光流速度, 高度等
     */
    void call_back(const nav_msgs::Odometry::ConstPtr &t265_pose_msg,
                   const data_fusion::OpticalFlow_msg::ConstPtr &OF_msg, const sensor_msgs::Imu::ConstPtr &imu_msg);

    /**
     * @brief gps回调函数
     * @param gps_msg 订阅的gps位置数据
     */
    void gps_call_back(const sensor_msgs::NavSatFix::ConstPtr &gps_msg);

    void pub_data();

    void broadcast_tf();

    void transform_tf_to_UAV(const nav_msgs::Odometry::ConstPtr &t265_pose_msg,
                             const data_fusion::OpticalFlow_msg::ConstPtr &OF_msg,
                             const sensor_msgs::Imu::ConstPtr &imu_msg);

    void transform_tf_to_UAV(const sensor_msgs::NavSatFix::ConstPtr &gps_msg);

    std::vector<float> solve_ceres(std::deque<std::vector<double>> data, std::deque<double> y);

    std::deque<double> calY(std::deque<std::vector<double>> data, std::deque<std::vector<double>> state1,
                            bool if_vz = false);

    void all_solve(const std::deque<std::vector<double>> &pose_x1, const std::deque<std::vector<double>> &pose_y1,
                   const std::deque<std::vector<double>> &pose_z1, const std::deque<std::vector<double>> &vel_x1,
                   const std::deque<std::vector<double>> &vel_y1, const std::deque<std::vector<double>> &vel_z1,
                   const std::deque<std::vector<double>> &state2);
};
}  // namespace fusion

#endif  // DATA_FUSION_GET_DATA_H
