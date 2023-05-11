//
// Created by tioeare on 5/6/23.
//
#include "get_data.h"

fusion::Data::Data() : nh("~"), spinner(0), listener(buffer), pose_filter(0.01) {
    read_param();
    t265_pose_sub = new message_filters::Subscriber<nav_msgs::Odometry>(nh, t265_pose_sub_topic, queueSize);
    OF_sub = new message_filters::Subscriber<data_fusion::OpticalFlow_msg>(nh, OF_sub_topic, queueSize);
    imu_sub = new message_filters::Subscriber<sensor_msgs::Imu>(nh, imu_sub_topic, queueSize);
    syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize),
                                                                               *t265_pose_sub, *OF_sub, *imu_sub);
    syncApproximate->registerCallback(boost::bind(&fusion::Data::call_back, this, _1, _2, _3));
    gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(gps_sub_topic, queueSize,
                                                   boost::bind(&fusion::Data::gps_call_back, this, _1));
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>(pose_pub_topic, queueSize);

    compensated_OF_pub = nh.advertise<geometry_msgs::PointStamped>(compensated_OF_pub_topic, queueSize);
    OF_sum_pub = nh.advertise<geometry_msgs::PointStamped>(OF_sum_pub_topic, queueSize);
    origin_dOF_pub = nh.advertise<geometry_msgs::PointStamped>(origin_dOF_pub_topic, queueSize);
    origin_dt265_pub = nh.advertise<geometry_msgs::PointStamped>(origin_dt265_pub_topic, queueSize);
    origin_dgps_pub = nh.advertise<geometry_msgs::PointStamped>(origin_dgps_pub_topic, queueSize);
    fusion_displace_pub = nh.advertise<geometry_msgs::PointStamped>(fusion_displace_pub_topic, queueSize);
    oriUAV_t265_pub = nh.advertise<geometry_msgs::PointStamped>(oriUAV_t265_pub_topic, queueSize);
    oriUAV_gps_pub = nh.advertise<geometry_msgs::PointStamped>(oriUAV_gps_pub_topic, queueSize);
}

void fusion::Data::pub_data() {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = tf_t265.header;
    pose_msg.pose.position.x = pub_pose[0];
    pose_msg.pose.position.y = pub_pose[1];
    pose_msg.pose.position.z = pub_pose[2];
    pose_msg.pose.orientation.x = orientation[0];
    pose_msg.pose.orientation.y = orientation[1];
    pose_msg.pose.orientation.z = orientation[2];
    pose_msg.pose.orientation.w = orientation[3];
    compensated_OF_pub.publish(compensated_OF_pub_msg);
    OF_sum_pub.publish(OF_UAV_point);
    origin_dOF_pub.publish(origin_dOF_pub_msg);
    origin_dt265_pub.publish(origin_dt265_pub_msg);
    origin_dgps_pub.publish(origin_dgps_pub_msg);
    fusion_displace_pub.publish(fusion_displace_pub_msg);
    oriUAV_t265_pub.publish(t265_UAV_point);
    oriUAV_gps_pub.publish(gps_UAV_point);
    pose_pub.publish(pose_msg);
}

void fusion::Data::gps_call_back(const sensor_msgs::NavSatFix::ConstPtr &gps_msg) {
    std::unique_lock<std::mutex> tf_lock(tf_mutex);
    tf_gps.header = gps_msg->header;
    broadcast_tf();
    transform_tf_to_UAV(gps_msg);
    if (if_first_gps) {
        if_first_gps = false;
        return;
    }
    gps_dpose[0] = gps_UAV_point.point.x - gps_UAV_last_point.point.x;
    gps_dpose[1] = gps_UAV_point.point.y - gps_UAV_last_point.point.y;
    gps_dpose[2] = gps_UAV_point.point.z - gps_UAV_last_point.point.z;

    origin_dgps_pub_msg.header = gps_UAV_point.header;
    origin_dgps_pub_msg.point.x = gps_dpose[0];
    origin_dgps_pub_msg.point.y = gps_dpose[1];
    origin_dgps_pub_msg.point.z = gps_dpose[2];

    gps_UAV_last_point = gps_UAV_point;
    if (vel_x.empty()) return;
    pose_gps_x.push_back({t265_dpose[0], OF_dpose[0], gps_dpose[0]});
    pose_gps_y.push_back({t265_dpose[1], OF_dpose[1], gps_dpose[1]});
    pose_gps_z.push_back({t265_dpose[2], OF_dpose[2], gps_dpose[2]});

    vel_gps_x.push_back({vel_x.back()[0], vel_x.back()[1], 0});
    vel_gps_y.push_back({vel_y.back()[0], vel_y.back()[1], 0});
    vel_gps_z.push_back({vel_z.back()[0], 0, 0});
    state_gps.push_back({state.back()[0], state.back()[1], gps_msg->position_covariance[8]});
    if (pose_gps_x.size() >= 50) {
        all_solve(pose_gps_x, pose_gps_y, pose_gps_z, vel_gps_x, vel_gps_y, vel_gps_z, state_gps);
        pose_gps_x.pop_front();
        pose_gps_y.pop_front();
        pose_gps_z.pop_front();
        vel_gps_x.pop_front();
        vel_gps_y.pop_front();
        vel_gps_z.pop_front();
        state_gps.pop_front();
    }
}

void fusion::Data::call_back(const nav_msgs::Odometry::ConstPtr &t265_pose_msg,
                             const data_fusion::OpticalFlow_msg::ConstPtr &OF_msg,
                             const sensor_msgs::Imu::ConstPtr &imu_msg) {
    std::unique_lock<std::mutex> tf_lock(tf_mutex);
    tf_t265.header = t265_pose_msg->header;
    tf_OF.header = OF_msg->header;
    broadcast_tf();
    transform_tf_to_UAV(t265_pose_msg, OF_msg, imu_msg);

    t265_dpose[0] = t265_UAV_point.point.x - t265_UAV_last_point.point.x;
    t265_dpose[1] = t265_UAV_point.point.y - t265_UAV_last_point.point.y;
    t265_dpose[2] = t265_UAV_point.point.z - t265_UAV_last_point.point.z;

    origin_dt265_pub_msg.header = t265_UAV_point.header;
    origin_dt265_pub_msg.point.x = t265_dpose[0];
    origin_dt265_pub_msg.point.y = t265_dpose[1];
    origin_dt265_pub_msg.point.z = t265_dpose[2];

    OF_dpose[0] = OF_UAV_point.point.x - OF_UAV_last_point.point.x;
    OF_dpose[1] = OF_UAV_point.point.y - OF_UAV_last_point.point.y;
    OF_dpose[2] = OF_UAV_point.point.z - OF_UAV_last_point.point.z;

    origin_dOF_pub_msg.header = OF_UAV_point.header;
    origin_dOF_pub_msg.point.x = OF_dpose[0];
    origin_dOF_pub_msg.point.y = OF_dpose[1];
    origin_dOF_pub_msg.point.z = OF_dpose[2];

    t265_dvel[0] = t265_pose_msg->twist.twist.linear.x - t265_last_speed[0];
    t265_dvel[1] = t265_pose_msg->twist.twist.linear.y - t265_last_speed[1];
    t265_dvel[2] = t265_pose_msg->twist.twist.linear.z - t265_last_speed[2];
    OF_dvel[0] = OF_msg->distance * compensated_OF_pub_msg.point.x - OF_last_speed[0];
    OF_dvel[1] = OF_msg->distance * compensated_OF_pub_msg.point.y - OF_last_speed[1];

    pose_x.push_back({t265_dpose[0], OF_dpose[0], 0});
    pose_y.push_back({t265_dpose[1], OF_dpose[1], 0});
    pose_z.push_back({t265_dpose[2], OF_dpose[2], 0});
    vel_x.push_back({t265_pose_msg->twist.twist.linear.x, OF_msg->distance * compensated_OF_pub_msg.point.x, 0});
    vel_y.push_back({t265_pose_msg->twist.twist.linear.y, OF_msg->distance * compensated_OF_pub_msg.point.y, 0});
    vel_z.push_back({t265_pose_msg->twist.twist.linear.z, 0, 0});
    state.push_back({t265_pose_msg->pose.covariance[35], (double)OF_msg->strength, 0});
    if (pose_x.size() >= 50) {
        all_solve(pose_x, pose_y, pose_z, vel_x, vel_y, vel_z, state);
        pose_x.pop_front();
        pose_y.pop_front();
        pose_z.pop_front();
        vel_x.pop_front();
        vel_y.pop_front();
        vel_z.pop_front();
        state.pop_front();
    }
    t265_UAV_last_point = t265_UAV_point;
    OF_UAV_last_point = OF_UAV_point;
    t265_last_speed[0] = t265_pose_msg->twist.twist.linear.x;
    t265_last_speed[1] = t265_pose_msg->twist.twist.linear.y;
    t265_last_speed[2] = t265_pose_msg->twist.twist.linear.z;
    OF_last_speed[0] = OF_msg->distance * compensated_OF_pub_msg.point.x;
    OF_last_speed[1] = OF_msg->distance * compensated_OF_pub_msg.point.y;
}

void fusion::Data::broadcast_tf() {
    tf_t265.header.frame_id = "uav_base";
    tf_t265.child_frame_id = "t265";
    tf_OF.header.frame_id = "uav_base";
    tf_OF.child_frame_id = "OF";
    tf_gps.header.frame_id = "uav_base";
    tf_gps.child_frame_id = "fusion_gps";
    tf_t265.transform.translation.x = t2652UAV_pose[0];
    tf_t265.transform.translation.y = t2652UAV_pose[1];
    tf_t265.transform.translation.z = t2652UAV_pose[2];
    tf_t265.transform.rotation.x = t2652UAV_rotation[0];
    tf_t265.transform.rotation.y = t2652UAV_rotation[1];
    tf_t265.transform.rotation.z = t2652UAV_rotation[2];
    tf_t265.transform.rotation.w = t2652UAV_rotation[3];
    broadcaster.sendTransform(tf_t265);

    tf_OF.transform.translation.x = OF2UAV_pose[0];
    tf_OF.transform.translation.y = OF2UAV_pose[1];
    tf_OF.transform.translation.z = OF2UAV_pose[2];
    tf_OF.transform.rotation.x = OF2UAV_rotation[0];
    tf_OF.transform.rotation.y = OF2UAV_rotation[1];
    tf_OF.transform.rotation.z = OF2UAV_rotation[2];
    tf_OF.transform.rotation.w = OF2UAV_rotation[3];
    broadcaster.sendTransform(tf_OF);

    tf_gps.transform.translation.x = gps2UAV_pose[0];
    tf_gps.transform.translation.y = gps2UAV_pose[1];
    tf_gps.transform.translation.z = gps2UAV_pose[2];
    tf_gps.transform.rotation.x = gps2UAV_rotation[0];
    tf_gps.transform.rotation.y = gps2UAV_rotation[1];
    tf_gps.transform.rotation.z = gps2UAV_rotation[2];
    tf_gps.transform.rotation.w = gps2UAV_rotation[3];
    broadcaster.sendTransform(tf_gps);
}

void fusion::Data::transform_tf_to_UAV(const nav_msgs::Odometry::ConstPtr &t265_pose_msg,
                                       const data_fusion::OpticalFlow_msg::ConstPtr &OF_msg,
                                       const sensor_msgs::Imu::ConstPtr &imu_msg) {
    orientation[0] = t265_pose_msg->pose.pose.orientation.x;
    orientation[1] = t265_pose_msg->pose.pose.orientation.y;
    orientation[2] = t265_pose_msg->pose.pose.orientation.z;
    orientation[3] = t265_pose_msg->pose.pose.orientation.w;
    t265_UAV_point.header.seq = t265_pose_msg->header.seq;
    OF_UAV_point.header.seq = OF_msg->header.seq;
    t265_UAV_point.header.stamp = t265_pose_msg->header.stamp;
    OF_UAV_point.header.stamp = OF_msg->header.stamp;

    t265_point.header.seq = t265_UAV_point.header.seq;
    t265_point.header.stamp = t265_UAV_point.header.stamp;
    t265_point.header.frame_id = "t265";
    t265_point.point.x = t265_pose_msg->pose.pose.position.x;
    t265_point.point.y = t265_pose_msg->pose.pose.position.y;
    t265_point.point.z = t265_pose_msg->pose.pose.position.z;
    // 得到t265在机体坐标下目前测量的位置
    t265_UAV_point = buffer.transform(t265_point, "uav_base", ros::Duration(0.1));
    // 计算补偿后的光流速度
    compensated_OF_pub_msg.header = OF_msg->header;
    compensated_OF_pub_msg.point.y = OF_msg->flow_vel_x - OF_x_scale * imu_msg->angular_velocity.x;
    compensated_OF_pub_msg.point.x = OF_msg->flow_vel_y - OF_y_scale * imu_msg->angular_velocity.y;
    compensated_OF_pub_msg.point.z = 0;

    // 补偿光流速度并积分出光流测出的位移
    dtime = (OF_msg->header.stamp - OF_last_point.header.stamp).toSec();
    double of_x = OF_msg->distance * dtime * compensated_OF_pub_msg.point.x;
    double of_y = OF_msg->distance * dtime * compensated_OF_pub_msg.point.y;
    OF_point.header = OF_UAV_point.header;
    OF_point.header.frame_id = "OF";
    OF_point.point.x = of_x + OF_last_point.point.x;
    OF_point.point.y = of_y + OF_last_point.point.y;
    OF_point.point.z = OF_msg->distance;
    // 得到光流在机体坐标下目前测量的位置
    OF_UAV_point = buffer.transform(OF_point, "uav_base", ros::Duration(0.1));

    OF_last_point = OF_point;
}

void fusion::Data::transform_tf_to_UAV(const sensor_msgs::NavSatFix::ConstPtr &gps_msg) {
    gps_UAV_point.header.seq = gps_msg->header.seq;
    gps_UAV_point.header.stamp = gps_msg->header.stamp;

    gps_point.header.seq = gps_UAV_point.header.seq;
    gps_point.header.stamp = gps_UAV_point.header.stamp;
    gps_point.header.frame_id = "fusion_gps";
    if (if_first_gps) {
        double radLat1 = gps_msg->latitude * 3.1415926 / 180.0;
        double radLat2 = gps_msg->longitude * 3.1415926 / 180.0;
        gps_point.point.x =
            2 * asin(sqrt(pow(sin(radLat1 / 2), 2) + cos(0) * cos(radLat1) * pow(sin(0 / 2), 2))) * 63781370;
        gps_point.point.y =
            2 * asin(sqrt(pow(sin(0 / 2), 2) + cos(radLat1) * cos(radLat1) * pow(sin(radLat2 / 2), 2))) * 63781370;
        gps_point.point.z = gps_msg->altitude;
    } else {
        double radLat1 = gps_msg->latitude * 3.1415926 / 180.0;
        double radLat2 = gps_msg->longitude * 3.1415926 / 180.0;
        gps_point.point.x =
            2 * asin(sqrt(pow(sin(radLat1 / 2), 2) + cos(0) * cos(radLat1) * pow(sin(0 / 2), 2))) * 63781370 -
            gps_last_point.point.x;
        gps_point.point.y =
            2 * asin(sqrt(pow(sin(0 / 2), 2) + cos(radLat1) * cos(radLat1) * pow(sin(radLat2 / 2), 2))) * 63781370 -
            gps_last_point.point.y;
        gps_point.point.z = gps_msg->altitude - gps_last_point.point.z;
    }
    // 得到gps在机体坐标下目前测量的位置
    gps_UAV_point = buffer.transform(gps_point, "uav_base", ros::Duration(0.1));

    dtime = (gps_point.header.stamp - gps_last_point.header.stamp).toSec();

    gps_last_point = gps_point;
}

std::vector<float> fusion::Data::solve_ceres(std::deque<std::vector<double>> data, std::deque<double> y) {
    double abc[3] = {1. / 3, 1. / 3, 1. / 3};
    ceres::Problem problem;
    for (int i = 0; i < data.size(); i++) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<COST, 1, 3>(new COST(data[i][0], data[i][1], data[i][2], y[i])), nullptr,
            abc);
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::vector<float> result = {float(abc[0]), float(abc[1]), float(abc[2])};
    return result;
}

std::deque<double> fusion::Data::calY(std::deque<std::vector<double>> data, std::deque<std::vector<double>> state1,
                                      bool if_vz) {
    std::deque<double> y;
    for (int i = 0; i < data.size(); i++) {
        std::vector<double> sum;
        if (state1[i][0] < 0.01 && data[i][0] != 0 && !if_vz && abs(data[i][1]) > 0.03 && data[i][0] < 5) {
            sum.push_back(data[i][0]);
        } else if (state1[i][1] > 120 && data[i][1] != 0 && data[i][1] < 5) {  // 光流强度
            sum.push_back(data[i][1]);
        } else if (state1[i][2] < 15 && data[i][2] != 0 && data[i][2] < 5) {  // gps协方差
            sum.push_back(data[i][2]);
        }
        double y1 = 0;
        for (auto &&one : sum) {
            y1 += one;
        }
        y.push_back(y1 / (double)sum.size());
    }
    return y;
}

void fusion::Data::all_solve(const std::deque<std::vector<double>> &pose_x1,
                             const std::deque<std::vector<double>> &pose_y1,
                             const std::deque<std::vector<double>> &pose_z1,
                             const std::deque<std::vector<double>> &vel_x1,
                             const std::deque<std::vector<double>> &vel_y1,
                             const std::deque<std::vector<double>> &vel_z1,
                             const std::deque<std::vector<double>> &state2) {
    aPx = solve_ceres(pose_x1, calY(pose_x1, state2));
    aPy = solve_ceres(pose_y1, calY(pose_y1, state2));
    aPz = solve_ceres(pose_z1, calY(pose_z1, state2));
    aVx = solve_ceres(vel_x1, calY(vel_x1, state2));
    aVy = solve_ceres(vel_y1, calY(vel_y1, state2));
    aVz = solve_ceres(vel_z1, calY(vel_z1, state2, true));

    std::vector<float> pose_date = {
        (float)t265_dpose[0], (float)t265_dpose[1], (float)t265_dpose[2], (float)OF_dpose[0],  (float)OF_dpose[1],
        (float)OF_dpose[2],   (float)gps_dpose[0],  (float)gps_dpose[1],  (float)gps_dpose[2], (float)t265_dvel[0],
        (float)t265_dvel[1],  (float)t265_dvel[2],  (float)OF_dvel[0],    (float)OF_dvel[1]};
    Eigen::MatrixXf control_H;
    control_H.resize(6, 14);

    control_H << aPx[0], 0, 0, aPx[1], 0, 0, aPx[2], 0, 0, 0, 0, 0, 0, 0, 0, aPy[0], 0, 0, aPy[1], 0, 0, aPy[2], 0, 0,
        0, 0, 0, 0, 0, 0, aPz[0], 0, 0, aPz[1], 0, 0, aPz[2], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, aVx[0], 0, 0,
        aVx[1], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, aVy[0], 0, 0, aVy[1], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, aVz[0], 0, 0;

    pose_filter.update(pose_date, (float)dtime, control_H, after_filter_pose);

    fusion_displace_pub_msg.header = tf_t265.header;
    fusion_displace_pub_msg.point.x = after_filter_pose[0];
    fusion_displace_pub_msg.point.y = after_filter_pose[1];
    fusion_displace_pub_msg.point.z = after_filter_pose[2];

    pub_pose[0] += after_filter_pose[0];  // = t265_UAV_last_point.point.x
    pub_pose[1] += after_filter_pose[1];  //= t265_UAV_last_point.point.y
    pub_pose[2] += after_filter_pose[2];  // = t265_UAV_last_point.point.z

    pub_data();
}

void fusion::Data::start() {
    spinner.start();
    while (ros::ok()) {
        ros::spinOnce();
        ros::spinOnce();
    }
    spinner.stop();
}

void fusion::Data::read_param() {
    if (ros::param::get("~t265_pose_sub_topic", t265_pose_sub_topic))
        ;
    else
        t265_pose_sub_topic = "/camera/odom/sample";
    if (ros::param::get("~gps_sub_topic", gps_sub_topic))
        ;
    else
        gps_sub_topic = "/gps/fix";
    if (ros::param::get("~OF_sub_topic", OF_sub_topic))
        ;
    else
        OF_sub_topic = "/data_fusion/OF_data";
    if (ros::param::get("~queueSize", queueSize))
        ;
    else
        queueSize = 10;
    if (ros::param::get("~imu_sub_topic", imu_sub_topic))
        ;
    else
        imu_sub_topic = "/mavros/imu/data";

    pose_pub_topic = "/mavros/vision_pose/pose";
    compensated_OF_pub_topic = "/data_fusion/compensated_OF";
    OF_sum_pub_topic = "/data_fusion/OF_sum";
    origin_dOF_pub_topic = "/data_fusion/origin_dOF";
    origin_dt265_pub_topic = "/data_fusion/origin_dt265";
    origin_dgps_pub_topic = "/data_fusion/origin_dgps";
    fusion_displace_pub_topic = "/data_fusion/fusion_displace";
    oriUAV_t265_pub_topic = "/data_fusion/oriUAV_t265";
    oriUAV_gps_pub_topic = "/data_fusion/oriUAV_gps";

    tf_t265.header.frame_id = "uav_base";
    tf_t265.child_frame_id = "t265";
    tf_OF.header.frame_id = "uav_base";
    tf_OF.child_frame_id = "OF";
    tf_gps.header.frame_id = "uav_base";
    tf_gps.child_frame_id = "fusion_gps";

    t265_UAV_point.header.frame_id = "uav_base";
    OF_UAV_point.header.frame_id = "uav_base";
    gps_UAV_point.header.frame_id = "uav_base";

    OF_last_point.header.seq = 0;
    gps_last_point.header.seq = 0;
    t265_UAV_last_point.header.seq = 0;
    OF_UAV_last_point.header.seq = 0;
    gps_UAV_last_point.header.seq = 0;
    OF_last_point.header.frame_id = "OF";
    gps_last_point.header.frame_id = "fusion_gps";
    t265_UAV_last_point.header.frame_id = "uav_base";
    OF_UAV_last_point.header.frame_id = "uav_base";
    gps_UAV_last_point.header.frame_id = "uav_base";
    OF_last_point.header.stamp = ros::Time::now();
    gps_last_point.header.stamp = ros::Time::now();
    t265_UAV_last_point.header.stamp = ros::Time::now();
    OF_UAV_last_point.header.stamp = ros::Time::now();
    gps_UAV_last_point.header.stamp = ros::Time::now();

    OF_last_point.point.x = 0;
    OF_last_point.point.y = 0;
    OF_last_point.point.z = 0;
    gps_last_point.point.x = 0;
    gps_last_point.point.y = 0;
    gps_last_point.point.z = 0;
    t265_UAV_last_point.point.x = 0;
    t265_UAV_last_point.point.y = 0;
    t265_UAV_last_point.point.z = 0;
    OF_UAV_last_point.point.x = 0;
    OF_UAV_last_point.point.y = 0;
    OF_UAV_last_point.point.z = 0;
}
