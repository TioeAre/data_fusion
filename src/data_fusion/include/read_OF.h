//
// Created by tioeare on 5/6/23.
//

#ifndef DATA_FUSION_READ_OF_H
#define DATA_FUSION_READ_OF_H

#include <cstdint>
#include <cstring>
#include <mutex>
#include <iostream>
#include <string>
#include <chrono>
#include <ctime>
#include <cmath>
#include <cstdlib>
#include <unistd.h>  // Unix 标准函数定义
#include <sys/types.h>  // Unix/Linux系统的基本系统数据类型，含有size_t, pid_t等类型
#include <sys/stat.h>  // Unix/Linux系统定义文件状态所在的伪标准头文件
#include <fcntl.h>  // 文件控制
#include <termios.h>  // POSIX 终端控制定义
#include <pthread.h> // 线程库   需要加LIB
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/BatteryState.h>

#include "data_fusion/OpticalFlow_msg.h"

#define MICOLINK_MSG_HEAD 0xEF
#define MICOLINK_MAX_PAYLOAD_LEN 64
#define MICOLINK_MAX_LEN (MICOLINK_MAX_PAYLOAD_LEN + 7)

namespace fusion {

    class read_OF {
    public:

        data_fusion::OpticalFlow_msg OF_data{};
        std::mutex OF_mutex;

        /**
         * @brief 读取光流数据
         * @param data 从串口读到的数据
         */
        void readOpticalFlow(uint8_t data);

        void read_serial();

        read_OF();
        ~read_OF()=default;

    private:
        ros::NodeHandle nh;
        ros::Publisher OF_pub;

        int fd;
        char index[13] = "/dev/ttyUSB0";
        char MyOrgBuffer[128]{};
        char cache[128]{};  // 用于存放缓存数据
        char *p = nullptr;
        ssize_t nread{};
        int num = 0;
        bool read_ok = false;

        const int MyBufferSize = MICOLINK_MAX_LEN;
        enum {
            MICOLINK_MSG_ID_RANGE_SENSOR = 0x51,  // 测距传感器
        };
        typedef struct {
            uint8_t head;
            uint8_t dev_id;
            uint8_t sys_id;
            uint8_t msg_id;
            uint8_t seq;
            uint8_t len;
            uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN];
            uint8_t checksum;
            uint8_t status;
            uint8_t payload_cnt;
        } MICOLINK_MSG_t;
#pragma pack(1)
        typedef struct {
            uint32_t time_ms;      // 系统时间 ms
            uint32_t distance;     // 距离(mm) 最小值为10，0表示数据不可用
            uint8_t strength;      // 信号强度
            uint8_t precision;     // 精度
            uint8_t tof_status;    // 状态
            uint8_t reserved1;     // 预留
            int16_t flow_vel_x;    // 光流速度x轴
            int16_t flow_vel_y;    // 光流速度y轴
            uint8_t flow_quality;  // 光流质量
            uint8_t flow_status;   // 光流状态
            uint16_t reserved2;    // 预留
        } MICOLINK_PAYLOAD_RANGE_SENSOR_t;
#pragma pack()

        static bool micolink_parse_char(MICOLINK_MSG_t *msg, uint8_t data);

        static bool micolink_check_sum(MICOLINK_MSG_t *msg);
    };

}
#endif // DATA_FUSION_READ_OF_H
