//
// Created by tioeare on 5/6/23.
//
#include "read_OF.h"

void fusion::read_OF::readOpticalFlow(uint8_t data) {
    static MICOLINK_MSG_t msg;
    if (!micolink_parse_char(&msg, data)) return;
    switch (msg.msg_id) {
        case MICOLINK_MSG_ID_RANGE_SENSOR: {
            MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
            memcpy(&payload, msg.payload, msg.len);
            std::unique_lock<std::mutex> lock(OF_mutex);
            OF_data.distance = (float) payload.distance / 1000; // 距离 mm /1000
            OF_data.strength = payload.strength; // 强度
            OF_data.precision = payload.precision;   // 精度
            OF_data.tof_status = payload.tof_status; // 距离状态
            OF_data.flow_vel_x = (float) payload.flow_vel_x / 100; // 光流速度x轴 cm/s /100
            OF_data.flow_vel_y = (float) payload.flow_vel_y / 100; // 光流速度y轴 cm/s /100
            OF_data.flow_quality = payload.flow_quality;  // 光流质量
            OF_data.flow_status = payload.flow_status;   // 光流状态

            OF_data.header.seq = msg.seq;
            OF_data.header.stamp = ros::Time::now();
            OF_data.header.frame_id = "OF_data";

            OF_pub.publish(OF_data);
            break;
        }
        default:
            break;
    }
}

bool fusion::read_OF::micolink_check_sum(MICOLINK_MSG_t *msg) {
    uint8_t length = msg->len + 6;
    uint8_t temp[MICOLINK_MAX_LEN];
    uint8_t checksum = 0;
    memcpy(temp, msg, length);
    for (uint8_t i = 0; i < length; i++) {
        checksum += temp[i];
    }
    if (checksum == msg->checksum)
        return true;
    else
        return false;
}

bool fusion::read_OF::micolink_parse_char(MICOLINK_MSG_t *msg, uint8_t data) {
    switch (msg->status) {
        case 0:  // 帧头
            if (data == MICOLINK_MSG_HEAD) {
                msg->head = data;
                msg->status++;
            }
            break;
        case 1:  // 设备ID
            msg->dev_id = data;
            msg->status++;
            break;
        case 2:  // 系统ID
            msg->sys_id = data;
            msg->status++;
            break;
        case 3:  // 消息ID
            msg->msg_id = data;
            msg->status++;
            break;
        case 4:  // 包序列
            msg->seq = data;
            msg->status++;
            break;
        case 5:  // 负载长度
            msg->len = data;
            if (msg->len == 0)
                msg->status += 2;
            else if (msg->len > MICOLINK_MAX_PAYLOAD_LEN)
                msg->status = 0;
            else
                msg->status++;
            break;
        case 6:  // 数据负载接收
            msg->payload[msg->payload_cnt++] = data;
            if (msg->payload_cnt == msg->len) {
                msg->payload_cnt = 0;
                msg->status++;
            }
            break;
        case 7:  // 帧校验
            msg->checksum = data;
            msg->status = 0;
            if (micolink_check_sum(msg)) {
                return true;
            }
        default:
            msg->status = 0;
            msg->payload_cnt = 0;
            break;
    }

    return false;
}

fusion::read_OF::read_OF() : nh("~") {
//    for (int i = 0; i < 4; i++) {
//        index[11] = char(0 + i);
    fd = open(index, O_RDWR | O_NONBLOCK | O_NDELAY);   // UART_DEVICE, O_RDWR | O_NOCTTY |O_NONBLOCK
//        if (fd != -1) break;
//    }
//    std::cout << index << index[11] << std::endl;
    //如果没读取成功，则一直尝试读取
    struct termios Opt{};
    tcgetattr(fd, &Opt);
    cfsetispeed(&Opt, B115200);
    cfsetospeed(&Opt, B115200);
    Opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    Opt.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &Opt);
    tcflush(fd, TCIOFLUSH);
    p = cache;

    OF_pub = nh.advertise<data_fusion::OpticalFlow_msg>("OF_data", 10);

}

void fusion::read_OF::read_serial() {
    ros::Rate loop_rate(200);
    while (ros::ok()) {
        // 判断读取是否成功，获取得到数据的数量
        if ((nread = read(fd, p, MyBufferSize - num)) > 0) {
            num += (int) nread;
            if (num == MyBufferSize) {
                read_ok = true;
                p[nread] = '\0';
                std::copy(std::begin(cache), std::end(cache), std::begin(MyOrgBuffer));
                p = cache;
            } else {
                p += nread;
            }
        }
        // 读取成功则进入循环判断串口发送的数据是否匹配
        if (read_ok) {
            read_ok = false;
            num = 0;
            for (int i = 0; i <= MyBufferSize - 1; i++) {
                readOpticalFlow(MyOrgBuffer[i]);
            }
        }
        loop_rate.sleep();
    }
}
