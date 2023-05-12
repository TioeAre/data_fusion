## 数据格式

### GPS

经度，维度，高度的绝对值

gps获取到的数据的协方差(正常室外时协方差<1.5)

topic: `/gps/fix`

type: `sensor_msgs/NavSatFix`

```bash
uint8 COVARIANCE_TYPE_UNKNOWN=0
uint8 COVARIANCE_TYPE_APPROXIMATED=1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
uint8 COVARIANCE_TYPE_KNOWN=3
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
sensor_msgs/NavSatStatus status
  int8 STATUS_NO_FIX=-1
  int8 STATUS_FIX=0
  int8 STATUS_SBAS_FIX=1
  int8 STATUS_GBAS_FIX=2
  uint16 SERVICE_GPS=1
  uint16 SERVICE_GLONASS=2
  uint16 SERVICE_COMPASS=4
  uint16 SERVICE_GALILEO=8
  int8 status
  uint16 service
float64 latitude
float64 longitude
float64 altitude
float64[9] position_covariance
uint8 position_covariance_type
```

### 光流

光流x，y轴速度及竖直方向上相对地面的高度

信号强度，光流质量

```bash
# 自定义光流数据
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 distance
uint8 strength
uint8 precision
uint8 tof_status
float32 flow_vel_x
float32 flow_vel_y
uint8 flow_quality
uint8 flow_status

# 板载imu数据
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Quaternion orientation
  float64 x
  float64 y
  float64 z
  float64 w
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
  float64 x
  float64 y
  float64 z
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
  float64 x
  float64 y
  float64 z
float64[9] linear_acceleration_covariance
```

### t265

相机当前坐标(相对于开机时的坐标)及姿态四元数

坐标为相对坐标, 姿态四元数为绝对值

topic: `/camera/odom/sample`

type: `nav_msgs/Odometry`

```bash
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
```

## 实现步骤

### 1. 获取数据

t265与gps通过ros订阅话题读取，光流通过串口读取

gps数据需要与第一帧相减得到相对位移

将t265的位姿用tf转换到机体坐标系, 光流与gps同样需要做转化

光流数据还需要提前进行补偿，用imu或t265得到的位姿角度补偿光流速度

将转化后的数据再与自己的前一帧数据做对比转化, 然后发布光流, t265, gps相对于自己前一帧的tf数据

在滤波中再进行降噪和故障诊断

故障诊断中, 光流可以用光流质量, 光流状态(待测), gps用协方差, t265用协方差(0.01时不可用, 0.001时可能出现漂移) 和光流/imu(imu没有给速度, 但t265却认为移动的情况) 单独判断

### 滤波

#### 递推加权最小二乘(WRLS)

最小二乘法得到多个传感器与真实值之间误差最小的权重, 再以该权重对多传感器数据进行加权

$\hat{X}=\sum^n_{i=1}a_iZ_i$, $a_i$是$i$个传感器的加权系数, $Z_i$为每个传感器的测量值, 需满足
$$
E[\hat{x}]=X=E[\sum^n_{i=1}a_iX]+E[\sum^n_{i=1}a_iV_i]=X\sum^n_{i=1}a_i
$$
$V_i$为噪声, 均值为0, 方差为$\sigma^2$

均方误差满足
$$
E[(X-\hat{X})^2]=E[X-\sum^n_{i=1}a_1(X+V_i)]^2=\sum^n_{i=1}a^2_i\sigma^2_i
$$
构造拉格朗日函数
$$
f(a_1,...a_n,\lambda)=\sum^n_{i=1}a^2_i\sigma^2_i-\lambda(\sum^n_{i=1}-1)
$$
求得$a_i=\frac{\sigma^{-2}_{i}}{\sum^n_{i=1}\sigma^{-2}_{i}}$

#### EKF

将WRLS融合出的数据作为测量值, 无人机真实位置作为状态量, 三种传感器获取到的数据作为观测值
$$
X_{k|k-1}\&=A_{k|k-1}*X_{k-1} \\ P_{k|k-1}=A_{k|k-1}P_{k-1}A_{k|k-1}^T+RQR^T \\ K_k=P_{k|k-1}H^T_k(H_kP_{k|k-1}H^T_k+P_k)^{-1} \\ X=X_{k-1}+K_k(Z_k-H_kX_{k|k-1})
$$
Z为WRLS中融合得到的值

#### 滤波监测

对EKF输出的值进行检查，避免EKF结果过于发散

1.   室内x, y轴方向光流和imu互补滤波, 然后与EKF的数据进行对比, 相差过大时用互补滤波数据替代
2.   GPS监测, 有GPS信号时根据GPS与imu互补滤波, 相差过大时进行复位, 对轨迹进行优化
3.   室内回环检测, 鱼眼相机检测到回环的时候对目前的位姿进行优化

#### 发布数据用以监测参数

1.   光流原始数据(接收到的, 无需另外发布) `/data_fusion/OF_data/flow_vel_x(y)`

2.   imu数据 `/mavros/imu/data/angular_velocity/x(y)`

3.   光流补偿后的数据 `/data_fusion/compensated_OF/point/x(y)`

4.   融合前的t265, 光流, gps累加位移信息(**光流**需要单独积分然后发布)

     `/data_fusion/oriUAV_t265/point/x(y, z)`

     `/data_fusion/OF_sum/point/x(y, z)`

     `/data_fusion/oriUAV_gps/point/x(y, z)`

5.   用于融合的t265, 光流, gps的帧间数据

     `/data_fusion/origin_dOF/point/x(y, z)`

     `/data_fusion/origin_dt265/point/x(y, z)`

     `/data_fusion/origin_dgps/point/x(y, z)`

6.   融合后的帧间数据 `/data_fusion/fusion_displace/point/x(y, z)`

7.   融合后累加起来的位移 `/data_fusion/pose_msg/pose/position/x(y, z)`

type: `geometry_msgs::PointStamped`

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Point position
  float64 x
  float64 y
  float64 z
```

```bash
# 光流补偿效果
rqt_plot /data_fusion/OF_data/flow_vel_x /mavros/imu/data/angular_velocity/x /data_fusion/compensated_OF/point/x
rqt_plot /data_fusion/OF_data/flow_vel_y /mavros/imu/data/angular_velocity/y /data_fusion/compensated_OF/point/y

# 帧间数据融合效果
rqt_plot /data_fusion/origin_dOF/point/x /data_fusion/origin_dt265/point/x /data_fusion/origin_dgps/point/x /data_fusion/fusion_displace/point/x
rqt_plot /data_fusion/origin_dOF/point/y /data_fusion/origin_dt265/point/y /data_fusion/origin_dgps/point/y /data_fusion/fusion_displace/point/y
rqt_plot /data_fusion/origin_dOF/point/z /data_fusion/origin_dt265/point/z /data_fusion/origin_dgps/point/z /data_fusion/fusion_displace/point/z

# 融合pose效果
rqt_plot /data_fusion/oriUAV_t265/point/x /data_fusion/OF_sum/point/x /data_fusion/oriUAV_gps/point/x /mavros/vision_pose/pose/pose/position/x
rqt_plot /data_fusion/oriUAV_t265/point/y /data_fusion/OF_sum/point/y /data_fusion/oriUAV_gps/point/y /mavros/vision_pose/pose/pose/position/y
rqt_plot /data_fusion/oriUAV_t265/point/z /data_fusion/OF_sum/point/z /data_fusion/oriUAV_gps/point/z /mavros/vision_pose/pose/pose/position/z
```



## BUG

### mavros编译

```bash
CMake Error: File /home/tioeare/project/FASTLAB/mavros_ws/src/mavlink/config.h.in does not exist.
CMake Error at CMakeLists.txt:68 (configure_file):
  configure_file Problem configuring file
```

catkin_ws/src/mavlink下新建config.h.in文件

写入

```txt
#define MAVLINK_VERSION "${PROJECT_VERSION}"
```

重新编译

### mavros读不到imu等的数据

先连一下qgc地面站

## 局域网ros

```bash
export ROS_HOSTNAME=tioeare
export ROS_MASTER_URI=http://192.168.112.86:11311
```

