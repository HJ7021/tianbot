#include "tianracer_navigation/Pure_Pursuit.h"
#define PI 3.1415927
using namespace std;
PurePursuit::PurePursuit() {
    lap_count_ = 0;
    Pure_Pursuit_out = 0;
    // 创建私有的节点句柄
    ros::NodeHandle pravite_nh("~");
    pravite_nh.param("base_shape_L", base_shape_L, 0.3); // 机器人轴距
    pravite_nh.param("reference_v", reference_v, 2.7);// 目标速度
    pravite_nh.param("Lfw", Lfw, 1.0); // 前视距离 

    pravite_nh.param("perpendicular_tolerance", perpendicular_tolerance, 1.4); // 横向容忍程度   //郝红进
    pravite_nh.param("v1", v1, 3.0); // v_min
    pravite_nh.param("v2", v2, 3.0); // v_middle
    pravite_nh.param("v3", v3, 3.0); // v_max 
    pravite_nh.param("kz", kz, 1.5); // angular.z 的倍数

    pravite_nh.param("line_start_x", line_start_.x, 2.6);
    pravite_nh.param("line_start_y", line_start_.y, 1.5);
    pravite_nh.param("line_end_x", line_end_.x, -9.9);
    pravite_nh.param("line_end_y", line_end_.y, 1.5); 
    pravite_nh.param("lap_count_max", lap_count_max, 5); 


    // pravite_nh.param("ka", ka, 1.5); // angular.z 的倍数

    pravite_nh.param("controller_freq", controller_freq_, 50);   // 控制频率
    pravite_nh.param("goal_radius", goal_radius, 0.2); // 目标容忍度
    pravite_nh.param("debug_mode", debug_mode, false); // debug mode
    
    odom_sub_ = nh_.subscribe("/tianracer/odom", 1, &PurePursuit::odomCallback, this);
    PurePursuit_sub_ = nh_.subscribe("/pure_pursuit_mode", 1, &PurePursuit::PurePursuitCallback, this);
    // path_sub_ = nh_.subscribe("/move_base/TebLocalPlannerROS/global_plan", 1, &PurePursuit::pathCallback, this); //与move_base连用
    path_sub_ = nh_.subscribe("/global_plan", 1, &PurePursuit::pathCallback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/path_marker", 10);
    ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);    //Timer
    cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/tianracer/cmd_vel", 100);
    timer1 = nh_.createTimer(ros::Duration((1.0) / controller_freq_), &PurePursuit::controlLoopCallback, this); // 发布控制

    foundForwardPt_ = false;     //是否找到了前瞻点
    goal_received_ = false;      //是否收到了目标点
    goal_reached_ = false;       //是否已经到达了目标点
    velocity = 0.0;
    steering_ = 0.0;
    ackermann_cmd_ = ackermann_msgs::AckermannDriveStamped();
    this->cmd_vel = geometry_msgs::Twist();
    initMarker();
}

PurePursuit::~PurePursuit(){};
void PurePursuit::initMarker() {
    points_.header.frame_id = line_strip_.header.frame_id = goal_circle_.header.frame_id = "tianracer/odom";
    points_.ns = line_strip_.ns = goal_circle_.ns = "Markers";
    points_.action = line_strip_.action = goal_circle_.action = visualization_msgs::Marker::ADD;
    points_.pose.orientation.w = line_strip_.pose.orientation.w = goal_circle_.pose.orientation.w = 1.0;
    points_.id = 0;
    line_strip_.id = 1;
    goal_circle_.id = 2;

    points_.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip_.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle_.type = visualization_msgs::Marker::CYLINDER;

    points_.scale.x = 0.3;
    points_.scale.y = 0.3;
    points_.scale.z = 0.3;

    line_strip_.scale.x = 0.0;

    goal_circle_.scale.x = goal_radius;
    goal_circle_.scale.y = goal_radius;
    goal_circle_.scale.z = 0.1;
    // 绿色
    points_.color.g = 1.0f;
    points_.color.a = 1.0;
    // 蓝色
    line_strip_.color.b = 1.0;
    line_strip_.color.a = 1.0;
    // 黄色
    goal_circle_.color.r = 1.0;
    goal_circle_.color.g = 1.0;
    goal_circle_.color.b = 0.0;
    goal_circle_.color.a = 0.5;
}
/*!
 *
 * @param wayPt  路径点
 * @param car_pos   位姿
 * @return
 */
double PurePursuit::PointDistanceSquare(geometry_msgs::PoseStamped& wayPt,const geometry_msgs::Point& car_pos){
    double dx = wayPt.pose.position.x - car_pos.x;
    double dy = wayPt.pose.position.y - car_pos.y;
    return dx * dx + dy * dy;
}
/*!
 *
 * @param  odomMsg odom传感器数据
 * @author 测试
 * @author odom数据会出现漂移导致不准，可尝试使用tf变换 计算出base_link在map下的坐标
 */
void PurePursuit::PurePursuitCallback(const std_msgs::Int32::ConstPtr& msg) {
    Pure_Pursuit_out = msg->data;
}

bool PurePursuit::crossesLine(const geometry_msgs::Point& line_start, const geometry_msgs::Point& line_end, const geometry_msgs::Point& robot_position) {
    // Calculate the equation of the line (ax + by + c = 0)
    double a = line_end.y - line_start.y;
    double b = line_start.x - line_end.x;
    double c = line_end.x * line_start.y - line_start.x * line_end.y;

    // Calculate the distance from the robot position to the line
    double distance = fabs(a * robot_position.x + b * robot_position.y + c) / sqrt(a * a + b * b);

    // Check if the distance is small enough to consider the robot crossing the line
    return distance < 0.05;  // Adjust the threshold as needed
}

void PurePursuit::odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg) {  //车辆当前位置和目标点之间的欧几里得距离
    this->odom_ = *odomMsg; 
    if(this->goal_received_)  //收到目标点
    {
        double car2goal_x = this->goal_pos_.x - odomMsg->pose.pose.position.x;
        double car2goal_y = this->goal_pos_.y - odomMsg->pose.pose.position.y;
        double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
        /// 和move_base一起使用时这段注释解除
//        if(dist2goal < this->goal_radius)
//        {
//            this->goal_reached_ = true;
//            this->goal_received_ = false;
//            ROS_INFO("Goal Reached !");
//        }
    }
    geometry_msgs::Point robot_position = odomMsg->pose.pose.position;
    // ROS_INFO("Robot Position: x=%f, y=%f", robot_position.x, robot_position.y);
    // Define the line segment
    // geometry_msgs::Point line_start;
    // line_start.x = 2.6;
    // line_start.y = 1.5;

    // geometry_msgs::Point line_end;
    // line_end.x = -9.9;
    // line_end.y = 1.5;
        if (crossesLine(line_start_, line_end_, robot_position))
    {
        lap_count_++;
        ROS_INFO("Lap count: %d", lap_count_);
    }
}
/*!
 * @param pathMsg 全局路径
 * TODO importion 后期会使用局部路径
 */
void PurePursuit::pathCallback(const nav_msgs::Path::ConstPtr &pathMsg) {
    this->map_path_ = *pathMsg;
    /// 和move_base一起使用时下面两行注释掉
    goal_received_ = true;     
    goal_reached_ = false;
}
/*!
 * @param goalMsg
 */
/*!
 * @param carPose 当前位姿
 * @return 获取航向角误差
 */
double PurePursuit::getEta(const geometry_msgs::Pose& carPose)     ///计算机器人当前位姿与预瞄点之间的航向角误差
{
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);
    return atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);  //返回一个航向角误差
}
/*!
 * @param wayPt 寻找到的预选点
 * @param car_pos 位姿
 * @return 是否为前瞻点
 */
bool PurePursuit::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
//isWayPtAwayFromLfwDist判断路径点是否与车辆当前位置之间的距离是否大于或等于给定的前视距离 Lfw
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);
    if(dist < Lfw)
        return false;
    else if(dist >= Lfw)
        return true;
}
/*!
 * @param Lfm 前瞻距离
 * @param wayPt 路径点
 * @param carPose 当前机器人位姿
 * @return 
 */
int PurePursuit::minIndex(const geometry_msgs::Point& carPose){
    //找到机器人当前位置到路径上最近的路径点的索引
    int index_min = 0;  //表示最近点的索引
    int d_min = INT16_MAX;
    for(int i =0; i< map_path_.poses.size(); i++)
    {
        geometry_msgs::PoseStamped map_path_pose = map_path_.poses[i];
        double d_temp = PointDistanceSquare(map_path_pose,carPose);  //计算当前路径点与机器人当前位置之间的距离的平方
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
            // ROS_INFO("d_temp: %f", d_temp); // 打印最小距离
            // ROS_INFO("index_min: %d", index_min);
        }
    }
    // ROS_INFO("d_min: %d", d_min); // 打印最小距离
    return index_min;
}

bool PurePursuit::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta =  tf::getYaw(carPose.orientation);  //四元数转yaw

    //使用旋转矩阵，得到机器人坐标系下坐标
    float car_car2wayPt_x = cos(car_theta) * car2wayPt_x + sin(car_theta) * car2wayPt_y;//计算了路径点在机器人局部坐标系下的 x 轴方向的投影
    float car_car2wayPt_y = cos(car_theta) * car2wayPt_y - sin(car_theta) * car2wayPt_x;

    if(car_car2wayPt_x >0 && fabs(car_car2wayPt_y) < perpendicular_tolerance) //前方且侧向距离合理
        return true;
    else
        return false;
}
/*!
 * @param carPose 机器人位姿
 * @return 预瞄点对于机器人的距离
 */
geometry_msgs::Point PurePursuit::get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = tf::getYaw(carPose.orientation);
    geometry_msgs::Point forwardPt;                      //存储找到的前瞻点
    geometry_msgs::Point odom_car2WayPtVec;              //前瞻点相对于机器人坐标系下的位置向量
    foundForwardPt_ = false;
    int count_index_repate = 0;

    if(!goal_reached_){   //如果尚未到达目标点
        //寻找离机器人最近的路径点
        int index = minIndex(carPose_pos);     //机器人当前位置最近索引
        for(int i =index; i< map_path_.poses.size(); i++)  //寻找前瞻点索引
        {
            geometry_msgs::PoseStamped map_path_pose = map_path_.poses[i];
            geometry_msgs::PoseStamped odom_path_pose;

            // ROS_INFO("Total ForwardPt: %zu", map_path_.poses.size()); // 打印 map_path_.poses 的大小
            // ROS_INFO("Current Index: %d", index); // 打印索引值
            if (i == index && i == 0) {
                count_index_repate++;
            }
            try
            {
                tf_listener_.transformPose("tianracer/odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);
                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;

                bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose);       //判断提取的路径点是否在机器人的前方

                if(_isForwardWayPt)    //路径点同时在机器人的前方且超出前视距离
                {
                    bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos);   //路径点是否位于机器人的前视距离之外
                    if(_isWayPtAwayFromLfwDist)
                    {
                        forwardPt = odom_path_wayPt;  //该路径点标记为预选点
                        foundForwardPt_ = true;
                        break;
                    }
                }
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }
    }
    else if(goal_reached_)
    {
        forwardPt = odom_goal_pos_;
        foundForwardPt_ = false;
        //ROS_INFO("goal REACHED!");
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points_.points.clear();
    line_strip_.points.clear();

    if(foundForwardPt_ && !goal_reached_)
    {
        points_.points.push_back(carPose_pos);
        points_.points.push_back(forwardPt);
        line_strip_.points.push_back(carPose_pos);
        line_strip_.points.push_back(forwardPt);
    }

    marker_pub_.publish(points_);    //发布mark
    marker_pub_.publish(line_strip_);

    // 转到小车坐标系
    odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);

    return odom_car2WayPtVec;
}
/*!
 * @param eta 获取航向角误差
 * @return 机器人舵机转角
 */
double PurePursuit::getSteering(double eta)  //计算舵机转向
{
    return atan2(2*(this->base_shape_L*sin(eta)),(this->Lfw));
}

void PurePursuit::controlLoopCallback(const ros::TimerEvent&)  //控制车辆
{
        geometry_msgs::Pose carPose = this->odom_.pose.pose;
        geometry_msgs::Twist carVel = this->odom_.twist.twist;
    
        if(this->goal_received_)
        {
            double eta = getEta(carPose);  //计算yaw误差
        //        cout<<eta<<endl;
            if(foundForwardPt_)
            {
                this->steering_ = this->base_angle_ + getSteering(eta);  //计算舵机转角
                /*Estimate Gas Input*/
                if(!this->goal_reached_)
                {
                    this->velocity = this->reference_v;   //设置为参考速度 - 对应/tianracer/ackermann_cmd_stamped
                    if (abs(this->steering_) > 20.0 / 180.0 * PI) {
                        cmd_vel.linear.x = v1;
                        ka = 1.3;
                    } else if (abs(this->steering_) > 10.0 / 180.0 * PI) {
                        cmd_vel.linear.x = v2;
                        ka = 1.0;
                    } else {
                        cmd_vel.linear.x = v3;
                        ka = 1.0;
                    }         
                }
            }
        }
        if(goal_reached_)
        {
            velocity = 0.0;
            steering_ = 0.0;
        }

        if (lap_count_ >= lap_count_max)
        {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        }

            this->ackermann_cmd_.drive.steering_angle = this->steering_;
            this->ackermann_cmd_.drive.speed = this->velocity;
            this->ackermann_pub_.publish(this->ackermann_cmd_);

            this->cmd_vel.angular.z = kz * (this->steering_);
            // this->cmd_vel.linear.x = this->velocity;

                // if (abs(this->steering_) > 20.0 / 180.0 * PI) {
                //     cmd_vel.linear.x = v1;
                // } else if (abs(this->steering_) > 10.0 / 180.0 * PI) {
                //     cmd_vel.linear.x = v2;
                // } else {
                //     cmd_vel.linear.x = v3;
                // }
            this->cmd_vel_pub.publish(this->cmd_vel);    
}

int main(int argc, char **argv) {
    // 创建ROS节点
    ros::init(argc, argv, "PurePursuit");
    // 调用类
    PurePursuit ppc;
    // 多线程工作
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}