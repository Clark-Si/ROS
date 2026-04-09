#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <ctime>  // 用于统计运动耗时（仅日志用，不影响编译）

// 运动状态枚举（新增DELAY状态，仅用于间隔延迟）
enum MoveState {
    MOVE_FORWARD,  // 直行1米
    TURN_90DEG,    // 转向90度
    DELAY,         // 状态切换间隔延迟（新增）
    STOP           // 完成正方形，停止
};

// 全局变量（新增延迟相关变量）
bool odom_init = false;           // 里程计是否初始化完成
int current_side = 0;             // 已完成边数（0-4）
MoveState current_state = MOVE_FORWARD;
std::time_t start_time;           // 总运动耗时统计
bool first_move = true;           // 标记首次运动
// 新增：延迟相关变量
double delay_start_time = 0.0;    // 延迟开始时间（秒）
const double DELAY_DURATION = 0.5;// 间隔延迟时间（0.5秒）
MoveState next_state = MOVE_FORWARD; // 延迟后要切换的状态

// 核心配置（适配大部分底盘）
const double LINEAR_SPEED = 0.3;    // 直行速度 (m/s)
const double ANGULAR_SPEED = -0.3;  // 顺时针转向（负数）
const double TARGET_DISTANCE = 1.0; // 目标距离
const double TARGET_ANGLE = M_PI/2; // 90度（弧度）
const double DIST_TOLERANCE = 0.05; // 距离误差容忍
const double ANGLE_TOLERANCE = 0.05;// 角度误差容忍

// 单次运动的临时变量
struct MoveData {
    double start_x = 0.0;
    double start_y = 0.0;
    double start_yaw = 0.0;
    std::time_t phase_start; // 记录每段运动起始时间
} move_data;

// 四元数转偏航角（修复：增加四元数归一化，避免数据异常）
double get_yaw(const nav_msgs::Odometry::ConstPtr& msg) {
    double q0 = msg->pose.pose.orientation.w;
    double q1 = msg->pose.pose.orientation.x;
    double q2 = msg->pose.pose.orientation.y;
    double q3 = msg->pose.pose.orientation.z;
    // 新增：四元数归一化，防止里程计数据异常导致角度计算错误
    double norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm < 1e-6) return 0.0;
    q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
    return atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
}

// 里程计回调（仅修改状态切换逻辑，新增延迟触发）
void odom_callback(const nav_msgs::OdometryConstPtr &odom_msg)
{
    // 首次回调初始化
    if (!odom_init) {
        move_data.start_x = odom_msg->pose.pose.position.x;
        move_data.start_y = odom_msg->pose.pose.position.y;
        move_data.start_yaw = get_yaw(odom_msg);
        odom_init = true;
        start_time = std::time(nullptr); // 记录总运动起始时间
        
        // 初始化日志（关键参数+初始状态）
        ROS_INFO("\n=====================================");
        ROS_INFO("✅ 里程计初始化完成！");
        ROS_INFO("📌 初始位置：X=%.3f m, Y=%.3f m", move_data.start_x, move_data.start_y);
        ROS_INFO("📌 初始偏航角：%.3f rad (%.1f 度)", move_data.start_yaw, move_data.start_yaw*180/M_PI);
        ROS_INFO("⚙️  运动参数配置：");
        ROS_INFO("   - 直行速度：%.2f m/s", LINEAR_SPEED);
        ROS_INFO("   - 转向速度：%.2f rad/s (顺时针)", ANGULAR_SPEED);
        ROS_INFO("   - 目标边长：%.2f m，转向角度：90度", TARGET_DISTANCE);
        ROS_INFO("   - 状态切换间隔：%.1f 秒", DELAY_DURATION); // 新增日志
        ROS_INFO("=====================================\n");
        return;
    }

    // 实时里程计数据
    double curr_x = odom_msg->pose.pose.position.x;
    double curr_y = odom_msg->pose.pose.position.y;
    double curr_yaw = get_yaw(odom_msg);
    static int log_count = 0; // 控制日志输出频率（避免刷屏）
    
    // 每10次回调输出一次实时里程计（约50Hz→5Hz输出）
    if (++log_count % 10 == 0) {
        ROS_DEBUG_THROTTLE(1.0, 
            "📊 实时里程计：X=%.3f m, Y=%.3f m | 偏航角：%.3f rad (%.1f 度)",
            curr_x, curr_y, curr_yaw, curr_yaw*180/M_PI
        );
    }

    // 延迟状态下不处理任何逻辑（避免干扰）
    if (current_state == DELAY) return;

    // 闭环逻辑 + 状态切换日志
    if (current_state == MOVE_FORWARD) {
        // 首次进入直行阶段，记录起始时间
        if (first_move) {
            move_data.phase_start = std::time(nullptr);
            first_move = false;
            ROS_INFO("\n🚀 开始第%d条边直行...", current_side+1);
        }

        // 计算已走距离
        double dist = sqrt(pow(curr_x - move_data.start_x, 2) + pow(curr_y - move_data.start_y, 2));
        
        // 实时直行进度日志（每0.5秒输出一次）
        ROS_INFO_THROTTLE(0.5, 
            "📍 直行进度：已走 %.3f m / 目标 %.3f m (剩余 %.3f m)",
            dist, TARGET_DISTANCE, TARGET_DISTANCE - dist
        );

        // 直行完成，切换到延迟状态（准备转向）
        if (dist >= TARGET_DISTANCE - DIST_TOLERANCE) {
            std::time_t phase_end = std::time(nullptr);
            ROS_INFO("\n✅ 第%d条边直行完成！", current_side+1);
            ROS_INFO("   - 实际行驶距离：%.3f m", dist);
            ROS_INFO("   - 耗时：%ld 秒", phase_end - move_data.phase_start);
            ROS_INFO("   - 结束位置：X=%.3f m, Y=%.3f m", curr_x, curr_y);
            ROS_INFO("⏳ 间隔延迟%.1f秒后转向...\n", DELAY_DURATION); // 新增日志
            
            // 切换到延迟状态
            current_state = DELAY;
            next_state = TURN_90DEG; // 延迟后转向
            delay_start_time = ros::Time::now().toSec(); // 记录延迟开始时间
            move_data.start_yaw = curr_yaw;
            first_move = true; // 重置首次运动标记
        }
    } else if (current_state == TURN_90DEG) {
        // 首次进入转向阶段，记录起始时间
        if (first_move) {
            move_data.phase_start = std::time(nullptr);
            first_move = false;
            ROS_INFO("\n🔄 开始转向（第%d条边后）...", current_side+1);
        }

        // 计算转向角度差（归一化到[-π, π]）
        double angle_diff = curr_yaw - move_data.start_yaw;
        angle_diff = fmod(angle_diff + M_PI, 2*M_PI) - M_PI;
        
        // 实时转向进度日志（每0.5秒输出一次）
        ROS_INFO_THROTTLE(0.5, 
            "🔄 转向进度：当前角度差 %.3f rad (%.1f 度) / 目标 %.3f rad (90度)",
            angle_diff, angle_diff*180/M_PI, TARGET_ANGLE
        );

        // 转向完成，切换状态
        if (fabs(angle_diff) >= TARGET_ANGLE - ANGLE_TOLERANCE) {
            std::time_t phase_end = std::time(nullptr);
            current_side++;
            ROS_INFO("\n✅ 转向完成！");
            ROS_INFO("   - 实际转向角度：%.1f 度", angle_diff*180/M_PI);
            ROS_INFO("   - 耗时：%ld 秒", phase_end - move_data.phase_start);
            ROS_INFO("   - 当前偏航角：%.3f rad (%.1f 度)", curr_yaw, curr_yaw*180/M_PI);

            // 判断是否完成正方形
            if (current_side >= 4) {
                std::time_t total_end = std::time(nullptr);
                ROS_INFO("\n🎉 正方形运动全部完成！");
                ROS_INFO("📊 运动汇总：");
                ROS_INFO("   - 总边数：4 条，总耗时：%ld 秒", total_end - start_time);
                ROS_INFO("   - 初始位置：(%.3f, %.3f) m", move_data.start_x, move_data.start_y);
                ROS_INFO("   - 最终位置：(%.3f, %.3f) m", curr_x, curr_y);
                ROS_INFO("   - 最终偏航角：%.1f 度", curr_yaw*180/M_PI);
                ROS_INFO("=====================================\n");
                current_state = STOP;
            } else {
                ROS_INFO("⏳ 间隔延迟%.1f秒后直行...\n", DELAY_DURATION); // 新增日志
                // 切换到延迟状态
                current_state = DELAY;
                next_state = MOVE_FORWARD; // 延迟后直行
                delay_start_time = ros::Time::now().toSec(); // 记录延迟开始时间
                move_data.start_x = curr_x;
                move_data.start_y = curr_y;
                first_move = true;
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_square_move");
    ros::NodeHandle nh;

    // 开启ROS_DEBUG日志（默认关闭，需手动启用）
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    // 订阅/发布话题（增大队列）
    ros::Subscriber odom_sub = nh.subscribe("/odom", 50, odom_callback);
    ros::Publisher cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 50);

    geometry_msgs::Twist cmd_vel;
    ros::Rate loop_rate(50); // 50Hz控制频率

    // 节点启动日志
    ROS_INFO("\n📢 正方形运动节点已启动！");
    ROS_INFO("🔍 等待里程计数据（/odom）...");
    ROS_INFO("💡 提示：若长时间无响应，请检查：");
    ROS_INFO("   1. 底盘/仿真是否启动（/odom话题是否有数据）");
    ROS_INFO("   2. ROS环境是否配置（source ~/catkin_ws/devel/setup.bash）");
    ROS_INFO("   3. 话题名是否匹配（当前订阅/odom，发布/cmd_vel）\n");

    while (ros::ok())
    {
        // 等待里程计初始化
        if (!odom_init) {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        // 新增：处理延迟逻辑（核心改动）
        if (current_state == DELAY) {
            double current_time = ros::Time::now().toSec();
            // 延迟时间未到：保持静止
            if (current_time - delay_start_time < DELAY_DURATION) {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                cmd_pub_.publish(cmd_vel);
                ros::spinOnce();
                loop_rate.sleep();
                continue;
            }
            // 延迟时间到：切换到目标状态
            current_state = next_state;
            ROS_INFO("✅ 延迟结束，开始%s...", 
                     current_state == MOVE_FORWARD ? "直行" : "转向");
        }

        // 速度控制（原逻辑不变）
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        
        if (current_state == MOVE_FORWARD) {
            cmd_vel.linear.x = LINEAR_SPEED;
        } else if (current_state == TURN_90DEG) {
            cmd_vel.angular.z = ANGULAR_SPEED;
        } else if (current_state == STOP) {
            // 停止后输出最终日志
            static bool stop_log = true;
            if (stop_log) {
                ROS_INFO("\n🛑 小车已停止运动！");
                stop_log = false;
            }
        }

        // 发布速度指令 + 发布日志（DEBUG级别）
        cmd_pub_.publish(cmd_vel);
        ROS_DEBUG_THROTTLE(1.0, 
            "📤 发布速度指令：linear.x=%.3f m/s, angular.z=%.3f rad/s",
            cmd_vel.linear.x, cmd_vel.angular.z
        );
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
