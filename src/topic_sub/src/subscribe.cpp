// ROS2 节点和消息类型。
// Vector3Stamped 用来接收 IMU 欧拉角；PointStamped 用来接收图像像素坐标。
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/float64.hpp"

// Linux 串口和常用 C++ 工具。
#include <chrono>
#include <cerrno>
#include <cstdio>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

// ==================== 串口类 ====================
class Sender
{
private:
    int tty_ = -1; // 串口文件描述符，-1 表示打开失败

public:
    // 构造时直接打开并配置串口。
    Sender(const char* port = "/dev/ttyS0")
    {
        tty_ = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (tty_ == -1)
        {
            printf("串口打开失败: %s\n", port);
            return;
        }

        struct termios tty {};
        if (tcgetattr(tty_, &tty) != 0)
        {
            printf("读取串口配置失败\n");
            close(tty_);
            tty_ = -1;
            return;
        }

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
        cfmakeraw(&tty);

        // 8N1：8 位数据位、无校验、1 位停止位；关闭软硬件流控。
        tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
        tty.c_cflag |= CS8 | CREAD | CLOCAL;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(tty_, TCSANOW, &tty) != 0)
        {
            printf("写入串口配置失败\n");
            close(tty_);
            tty_ = -1;
            return;
        }

        // 清掉启动前串口里残留的旧数据。
        tcflush(tty_, TCIOFLUSH);
        printf("串口打开成功: %s @ 115200\n", port);
    }

    // 程序退出时关闭串口。
    ~Sender()
    {
        if (tty_ != -1)
        {
            close(tty_);
        }
    }

    // 发送字符串。
    // 调用时自己决定是否带换行，例如 "X1.23Y0.00E\n"。
    bool send_string(const std::string& data)
    {
        if (tty_ == -1) return false;

        size_t total_written = 0;
        while (total_written < data.size())
        {
            const ssize_t written = write(
                tty_,
                data.data() + total_written,
                data.size() - total_written);

            if (written > 0)
            {
                total_written += static_cast<size_t>(written);
                continue;
            }

            if (written < 0 && errno == EINTR)
            {
                continue;
            }

            return false;
        }

        return true;
    }
};

// ==================== PID 类 ====================
class PID
{
private:
    double kp_; double ki_; double kd_;
    double integral_; double prev_error_;
    double output_max_; double output_min_;
    double integral_max_; double integral_min_;

public:
    // 初始化 PID 参数、输出限幅和积分限幅。
    PID(
        double kp = 0.0,
        double ki = 0.0,
        double kd = 0.0,
        double output_max = 1e6,
        double output_min = -1e6,
        double integral_limit = 100.0)
    {
        kp_ = kp; ki_ = ki; kd_ = kd; integral_ = 0.0; prev_error_ = 0.0;
        output_max_ = output_max; output_min_ = output_min;
        set_integral_limit(integral_limit);
    }

    // 运行时设置 PID 参数。
    // integral_limit 是积分项单独限幅，不再和输出限幅共用。
    void pid_params(double p, double i, double d, double max, double min, double integral_limit = 100.0) 
    {
        kp_ = p; ki_ = i; 
        kd_ = d; 
        output_max_ = max; 
        output_min_ = min;
        set_integral_limit(integral_limit);
    }

    // 根据误差计算 PID 输出。
    double output_pos(double error, double dt) 
    {
        integral_ += error * dt;     
        if (integral_ > integral_max_) integral_ = integral_max_;
        if (integral_ < integral_min_) integral_ = integral_min_;
        double derivative_ = (error - prev_error_) / dt;
        prev_error_ = error;
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative_;       
        if (output > output_max_) output = output_max_;
        if (output < output_min_) output = output_min_;
        return output;
    }

    // 清空积分和上一次误差。
    void reset() 
    { 
        integral_ = 0.0; 
        prev_error_ = 0.0; 
    }

private:
    // 使用对称积分限幅，例如 50 表示 integral_ 限制在 [-50, 50]。
    void set_integral_limit(double limit)
    {
        if (limit < 0.0) limit = -limit;
        integral_max_ = limit;
        integral_min_ = -limit;

        if (integral_ > integral_max_) integral_ = integral_max_;
        if (integral_ < integral_min_) integral_ = integral_min_;
    }
};

// 所有可在 RQT 中实时调整的控制参数。
struct ControlParameters
{
    double imu_kp = 34.0;
    double imu_ki = 1.0;
    double imu_kd = 0.0185;
    double kp_x = 0.035;
    double ki_x = 0.0;
    double kd_x = 0.000025;
    double kp_y = 0.1;
    double ki_y = 0.0;
    double kd_y = 0.0;
    double output_max = 1000.0;
    double output_min = -1000.0;
    double outer_yaw_max = 180.0;
    double imu_integral_limit = 100.0;
    double x_integral_limit = 100.0;
    double y_integral_limit = 50.0;
    double vision_lookahead_time = 0.045;
    double vision_ff_max_px = 500.0;
};

// ==================== 订阅与控制节点 ====================
class Subscriber : public rclcpp::Node
{
private:
    // 固定话题和串口，不通过参数修改。
    static constexpr const char* IMU_TOPIC = "/imu/angle";
    static constexpr const char* STAR_TOPIC = "/star/center";
    static constexpr const char* IMAGE_CENTER_TOPIC = "/image/center";
    static constexpr const char* SERIAL_PORT = "/dev/ttyS3";
    static constexpr double YAW_OFFSET = 0.0;  // 和 IMU 内环 PID 相关的测试偏移，单位：度
    static constexpr double SERIAL_SEND_HZ = 100.0;        // 串口发送频率，单位 Hz

    std::shared_ptr<Sender> sender_;  // 串口对象
    PID pid_;                         // IMU 内环 PID
    PID pid_x_;                       // 视觉 X 外环 PID
    PID pid_y_;                       // 视觉 Y PID
    ControlParameters params_;        // RQT 实时调节的参数

    bool have_imu_yaw_ = false;       // 是否已经收到 IMU yaw
    bool have_visual_target_ = false; // 是否已经收到可用视觉目标
    double base_yaw_ = 0.0;           // 启动第一帧 yaw，没有视觉目标时作为内环保持角
    double current_yaw_ = 0.0;        // 当前 yaw 角，单位：度
    double target_yaw_ = 0.0;         // IMU 内环要追踪的目标 yaw
    double latest_x_speed_ = 0.0;     // IMU 内环最近一次输出的 X 速度
    double latest_y_speed_ = 0.0;     // 视觉 Y 轴最近一次输出
    rclcpp::Time last_imu_time_;      // 上一帧 IMU 时间，用来计算内环 dt

    bool have_image_center_ = false;  // 是否已经收到画面中心点
    bool visual_time_ready_ = false;  // 是否已有上一帧视觉 PID 时间
    bool zero_sent_after_timeout_ = true; // 目标丢失后是否已经发送过零速
    double image_center_x_ = 0.0;         // 最新画面中心 x，像素坐标
    double image_center_y_ = 0.0;         // 最新画面中心 y，像素坐标
    rclcpp::Time last_visual_time_;       // 上一帧视觉 PID 时间，用来计算外环 dt
    rclcpp::Time last_star_time_;         // 上一次收到五角星中心的时间，用来判断目标丢失
    bool have_last_error_x_ = false;      // 是否已经有上一帧 X 像素误差
    double last_error_x_ = 0.0;           // 上一帧 X 像素误差，用来估计目标横向运动速度

    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr angle_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr star_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr center_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr imu_inner_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr current_yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr imu_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vision_error_x_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr predicted_error_x_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_offset_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vision_y_speed_pub_;
    rclcpp::TimerBase::SharedPtr star_timeout_timer_;
    rclcpp::TimerBase::SharedPtr serial_send_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

public:
    // 初始化 PID、串口、IMU 订阅和视觉中心点订阅。
    Subscriber() : Node("subscriber")
    {
        declare_control_parameters();
        apply_control_parameters();
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&Subscriber::parameter_callback, this, std::placeholders::_1));

        sender_ = std::make_shared<Sender>(SERIAL_PORT);

        create_debug_publishers();

        // IMU 角度订阅：使用 vector.z 作为 yaw，单位为度。
        angle_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            IMU_TOPIC, 10,
            std::bind(&Subscriber::angle_callback, this, std::placeholders::_1));

        // 画面中心点订阅：由视觉发布节点持续发布。
        center_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            IMAGE_CENTER_TOPIC, 10,
            std::bind(&Subscriber::image_center_callback, this, std::placeholders::_1));

        // 五角星中心点订阅：只有识别到五角星时才会发布。
        star_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            STAR_TOPIC, 10,
            std::bind(&Subscriber::star_center_callback, this, std::placeholders::_1));

        // 定时检查五角星是否丢失，防止下位机一直执行旧速度。
        star_timeout_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Subscriber::check_star_timeout, this));

        // 串口发送定时器：固定频率发送最新速度，和 IMU 刷新频率解耦。
        serial_send_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / SERIAL_SEND_HZ)),
            std::bind(&Subscriber::send_latest_speed, this));

        RCLCPP_INFO(this->get_logger(), "订阅节点已启动");
        RCLCPP_INFO(
            this->get_logger(),
            "订阅: %s, %s, %s, 串口: %s @ 115200, 串口发送频率: %.1f Hz",
            IMU_TOPIC, STAR_TOPIC, IMAGE_CENTER_TOPIC, SERIAL_PORT, SERIAL_SEND_HZ);
        RCLCPP_INFO(
            this->get_logger(),
            "视觉前馈: lookahead_time %.3f s, ff_max %.1f px",
            params_.vision_lookahead_time, params_.vision_ff_max_px);
        RCLCPP_INFO(this->get_logger(), "PID 参数支持通过 RQT 实时调整");
    }

private:
    void declare_control_parameters()
    {
        params_.imu_kp = this->declare_parameter<double>("imu_kp", params_.imu_kp);
        params_.imu_ki = this->declare_parameter<double>("imu_ki", params_.imu_ki);
        params_.imu_kd = this->declare_parameter<double>("imu_kd", params_.imu_kd);
        params_.kp_x = this->declare_parameter<double>("kp_x", params_.kp_x);
        params_.ki_x = this->declare_parameter<double>("ki_x", params_.ki_x);
        params_.kd_x = this->declare_parameter<double>("kd_x", params_.kd_x);
        params_.kp_y = this->declare_parameter<double>("kp_y", params_.kp_y);
        params_.ki_y = this->declare_parameter<double>("ki_y", params_.ki_y);
        params_.kd_y = this->declare_parameter<double>("kd_y", params_.kd_y);
        params_.output_max = this->declare_parameter<double>("output_max", params_.output_max);
        params_.output_min = this->declare_parameter<double>("output_min", params_.output_min);
        params_.outer_yaw_max = this->declare_parameter<double>("outer_yaw_max", params_.outer_yaw_max);
        params_.imu_integral_limit = this->declare_parameter<double>(
            "imu_integral_limit", params_.imu_integral_limit);
        params_.x_integral_limit = this->declare_parameter<double>(
            "x_integral_limit", params_.x_integral_limit);
        params_.y_integral_limit = this->declare_parameter<double>(
            "y_integral_limit", params_.y_integral_limit);
        params_.vision_lookahead_time = this->declare_parameter<double>(
            "vision_lookahead_time", params_.vision_lookahead_time);
        params_.vision_ff_max_px = this->declare_parameter<double>(
            "vision_ff_max_px", params_.vision_ff_max_px);
    }

    // 将当前参数写入三个 PID，修改参数时不清空已有积分。
    void apply_control_parameters()
    {
        pid_.pid_params(
            params_.imu_kp, params_.imu_ki, params_.imu_kd,
            params_.output_max, params_.output_min, params_.imu_integral_limit);
        pid_x_.pid_params(
            params_.kp_x, params_.ki_x, params_.kd_x,
            params_.outer_yaw_max, -params_.outer_yaw_max, params_.x_integral_limit);
        pid_y_.pid_params(
            params_.kp_y, params_.ki_y, params_.kd_y,
            params_.output_max, params_.output_min, params_.y_integral_limit);
    }

    // RQT 修改参数时会进入这里，校验通过后立即作用到控制器。
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        ControlParameters next = params_;

        for (const auto& parameter : parameters)
        {
            const std::string& name = parameter.get_name();
            const auto value = [&parameter]() { return parameter.as_double(); };

            if (name == "imu_kp") next.imu_kp = value();
            else if (name == "imu_ki") next.imu_ki = value();
            else if (name == "imu_kd") next.imu_kd = value();
            else if (name == "kp_x") next.kp_x = value();
            else if (name == "ki_x") next.ki_x = value();
            else if (name == "kd_x") next.kd_x = value();
            else if (name == "kp_y") next.kp_y = value();
            else if (name == "ki_y") next.ki_y = value();
            else if (name == "kd_y") next.kd_y = value();
            else if (name == "output_max") next.output_max = value();
            else if (name == "output_min") next.output_min = value();
            else if (name == "outer_yaw_max") next.outer_yaw_max = value();
            else if (name == "imu_integral_limit") next.imu_integral_limit = value();
            else if (name == "x_integral_limit") next.x_integral_limit = value();
            else if (name == "y_integral_limit") next.y_integral_limit = value();
            else if (name == "vision_lookahead_time") next.vision_lookahead_time = value();
            else if (name == "vision_ff_max_px") next.vision_ff_max_px = value();
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;

        if (next.output_min >= next.output_max)
        {
            result.reason = "output_min 必须小于 output_max";
            return result;
        }
        if (next.outer_yaw_max <= 0.0 ||
            next.imu_integral_limit < 0.0 ||
            next.x_integral_limit < 0.0 ||
            next.y_integral_limit < 0.0 ||
            next.vision_lookahead_time < 0.0 ||
            next.vision_ff_max_px < 0.0)
        {
            result.reason = "限幅和预测时间不能为负，outer_yaw_max 必须大于 0";
            return result;
        }

        params_ = next;
        apply_control_parameters();
        result.successful = true;
        result.reason = "控制参数已更新";
        RCLCPP_INFO(this->get_logger(), "控制参数已实时更新");
        return result;
    }

    // 创建 RQT Plot / PlotJuggler 可以直接选择的标量话题。
    void create_debug_publishers()
    {
        imu_inner_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/pid/imu_inner_speed", 10);
        current_yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/pid/current_yaw", 10);
        target_yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/pid/target_yaw", 10);
        imu_error_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/pid/imu_error", 10);
        vision_error_x_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/pid/vision_error_x", 10);
        predicted_error_x_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/pid/vision_predicted_error_x", 10);
        yaw_offset_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/pid/vision_yaw_offset", 10);
        vision_y_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/pid/vision_y_speed", 10);
    }

    static void publish_value(
        const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& publisher,
        double value)
    {
        std_msgs::msg::Float64 msg;
        msg.data = value;
        publisher->publish(msg);
    }

    // 把角度误差限制到 [-180, 180]，避免 179 到 -179 时突变。
    static double normalize_angle(double angle)
    {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
        
    // 只保存最新的画面中心点，后续五角星回调用它计算像素误差。
    void image_center_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        image_center_x_ = msg->point.x;
        image_center_y_ = msg->point.y;
        have_image_center_ = true;
    }

    // 视觉外环回调：
    // 收到五角星中心后，与画面中心做差。
    // X 方向加入简单视觉前馈：
    // 先用相邻两帧 error_x 算出目标横向移动速度，再预测 lookahead 时间后的误差。
    // 预测误差进入 pid_x_，生成相对当前 yaw 的修正角，再得到 target_yaw_。
    // Y 误差进入 pid_y_，生成 latest_y_speed_。
    void star_center_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (!have_image_center_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "还没有收到画面中心点: %s", IMAGE_CENTER_TOPIC);
            return;
        }

        if (!have_imu_yaw_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "还没有收到 IMU yaw，暂不能生成双环目标");
            return;
        }

        const rclcpp::Time now = this->now();
        double dt = 0.0167; // 第一帧视觉数据没有上一帧时间，先给一个默认 dt
        if (visual_time_ready_)
        {
            dt = (now - last_visual_time_).seconds();
            if (dt <= 0.0)
            {
                dt = 0.001;
            }
        }

        //初始化
        visual_time_ready_ = true;
        last_visual_time_ = now;
        last_star_time_ = now;
        have_visual_target_ = true;
        zero_sent_after_timeout_ = false;

        // X 误差使用“画面中心 - 星点中心”。这样视觉 X 的 P、I、D
        // 参数都使用正数时，云台仍按原来正确的方向追踪目标。
        const double error_x = image_center_x_ - msg->point.x; // 星在画面左侧时为正
        const double error_y = msg->point.y - image_center_y_; // 星在画面下方时为正

        // 视觉前馈：估计 X 像素误差变化速度，提前预测一小段时间后的误差。
        // error_x_speed > 0 表示目标正在向画面左侧偏移，预测误差会比当前误差更大。
        double error_x_speed = 0.0;
        if (have_last_error_x_)
        {
            error_x_speed = (error_x - last_error_x_) / dt;
        }

        double feedforward_error_x = error_x_speed * params_.vision_lookahead_time;
        if (feedforward_error_x > params_.vision_ff_max_px)
            feedforward_error_x = params_.vision_ff_max_px;
        if (feedforward_error_x < -params_.vision_ff_max_px)
            feedforward_error_x = -params_.vision_ff_max_px;

        const double predicted_error_x = error_x + feedforward_error_x;
        last_error_x_ = error_x;
        have_last_error_x_ = true;

        const double yaw_offset = pid_x_.output_pos(predicted_error_x, dt);  // 预测误差进入视觉外环
        latest_y_speed_ = pid_y_.output_pos(error_y, dt);
        target_yaw_ = normalize_angle(current_yaw_ + yaw_offset);

        publish_value(vision_error_x_pub_, error_x);
        publish_value(predicted_error_x_pub_, predicted_error_x);
        publish_value(yaw_offset_pub_, yaw_offset);
        publish_value(vision_y_speed_pub_, latest_y_speed_);
    }

    // 目标丢失保护：
    // /star/center 只在识别到目标时发布。
    // 如果超过 0.5 秒没有新目标，就发送一次零速，并重置三路 PID。
    void check_star_timeout()
    {
        if (!have_visual_target_ || zero_sent_after_timeout_)
        {
            return;
        }

        if ((this->now() - last_star_time_).seconds() <= 0.5)
        {
            return;
        }

        send_visual_speed(0.0, 0.0);
        pid_.reset();
        pid_x_.reset();
        pid_y_.reset();
        visual_time_ready_ = false;
        have_visual_target_ = false;
        latest_x_speed_ = 0.0;
        latest_y_speed_ = 0.0;
        have_last_error_x_ = false;
        last_error_x_ = 0.0;
        target_yaw_ = normalize_angle(base_yaw_ + YAW_OFFSET);
        zero_sent_after_timeout_ = true;

        RCLCPP_WARN(
            this->get_logger(),
            "超过 0.5 秒未收到五角星中心点，已发送零速");
    }

    // 定时发送最新速度。
    void send_latest_speed()
    {
        if (!have_imu_yaw_)
        {
            return;
        }

        const double y_speed = have_visual_target_ ? latest_y_speed_ : 0.0;
        send_visual_speed(latest_x_speed_, y_speed);
    }

    // 按下位机协议发送速度命令。
    // 当前实际发送格式：X%.2fY%.2fE\n
    void send_visual_speed(double speed_x, double speed_y)
    {
        char speed_cmd[64];
        std::snprintf(speed_cmd, sizeof(speed_cmd), "X%.2fY%.2fE\n", -speed_x, -speed_y);
        sender_->send_string(speed_cmd);
    }

    // IMU 内环回调：
    // 第一帧 yaw 作为基准角 base_yaw_。
    // 没有视觉目标时：追踪 base_yaw_ + YAW_OFFSET，用来单独调内环。
    // 有视觉目标时：追踪视觉外环生成的 target_yaw_，形成双环控制。
    void angle_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
    {
        current_yaw_ = msg->vector.z;
        const rclcpp::Time now = this->now();

        // 第一帧 yaw 作为双环控制的基准角。
        if (!have_imu_yaw_)
        {
            base_yaw_ = current_yaw_;
            target_yaw_ = normalize_angle(base_yaw_ + YAW_OFFSET);
            last_imu_time_ = now;
            have_imu_yaw_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "IMU 基准 yaw 初始化为: %.2f 度, 当前内环目标 yaw: %.2f 度",
                base_yaw_, target_yaw_);
            return;
        }

        double dt = (now - last_imu_time_).seconds();
        if (dt <= 0.0)
        {
            dt = 0.001;
        }
        last_imu_time_ = now;

        const double imu_error = normalize_angle(target_yaw_ - current_yaw_);
        latest_x_speed_ = pid_.output_pos(imu_error, dt);
        publish_value(current_yaw_pub_, current_yaw_);
        publish_value(target_yaw_pub_, target_yaw_);
        publish_value(imu_error_pub_, imu_error);
        publish_value(imu_inner_speed_pub_, latest_x_speed_);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Subscriber>());
    rclcpp::shutdown();
    return 0;
}
