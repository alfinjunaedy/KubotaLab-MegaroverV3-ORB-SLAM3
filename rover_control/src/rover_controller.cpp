#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include <chrono>
#include <cmath>
#include <thread>

using namespace std;
using namespace std::chrono_literals;

class RoverController : public rclcpp::Node {
public:
    RoverController() : Node("rover_controller") {
        linear_speed_ = this->declare_parameter<double>("linear_speed", 0.1);   // not used
        angular_speed_ = this->declare_parameter<double>("angular_speed", 0.5); // not used

        param_cb_handle_ = this->add_on_set_parameters_callback(std::bind(&RoverController::on_param_change, this, std::placeholders::_1));
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/rover_twist", 10);
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.best_effort();
        odo_sub_ = create_subscription<geometry_msgs::msg::Twist>("/rover_odo", qos, std::bind(&RoverController::odom_callback, this, std::placeholders::_1));
        sensor_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>("/rover_sensor", qos, std::bind(&RoverController::sensor_callback, this, std::placeholders::_1));

        // x+=forward, y+=left, z+=up, quaternion xyzw, NWU frame.
        orb_pose_sub_ = create_subscription<geometry_msgs::msg::Pose>("/orb_pose", qos, std::bind(&RoverController::orb_pose_callback, this, std::placeholders::_1));
        timer_ = create_wall_timer(100ms, std::bind(&RoverController::update_odometry, this));
        last_time_ = now();
        last_orb_time_ = now();
        RCLCPP_INFO(get_logger(), "Rover Controller Started (linear_speed=%.2f m/s, angular_speed=%.2f rad/s)", linear_speed_, angular_speed_);

        std::thread([this]() {
            std::this_thread::sleep_for(2s);
            
            // Planner ****************************************************************************************
            move_forward(1, 0.2);
            rotate_cw(45);
            move_forward(1, 0.2);
            rotate_ccw(45);
            move_forward(3, 0.2);

            rotate_ccw(90);
            move_forward(3.4, 0.2);
            rotate_ccw(80);
            move_forward(3.4, 0.2);
            rotate_ccw(80);
            move_forward(2.5, 0.2);
            rotate_ccw(90);
            move_backward(1.4, 0.2);

            // move_forward(2.0);  delay_seconds(1); 
            // rotate_cw(45);      delay_seconds(1);
            // move_forward(3.0);  delay_seconds(1);
            // rotate_ccw(45);     delay_seconds(1);
            // move_forward(2);    delay_seconds(1);
            // rotate_cw(45);      delay_seconds(1);
            // move_forward(2);    delay_seconds(1);
            // rotate_ccw(45);     delay_seconds(1);
            // move_forward(4);    delay_seconds(1);    
            // ************************************************************************************************
            
            stop_robot();

        }).detach();
    }

private:
    rcl_interfaces::msg::SetParametersResult on_param_change(const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for(const auto & p : params) {
            if(p.get_name() == "linear_speed") {
                double v = p.as_double();
                if(v <= 0.0) {
                    result.successful = false;
                    result.reason = "linear_speed must be > 0";
                    continue;
                }
                linear_speed_ = v;
                RCLCPP_INFO(get_logger(), "linear_speed updated to %.2f m/s", linear_speed_);
            }
            else if(p.get_name() == "angular_speed") {
                double v = p.as_double();
                if(v <= 0.0) {
                    result.successful = false;
                    result.reason = "angular_speed must be > 0";
                    continue;
                }
                angular_speed_ = v;
                RCLCPP_INFO(get_logger(), "angular_speed updated to %.2f rad/s", angular_speed_);
            }
        }
        return result;
    }

    void odom_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        linear_vel_ = msg->linear.x;
        angular_vel_ = msg->angular.z;
    }

    void orb_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        double raw_yaw = yaw_from_quaternion(msg->orientation);
        if(!yaw_offset_set_) { 
            yaw_offset_ = raw_yaw;
            yaw_offset_set_ = true;
            RCLCPP_INFO(get_logger(), "Yaw offset captured: raw=%.2f deg -> treated as 0 deg", raw_yaw * 180.0 / M_PI);
        }
        orb_x_ = msg->position.x;
        orb_y_ = msg->position.y;
        orb_yaw_ = normalize_angle(raw_yaw - yaw_offset_);
        if(!orb_initialized_) {
            x_ = orb_x_;
            y_ = orb_y_;
            yaw_ = orb_yaw_;
            orb_initialized_ = true;
        }
        last_orb_time_ = now();
        orb_fresh_ = true;
    }

    void sensor_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        bumper_ = msg->data[0];
        battery_ = msg->data[1];
        RCLCPP_INFO( //RCLCPP_INFO_THROTTLE
            get_logger(),
            //*get_clock(),
            //2000,
            "my x=%.2f y=%.2f yaw=%.2f deg | bumper=%d | battery=%.2fV | orb_age=%.2fs",
            x_,
            y_,
            yaw_ * 180.0 / M_PI,
            bumper_,
            battery_ / 1000.0,
            (now() - last_orb_time_).seconds()
        );
    }

    void update_odometry() {
        auto current_time = now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        double x_pred = x_ + linear_vel_ * cos(yaw_) * dt;
        double y_pred = y_ + linear_vel_ * sin(yaw_) * dt;
        double yaw_pred = yaw_ + angular_vel_ * dt;
        bool orb_valid = orb_initialized_ && (current_time - last_orb_time_).seconds() < orb_timeout_;
        if(orb_valid && orb_fresh_) {
            x_ = orb_weight_ * orb_x_ + odom_weight_ * x_pred;
            y_ = orb_weight_ * orb_y_ + odom_weight_ * y_pred;
            double sin_blend = orb_weight_ * sin(orb_yaw_) + odom_weight_ * sin(yaw_pred);
            double cos_blend = orb_weight_ * cos(orb_yaw_) + odom_weight_ * cos(yaw_pred);
            yaw_ = atan2(sin_blend, cos_blend);
            orb_fresh_ = false;
        }
        else {
            x_ = x_pred;
            y_ = y_pred;
            yaw_ = yaw_pred;
            if(orb_initialized_ && !orb_valid) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "ORB pose stale (>%.1fs) - running on odometry only",
                    orb_timeout_
                );
            }
        }
    }

    static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q) {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return atan2(siny_cosp, cosy_cosp);
    }

    static double normalize_angle(double angle) {
        while(angle > M_PI) angle -= 2.0 * M_PI;
        while(angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void send_velocity(double linear, double angular) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = linear;
        cmd.angular.z = angular;
        cmd_pub_->publish(cmd);
    }

    void stop_robot() {
        send_velocity(0.0, 0.0);
        RCLCPP_INFO(get_logger(), "STOP");
    }

    void delay_seconds(double seconds) {
        RCLCPP_INFO(get_logger(), "DELAY %.2f s", seconds);
        auto start = now();
        while(rclcpp::ok() && (now() - start).seconds() < seconds) {
            if(check_safety()) return;
            send_velocity(0.0, 0.0); // hold still
            rclcpp::sleep_for(50ms);
        }
    }

    void move_forward(double meters, double speed=0.1) {
        speed = fabs(speed);
        if (speed<0.01) speed=0.01; else if (speed>0.5) speed=0.5;
        double start_x = x_;
        double start_y = y_;
        RCLCPP_INFO(
            get_logger(),
            "MOVE FORWARD %.2f m (speed=%.2f m/s)",
            meters,
            speed
        );
        while(rclcpp::ok()) {
            if(check_safety()) return;
            double dx = x_ - start_x;
            double dy = y_ - start_y;
            double dist = sqrt(dx*dx + dy*dy);
            if(dist >= meters-0.050) break;
            send_velocity(speed, 0.0);
            rclcpp::sleep_for(50ms);
        }
        stop_robot();
    }

    void move_backward(double meters, double speed=0.1) {
        speed = fabs(speed);
        if (speed<0.01) speed=0.01; else if (speed>0.5) speed=0.5;
        double start_x = x_;
        double start_y = y_;
        RCLCPP_INFO(
            get_logger(),
            "MOVE BACKWARD %.2f m (speed=%.2f m/s)",
            meters,
            speed
        );
        while(rclcpp::ok()) {
            if(check_safety()) return;
            double dx = x_ - start_x;
            double dy = y_ - start_y;
            double dist = sqrt(dx*dx + dy*dy);
            if(dist >= meters-0.050) break;
            send_velocity(-speed, 0.0);
            rclcpp::sleep_for(50ms);
        }
        stop_robot();
    }

    void rotate_ccw(double deg, double speed=0.5) {
        deg = fabs(deg);
        speed = fabs(speed);
        if (speed<0.01) speed=0.01; else if (speed>1.58) speed=1.58;
        double target = normalize_angle(yaw_ + deg * M_PI / 180.0);
        RCLCPP_INFO(get_logger(),"ROTATE CW %.1f deg",deg);
        while(rclcpp::ok()) {
            if(check_safety()) return;
            double err = normalize_angle(target - yaw_);
            if(std::fabs(err) < 0.15) break;
            send_velocity(0.0,speed);
            rclcpp::sleep_for(20ms);
        }
        stop_robot();
    }

    void rotate_cw(double deg, double speed=0.5) {
        deg = fabs(deg);
        speed = fabs(speed);
        if (speed<0.01) speed=0.01; else if (speed>1.58) speed=1.58;
        double target = normalize_angle(yaw_ - deg * M_PI / 180.0);
        RCLCPP_INFO(get_logger(),"ROTATE CCW %.1f deg",deg);
        while(rclcpp::ok()) {
            if(check_safety()) return;
            double err = normalize_angle(target - yaw_);
            if(std::fabs(err) < 0.15) break;
            send_velocity(0.0,-speed);
            rclcpp::sleep_for(20ms);
        }
        stop_robot();
    }

    bool check_safety() {
        if(bumper_ != 0) {
            RCLCPP_WARN(get_logger(), "BUMPER HIT");
            stop_robot();
            return true;
        }
        if(battery_ < 22000) {
            RCLCPP_WARN(get_logger(), "LOW BATTERY");
            stop_robot();
            return true;
        }
        return false;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr odo_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sensor_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr orb_pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    double linear_speed_ = 0.1;
    double angular_speed_ = 0.5;
    double linear_vel_ = 0.0;
    double angular_vel_ = 0.0;

    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;

    double orb_x_ = 0.0;
    double orb_y_ = 0.0;
    double orb_yaw_ = 0.0;

    double yaw_offset_ = 0.0;
    bool yaw_offset_set_ = false;
    bool orb_initialized_ = false;
    bool orb_fresh_ = false;
    rclcpp::Time last_orb_time_;

    // Complementary filter weights
    const double orb_weight_ = 0.75; //0.75
    const double odom_weight_ = 1.0 - orb_weight_;
    const double orb_timeout_ = 1.0; // seconds

    int bumper_ = 0;
    int battery_ = 0;
    rclcpp::Time last_time_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoverController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}