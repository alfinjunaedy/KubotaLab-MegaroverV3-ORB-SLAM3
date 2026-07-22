#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include <cmath>
#include <mutex>

using std::placeholders::_1;

class MapBuilder : public rclcpp::Node
{
public:

    MapBuilder()
    : Node("map_builder")
    {
        //---------------------------------------------
        // Parameters
        //---------------------------------------------

        keyframe_distance_ =
            declare_parameter("keyframe_distance",0.50);

        keyframe_angle_deg_ =
            declare_parameter("keyframe_angle_deg",30.0);

        voxel_size_ =
            declare_parameter("voxel_size",0.05);

        //---------------------------------------------
        // Subscribers
        //---------------------------------------------

        auto qos = rclcpp::SensorDataQoS();

        pose_sub_ =
            create_subscription<geometry_msgs::msg::Pose>(
                "/orb_pose",
                qos,
                std::bind(
                    &MapBuilder::orbPoseCallback,
                    this,
                    _1));

        cloud_sub_ =
            create_subscription<sensor_msgs::msg::PointCloud2>(
                "/camera/camera/depth/color/points",
                qos,
                std::bind(
                    &MapBuilder::cloudCallback,
                    this,
                    _1));

        //---------------------------------------------
        // Publishers
        //---------------------------------------------

        rclcpp::QoS qos_map(1);
        qos_map.transient_local();
        qos_map.reliable();

        global_cloud_pub_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                "/global_cloud",
                qos_map);

        robot_marker_pub_ =
            create_publisher<visualization_msgs::msg::Marker>(
                "/robot_marker",
                1);

        //---------------------------------------------
        // Global cloud
        //---------------------------------------------

        global_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        current_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        last_keyframe_x_=0.0;
        last_keyframe_y_=0.0;
        last_keyframe_yaw_=0.0;

        initialized_=false;

        RCLCPP_INFO(get_logger(), "Map Builder Started");
    }

private:

    //--------------------------------------------------------
    // Normalize angle
    //--------------------------------------------------------

    double normalizeAngle(double a)
    {
        while(a>M_PI)
            a-=2.0*M_PI;

        while(a<-M_PI)
            a+=2.0*M_PI;

        return a;
    }

    //--------------------------------------------------------
    // Quaternion -> yaw
    //--------------------------------------------------------

    double quaternionYaw(
        const geometry_msgs::msg::Quaternion &q)
    {
        double siny=
            2.0*(q.w*q.z + q.x*q.y);

        double cosy=
            1.0-
            2.0*(q.y*q.y + q.z*q.z);

        return atan2(siny,cosy);
    }

    //--------------------------------------------------------
    // Quaternion -> rotation matrix
    //--------------------------------------------------------

    Eigen::Matrix3f quaternionRotation(
        const geometry_msgs::msg::Quaternion &q)
    {
        Eigen::Quaternionf quat(
            q.w,
            q.x,
            q.y,
            q.z);

        return quat.normalized().toRotationMatrix();
    }

    //--------------------------------------------------------
    // Member variables
    //--------------------------------------------------------

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr
        pose_sub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        cloud_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        global_cloud_pub_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
        robot_marker_pub_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud_;

    std::mutex mutex_;

    geometry_msgs::msg::Pose current_pose_;

    bool initialized_;

    double last_keyframe_x_;
    double last_keyframe_y_;
    double last_keyframe_yaw_;

    double current_x_;
    double current_y_;
    double current_z_;
    double current_yaw_;

    double keyframe_distance_;
    double keyframe_angle_deg_;
    double voxel_size_;

    //--------------------------------------------------------
    // Functions implemented in Part 2
    //--------------------------------------------------------

    void orbPoseCallback(
        const geometry_msgs::msg::Pose::SharedPtr msg);

    void cloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void addKeyframe();

    void publishCloud();

    void publishRobotMarker();
};


//--------------------------------------------------------
// ORB Pose callback
//--------------------------------------------------------

void MapBuilder::orbPoseCallback(
    const geometry_msgs::msg::Pose::SharedPtr msg)
{
    //RCLCPP_INFO(get_logger(),"Received pose");
    std::lock_guard<std::mutex> lock(mutex_);

    current_pose_ = *msg;

    current_x_ = msg->position.x;
    current_y_ = msg->position.y;
    current_z_ = msg->position.z;

    current_yaw_ = quaternionYaw(msg->orientation);

    if(!initialized_)
    {
        last_keyframe_x_ = current_x_;
        last_keyframe_y_ = current_y_;
        last_keyframe_yaw_ = current_yaw_;

        initialized_ = true;
    }

    publishRobotMarker();
}

////////////////////////////////////////////////////////////

//--------------------------------------------------------
// PointCloud callback
//--------------------------------------------------------

void MapBuilder::cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    //RCLCPP_INFO(get_logger(),"Received cloud");
    if(!initialized_)
        return;

    pcl::fromROSMsg(
        *msg,
        *current_cloud_);
    
    // RCLCPP_INFO(
    // get_logger(),
    // "Current cloud size = %lu",
    // current_cloud_->size());

    double dx =
        current_x_ - last_keyframe_x_;

    double dy =
        current_y_ - last_keyframe_y_;

    double distance =
        std::sqrt(dx*dx + dy*dy);

    double angle =
        std::fabs(
            normalizeAngle(
                current_yaw_ -
                last_keyframe_yaw_));

    if(global_cloud_->empty()) {
        //RCLCPP_INFO(get_logger(), "Creating first keyframe");
        addKeyframe();
        last_keyframe_x_ = current_x_;
        last_keyframe_y_ = current_y_;
        last_keyframe_yaw_ = current_yaw_;
        return;
    }

    if(distance < keyframe_distance_ && angle < keyframe_angle_deg_ * M_PI / 180.0) {
        return;
    }
    addKeyframe();
    last_keyframe_x_ = current_x_;
    last_keyframe_y_ = current_y_;
    last_keyframe_yaw_ = current_yaw_;
}

////////////////////////////////////////////////////////////

//--------------------------------------------------------
// Add one keyframe to map
//--------------------------------------------------------

void MapBuilder::addKeyframe()
{
    //RCLCPP_INFO(get_logger(), ">>> Enter addKeyframe");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

    //***********************************************

    Eigen::Matrix3f R_world = quaternionRotation(current_pose_.orientation);
    Eigen::Matrix3f R_cam;
    R_cam <<
        0,  0,  1,
        -1,  0,  0,
        0, -1,  0;
    Eigen::Matrix3f R = R_world * R_cam;

    //***********************************************

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

    T.block<3,3>(0,0)=R;

    T(0,3)=current_x_;
    T(1,3)=current_y_;
    T(2,3)=current_z_;

    pcl::transformPointCloud(
        *current_cloud_,
        *transformed,
        T);

    // RCLCPP_INFO(get_logger(),
    //     "Transform done. Cloud size = %lu",
    //     transformed->size());

    *global_cloud_ += *transformed;

    // RCLCPP_INFO(get_logger(),
    //         "Merged. Global size = %lu",
    //         global_cloud_->size());

    //----------------------------------------------------
    // voxel filter
    //----------------------------------------------------

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;

    voxel.setInputCloud(global_cloud_);

    voxel.setLeafSize(
        voxel_size_,
        voxel_size_,
        voxel_size_);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    voxel.filter(*filtered);

    // RCLCPP_INFO(get_logger(),
    //         "Voxel done. Filtered size = %lu",
    //         filtered->size());

    global_cloud_ = filtered;

    publishCloud();

    // RCLCPP_INFO(
    //     get_logger(),
    //     "Keyframe added. Map contains %lu points.",
    //     global_cloud_->size());
}

void MapBuilder::publishCloud()
{
    // RCLCPP_INFO(
    // get_logger(),
    // "Publishing cloud with %lu points",
    // global_cloud_->size());
    
    sensor_msgs::msg::PointCloud2 msg;

    pcl::toROSMsg(*global_cloud_, msg);
    msg.width = global_cloud_->size();
    msg.height = 1;
    msg.is_dense = false;

    msg.header.stamp = now();

    // Change this if your map frame has a different name
    msg.header.frame_id = "map";

    // RCLCPP_INFO(
    // get_logger(),
    // "width=%u height=%u row_step=%u point_step=%u data=%lu",
    // msg.width,
    // msg.height,
    // msg.row_step,
    // msg.point_step,
    // msg.data.size());

    global_cloud_pub_->publish(msg);
}

void MapBuilder::publishRobotMarker()
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = now();

    marker.ns = "robot";
    marker.id = 0;

    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose = current_pose_;

    marker.scale.x = 0.30;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;

    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    robot_marker_pub_->publish(marker);
}

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);

    rclcpp::spin(
        std::make_shared<MapBuilder>());

    rclcpp::shutdown();

    return 0;
}