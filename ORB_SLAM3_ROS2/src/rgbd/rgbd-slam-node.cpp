#include <cmath>
#include <iomanip>

#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM) {
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(
    this,"/camera/camera/color/image_raw");

    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(
    this,"/camera/camera/aligned_depth_to_color/image_raw");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/orb_pose", 10); //QoS history depth=10
}

RgbdSlamNode::~RgbdSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdSlamNode::GrabRGBD(
    const ImageMsg::SharedPtr msgRGB,
    const ImageMsg::SharedPtr msgD)
{
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
        cv_ptrD   = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "cv_bridge exception: %s",
                     e.what());
        return;
    }

    // Get ORB-SLAM pose
    Sophus::SE3f Tcw = m_SLAM->TrackRGBD(
        cv_ptrRGB->image,
        cv_ptrD->image,
        Utility::StampToSec(msgRGB->header.stamp));

    // Skip invalid pose
    if(Tcw.matrix().isZero())
        return;

    // ORB-SLAM gives Camera wrt World
    // Need inverse for World wrt Camera
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Vector3f t = Twc.translation();
    Eigen::Matrix3f R = Twc.rotationMatrix();

    //output to topic /orb_pose
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = t.z(); //out x+=fwd
    pose_msg.position.y = -t.x(); //out y+=lft
    pose_msg.position.z = -t.y(); //out z+=up 
    Eigen::Matrix3f R_convert;
    
    R_convert <<
        0, 0, 1,
        -1, 0, 0,
        0,-1, 0;

    Eigen::Matrix3f R_robot = R_convert * R * R_convert.transpose();
    Eigen::Quaternionf q(R_robot);
    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();
    pose_pub_->publish(pose_msg); //publish the message

    // //debugging
    // double roll  = atan2(R_robot(2,1), R_robot(2,2));
    // double pitch = atan2(-R_robot(2,0),
    //                     sqrt(R_robot(2,1)*R_robot(2,1) +
    //                         R_robot(2,2)*R_robot(2,2)));
    // double yaw   = atan2(R_robot(1,0), R_robot(0,0));
    // roll  *= 180.0 / M_PI;
    // pitch *= 180.0 / M_PI;
    // yaw   *= 180.0 / M_PI;

    // // Print pose
    // std::cout << std::fixed << std::setprecision(3)
    //           << "xyz:[" << pose_msg.position.x
    //           << " " << pose_msg.position.y
    //           << " " << pose_msg.position.z
    //           << "]m rpy:[" << roll
    //           << " " << pitch
    //           << " " << yaw
    //           << "]deg"
    //           << std::endl;
}
