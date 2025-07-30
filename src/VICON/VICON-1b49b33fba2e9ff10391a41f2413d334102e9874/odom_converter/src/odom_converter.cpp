#include <odom_converter.h>

namespace odom_converter{

void odom_converter::Init(ros::NodeHandle& n){

    registerSub(n);
    registerPub(n);

}

void odom_converter::registerSub(ros::NodeHandle &n){

    pose_sub_ptr_ = new message_filters::Subscriber<geometry_msgs::PoseStamped> (n, "pose", 100, ros::TransportHints().tcpNoDelay(true));
    twist_sub_ptr_ = new message_filters::Subscriber<geometry_msgs::TwistStamped> (n, "twist", 100, ros::TransportHints().tcpNoDelay(true));

    if(USE_EXACT_TIME_SYNC){

        sync_exact_.reset(new message_filters::Synchronizer<SyncPolicyExact>(SyncPolicyExact(100), *pose_sub_ptr_, *twist_sub_ptr_));
        sync_exact_->registerCallback(boost::bind(&odom_converter::pose_twist_callback, this, _1, _2));

        sync_approximate_.release();

    }else{

        sync_approximate_.reset(new message_filters::Synchronizer<SyncPolicyApproximate>(SyncPolicyApproximate(10), *pose_sub_ptr_, *twist_sub_ptr_));
        sync_approximate_->registerCallback(boost::bind(&odom_converter::pose_twist_callback, this, _1, _2));
        sync_exact_.release();

    }

}


void odom_converter::registerPub(ros::NodeHandle &n){
    odom_pub_ = n.advertise<nav_msgs::Odometry>("converted_odom", 10);
}

void odom_converter::pose_twist_callback(const geometry_msgs::PoseStampedConstPtr& pose_ptr, const geometry_msgs::TwistStampedConstPtr& twist_ptr){
    nav_msgs::Odometry odom;
    odom.header = pose_ptr->header;
    odom.pose.pose = pose_ptr->pose;
    odom.twist.twist = twist_ptr->twist;
    odom_pub_.publish(odom);
}


}