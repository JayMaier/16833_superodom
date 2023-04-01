#ifndef STER_h
#define STER_h

//Some Ros Packages
#include <ros/package.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>


#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include "parameters.h"
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
// #include <periodic_factor_graph/CameraMeasurement.h>
#include <pcl/common/centroid.h>
#include <sensor_msgs/Imu.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gazebo_msgs/LinkStates.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <super_odom/CameraMeasurement.h>
#include <super_odom/FeatureMeasurement.h>
// #include <periodic_factor_graph/msg/CameraMeasurement.msg>
// #include <periodic_factor_graph/msg/FeatureMeasurement.msg>


class VO_IMU_ISAM2
{
public:
    
    VO_IMU_ISAM2(ros::NodeHandle &nodehandle, image_transport::ImageTransport &imagehandle);
    ~VO_IMU_ISAM2();

    //Visualization Paramters
    // bool visualize;    
    // bool initialized;
    // cv::Mat debug_image;
    nav_msgs::Path pathGT;
    nav_msgs::Path pathOPTI;

    // //State of Robot
    gtsam::Pose3 priorPose; 
    gtsam::Vector3 priorVelocity;
    gtsam::imuBias::ConstantBias priorBias; 
    gtsam::NavState prev_state;
    gtsam::NavState prop_state;
    gtsam::imuBias::ConstantBias prev_bias;

    gtsam::Pose3 gtPose = gtsam::Pose3();
    gtsam::Pose3 imuPose = gtsam::Pose3();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr landmark_cloud_msg_ptr;
    // double phase;

    //Counters 
    int landmark;
    int frame;
    int bias;
    double prevAV;
    int loopKeyDown;
    int loopKeyUp;
    // bool estimatorInit;
    bool accel_init_done;
    

    // int lastUp;
    // int lastDown;
    gtsam::Pose3 startUp;
    gtsam::Pose3 startDown;

    // double begin;
    // double graphError;
    std::vector<int> landmarkOffsets;
    std::vector<int> landmarkIDs;
    std::map<int, std::pair<double,double>> idMap;
    
    //Initilize GTSAM Variables
    gtsam::IncrementalFixedLagSmoother smootherISAM2;
    gtsam::FixedLagSmoother::KeyTimestampMap newTimestamps;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimate;
    gtsam::Values currentEstimate;

    //Initialize Variables
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> IMUparams;
    std::shared_ptr<gtsam::PreintegrationType> preintegrated;
    std::deque<double> imu_times;
    std::deque<gtsam::Vector3> imu_linaccs;
    std::deque<gtsam::Vector3> imu_angvel;
    std::deque<gtsam::Vector3> imu_orientation;

    // Set up random noise generators
    std::default_random_engine generator;
    std::normal_distribution<double> distributionVO{0.0,0.0};
    std::normal_distribution<double> distributionIMU{0.0,0.0};
    


private:

    // Some Helper Functions
    void initializeSubsAndPubs();
    void initializeFactorGraph();
    void sendTfs(double timestep);
    void do_accel_init();
    void do_nominal_init();
    gtsam::CombinedImuFactor create_imu_factor(double updatetime);
    gtsam::Point3 triangulateFeature(super_odom::FeatureMeasurement feature);
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imuParams();
     
    // Ros Subscribers
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    ros::Subscriber imuSub;
    ros::Subscriber gazSub;
    ros::Subscriber camSub;
    ros::Subscriber camSub2;
    ros::Subscriber vioSub;
    ros::Subscriber imuodomSub;

    //Ros Publishers
    image_transport::Publisher debug_pub;
    ros::Publisher pose_pub;
    ros::Publisher pathOPTI_pub;
    ros::Publisher pathGT_pub;
    ros::Publisher point_pub;
    ros::Publisher pub_track_length;
    ros::Publisher pub_track;
    ros::Publisher vel_pub;
    
    //Ros Callbacks
    void camCallback(const super_odom::CameraMeasurementPtr& camera_msg);
    void imuCallback(const sensor_msgs::Imu &imu_msg);


     
     
};


#endif