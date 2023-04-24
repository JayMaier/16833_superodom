#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <VO_IMU_ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <super_odom/CameraMeasurement.h>
#include <super_odom/FeatureMeasurement.h>
#include "parameters.h"
// #include <periodic_factor_graph/msg/CameraMeasurement.msg>
// #include <periodic_factor_graph/msg/FeatureMeasurement.msg>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <chrono> 
using namespace std::chrono; 
  
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>


using namespace cv;
using namespace std;
using namespace gtsam;
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;
using symbol_shorthand::L;

int estimatorInit;

// Initializes the factor graphs 
VO_IMU_ISAM2::VO_IMU_ISAM2(ros::NodeHandle &nodehandle, image_transport::ImageTransport &imagehandle):nh(nodehandle),it(imagehandle){
    initializeSubsAndPubs();
    initializeFactorGraph();
}

// Defining noise model for the IMU, where p is where this stuff is stored
std::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> VO_IMU_ISAM2::imuParams() {

  // We use the sensor specs to build the noise model for the IMU factor.
  double accel_noise_sigma = 2.0;
  double gyro_noise_sigma = 2.0;
  double accel_bias_rw_sigma = 0.005;
  double gyro_bias_rw_sigma = 0.005;
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
  Matrix33 integration_error_cov =
      I_3x3 * .0001;  // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
  Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
  Matrix66 bias_acc_omega_int =
      I_6x6 * .0001;  // error in the bias used for preintegration
  gtsam::Pose3 body_IMU_TF = Pose3(Rot3::Ypr(0.0,-1.57,0.0), Point3(0.0,0.0,0.0));
  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedU(gravMag);
  // PreintegrationBase params:
  p->accelerometerCovariance =
      measured_acc_cov;  // acc white noise in continuous
  p->integrationCovariance =
      integration_error_cov;  // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance =
      measured_omega_cov;  // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov;      // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov;  // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_int;
  p->body_P_sensor = body_IMU_TF;

  return p;
}

void VO_IMU_ISAM2::initializeSubsAndPubs(){
    ROS_INFO("Initializing Subscribers and Publishers");
    point_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("landmark_point_cloud", 10);
    pathOPTI_pub = nh.advertise<nav_msgs::Path>("vo/pathOPTI", 1);
    pathGT_pub = nh.advertise<geometry_msgs::PoseStamped>("vo/pathGT", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("vo/pose", 1);

    // ros::Duration(0.5).sleep();
    imuSub = nh.subscribe("/imu0", 1, &VO_IMU_ISAM2::imuCallback, this);
    camSub = nh.subscribe("/features", 1, &VO_IMU_ISAM2::camCallback, this);
    }

VO_IMU_ISAM2::~VO_IMU_ISAM2 () {} 

Point3 VO_IMU_ISAM2::triangulateFeature(super_odom::FeatureMeasurement feature){ 

    double d = feature.u0 - feature.u1;
    cout << "d " << d << endl;
    cout << "fx " << fx << endl;
    cout << "baseline " << baseline << endl;
    double z = fx*baseline/d;
    // cout << "z " << z << endl;
    double x = (feature.u0 - cx)*z/fx;
    // cout << "x " << x << endl;
    double y = (feature.v0 - cy)*z/fy;

    Point3 camera_point = Point3(x,y,z); 
    cout << "camera_point" << camera_point << endl;
    Point3 body_point = bodyToSensor.transformFrom(camera_point);
    cout << "body_point" << body_point << endl;
    Point3 world_point = prop_state.pose().transformFrom(body_point);
    cout << "world_point" << world_point << endl;
    return world_point;
}

void VO_IMU_ISAM2::camCallback(const super_odom::CameraMeasurementPtr& camera_msg){

    cout << "camCallback" << endl;
    vector<super_odom::FeatureMeasurement> feature_vector = camera_msg->features;
    double timestep = camera_msg->header.stamp.toSec();
    auto gaussian = noiseModel::Isotropic::Sigma(3, 6.0);

    if (!accel_init_done) {return;}

    auto huber = noiseModel::Robust::Create(
    noiseModel::mEstimator::GemanMcClure::Create(2.0), gaussian);

    if (camera_msg->section.data == 2 && estimatorInit){
        auto gaussian = noiseModel::Isotropic::Sigma(3, 10.0);
    }


    // Initialize new pointer for Cal3_S2Stereo
    gtsam::Cal3_S2Stereo::shared_ptr K{new gtsam::Cal3_S2Stereo(fx, fy, 0.0, cx, cy, baseline)};

    noiseModel::Isotropic::shared_ptr prior_landmark_noise = noiseModel::Isotropic::Sigma(3, 3000); 
    auto huberPrior = noiseModel::Robust::Create(
        noiseModel::mEstimator::Cauchy::Create(1.0), prior_landmark_noise);
    
    // cout << timestep << endl;
    // cout << frame << endl;
        // cout << "accel_init_done" << endl;
        if (frame == 0){
            newTimestamps[X(0)] = timestep;
            newTimestamps[V(0)] = timestep;
            newTimestamps[B(0)] = timestep;
            }
        else {
            // cout << "Add IMU Factor" << endl;
            //Add Imu Factor
            CombinedImuFactor imufac = create_imu_factor(timestep);
            // cout << "created imufac" <<endl;
            graph.add(imufac);
            // cout << frame << endl;
            prop_state = preintegrated->predict(prev_state, prev_bias);
            initialEstimate.insert(X(frame), prop_state.pose());
            initialEstimate.insert(V(frame), prop_state.v());
            initialEstimate.insert(B(frame), prev_bias);   
            // cout << "Added IMU Factor" << endl;
            newTimestamps[X(frame)] = timestep;
            newTimestamps[V(frame)] = timestep; 
            newTimestamps[B(frame)] = timestep;

            noiseModel::Isotropic::shared_ptr pose_correction_noise = noiseModel::Isotropic::Sigma(6, 10.0);

            gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(frame), imuPose,
                                                        pose_correction_noise);

            graph.add(pose_factor);

        }

    // cout << "section" << camera_msg->section.data << endl;
    if (camera_msg->section.data != -1){
        // cout << "aqui" << endl;
        // Estimators = # of "views", first line finding what camera view it's in, and saying != to the last one
        // if(std::find(begin(estimators), end(estimators), camera_msg->section.data) != end(estimators) || !estimatorInit){
        if(std::find(estimators.begin(), estimators.end(), camera_msg->section.data) != estimators.end() || !estimatorInit){
            if (!estimatorInit && std::find(begin(estimators), end(estimators), camera_msg->section.data)!= end(estimators) ) {estimatorInit = true;}
            // cout << "made it past these two" << endl;
        // For each feature, feature vector comes from camera msg
        for (int i = 0; i < feature_vector.size(); i++){
            // cout << "in for loop" << endl;
            super_odom::FeatureMeasurement feature = feature_vector[i];

            std::cout << feature_vector[i] << std::endl;
            
            // // u is horizontal, v is vertical
            // // These are checking that initial conditions aren't met and that the two features aren't essentially the same.
            if ( (feature.u0 - feature.u1 ) > 20 || (feature.u0 - feature.u1 ) < 3 || (abs(feature.v0- feature.v1) > 20)){
                cout << "in this if" << endl;
                continue;
            }

            Point3 world_point = triangulateFeature(feature); // finds world coordinate, only used for TF visualization and to initialize first estimate
            cout << world_point << endl;

            int landmark_id = feature.id + (camera_msg->section.data); // Define landmakr id, from feature

            landmarkIDs.push_back(landmark_id);
            int feature_id = landmark_id;

            if (!currentEstimate.exists(L(landmark_id))) {
                // cout << "current estimate doesnt exist" << endl;
                initialEstimate.insert(L(landmark_id), world_point);
                newTimestamps[L(landmark_id)] = timestep;
                graph.add(PriorFactor< Point3>(L(landmark_id), world_point, prior_landmark_noise   ));
            } 
            else {
                // cout << "new timestamps" << endl;
                newTimestamps[L(landmark_id)] = timestep;
            }

            // cout << "generic stereo factor" << endl;
            // Generic Visual Factor, given the readings from the image, the current state, the Landmark, K matrix, and body2sensor transform.
            GenericStereoFactor<Pose3, Point3> visualFactor(StereoPoint2(feature.u0, feature.u1, feature.v0), 
            huber, X(frame), L(landmark_id), K, bodyToSensor);
            // cout << "graph emplace" << endl;
            graph.emplace_shared<GenericStereoFactor<Pose3, Point3> >(visualFactor);
            
        }
        } 
    


    // cout << "smoother" << endl;
    smootherISAM2.update(graph, initialEstimate, newTimestamps);
    // for(size_t i = 1; i < 7; ++i) { // Optionally perform multiple iSAM2 iterations
    //     smootherISAM2.update();
    // }
        
    // cout << "curr estimate" << endl;
    currentEstimate = smootherISAM2.calculateEstimate();
    prev_state =
        gtsam::NavState(currentEstimate.at<Pose3>(X(frame)), currentEstimate.at<Vector3>(V(frame)));
    prev_bias = currentEstimate.at<imuBias::ConstantBias>(B(frame));

    // cout << "resize " << endl;
    graph.resize(0);
    initialEstimate.clear();
    newTimestamps.clear();
    preintegrated->resetIntegrationAndSetBias(prev_bias);
        
    sendTfs(timestep);
    }
    frame ++;

    }

void VO_IMU_ISAM2::initializeFactorGraph(){
    ROS_INFO("Initializing VIO Factor Graph");
        
    //SET ISAM2 PARAMS
    ISAM2Params parameters;
    double lag = 3.0;
    parameters.relinearizeThreshold = 0.003; // Set the relin threshold to zero such that the batch estimate is recovered
    parameters.relinearizeSkip = 1; // Relinearize every time
    smootherISAM2 = IncrementalFixedLagSmoother(lag, parameters);
    IMUparams = imuParams(); // Defined in last function
    preintegrated = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(IMUparams, priorBias);

    priorPose = Pose3(Rot3::Ypr(initYaw,initPitch,initRoll), Point3(initX,initY,initZ));
    cout << "priorPose" << priorPose << endl;

    accel_init_done = false;
    estimatorInit = false;
    frame = 0;
    landmark_cloud_msg_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());    


}

void VO_IMU_ISAM2::imuCallback(const sensor_msgs::Imu &imu_msg){
    // ROS_INFO("imuCallback");
    geometry_msgs::Vector3 aV = imu_msg.angular_velocity;
    geometry_msgs::Vector3 lA = imu_msg.linear_acceleration;
    Vector3 measuredAcc(lA.x,lA.y,lA.z);
    Vector3 measuredOmega(aV.x,aV.y,aV.z);
    
    
    if (!accel_init_done){
        // cout << "yo" << endl;
        do_nominal_init();
    }
    

    double timestep = imu_msg.header.stamp.toSec();
    imu_times.push_back(timestep);
    imu_linaccs.push_back(measuredAcc);
    imu_angvel.push_back(measuredOmega);

}

void VO_IMU_ISAM2::do_accel_init() {
    // ROS_INFO("Accel Init");
    gtsam::Vector3 acc_avg;

    // cout << "imu_size 2: " << imu_times.size() << endl;
    
    if (imu_times.size() < 30){
        return;
    } else {
        for (auto &accel : imu_linaccs) {
            acc_avg += accel;
            // cout << "accel" << accel << endl; 
        }
        acc_avg /= imu_times.size();

        // acc_avg << 0.0, 0.0, gravMag ;
    }
    
    // cout << "Gravity-aligning with accel. vector:\n" << acc_avg << endl;

    gtsam::Vector3 gravity_vec;
    gravity_vec << 0.0, 0.0, gravMag;
    auto initial_att = gtsam::Rot3(
        Eigen::Quaterniond().setFromTwoVectors(acc_avg, gravity_vec));

    // cout << "accel avg" << acc_avg << endl;

    // cout << "init att" << initial_att << endl;
    gtsam::Pose3 initial_pose_(initial_att, gtsam::Point3());
    // cout <<  "Gravity vector after alignment: " << (initial_pose_ * acc_avg) << endl;
    priorPose = priorPose*initial_pose_;

    // cout << "prior velocity " << priorVelocity << endl;
    // cout << "prior Pose " << priorPose << endl;

    // Pose prior 
    auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << Vector3::Constant(0.01), Vector3::Constant(0.01)).finished());
    graph.add(PriorFactor<Pose3>(X(0), priorPose, priorPoseNoise));
    initialEstimate.insert(X(0), priorPose);

    //Velocity Prior 
    auto velnoise = noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 0.01));
    graph.add(PriorFactor<Vector3>(V(0), priorVelocity, velnoise));
    initialEstimate.insert(V(0), priorVelocity);
     
    //Bias Prior
    auto biasnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
    graph.addPrior<imuBias::ConstantBias>(B(0), priorBias, biasnoise);
    initialEstimate.insert(B(0), priorBias);
    
    prev_state = gtsam::NavState(priorPose, priorVelocity);
    cout << "prev_state" << prev_state << endl;
    prop_state = prev_state;
    prev_bias = priorBias;

    accel_init_done = true;
}

// MAKE SURE TO CHECK NOISE
void VO_IMU_ISAM2::do_nominal_init() {
    ROS_INFO("Nominal Init");

    // Pose prior 
    auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << Vector3::Constant(0.01), Vector3::Constant(0.01)).finished());
    graph.add(PriorFactor<Pose3>(X(0), priorPose, priorPoseNoise));
    initialEstimate.insert(X(0), priorPose);

    //Velocity Prior 
    auto velnoise = noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 0.01));
    graph.add(PriorFactor<Vector3>(V(0), priorVelocity, velnoise));
    initialEstimate.insert(V(0), priorVelocity);
     
    //Bias Prior
    auto biasnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
    graph.addPrior<imuBias::ConstantBias>(B(0), priorBias, biasnoise);
    initialEstimate.insert(B(0), priorBias);
     
    prev_state = gtsam::NavState(priorPose, priorVelocity);
    // cout << "prev_state" << prev_state << endl;
    prop_state = prev_state;
    prev_bias = priorBias;

    accel_init_done = true;
}

CombinedImuFactor VO_IMU_ISAM2::create_imu_factor(double updatetime) {
    // cout << "create imu factor " << endl;
    int imucompound = 0;
    // This will add to the pre-integration through each time step until it reaches the timestep of the camera
    while(imu_times.size() > 1 && imu_times.at(1) <= updatetime) {
        double dt = imu_times.at(1) - imu_times.at(0); // new dt
        if (dt > 0) {
            // Preintegrate this measurement!
            preintegrated->integrateMeasurement(imu_linaccs.at(0), imu_angvel.at(0), dt); // adds to the preintegration object
        }
        // erases beginning of the vectors, to then integrate the next two points
        imu_angvel.erase(imu_angvel.begin());
        imu_linaccs.erase(imu_linaccs.begin());
        imu_times.erase(imu_times.begin());
        imucompound++;
    }
    // Checking the timing work out, if not, perform one more integration
    double dt_f = updatetime - imu_times.at(0);
    if (dt_f > 0) {
        // Preintegrate this measurement!
        preintegrated->integrateMeasurement(imu_linaccs.at(0), imu_angvel.at(0), dt_f);
        imu_times.at(0) = updatetime;
        imucompound++;
    }
    // Follows documentation for imu_integration in gtsam
    auto preint_imu_combined =
          dynamic_cast<const PreintegratedCombinedMeasurements&>(
              *preintegrated);


    // cout << "got down here" << endl;
    // cout << "frame" << frame << endl;
    // Creates factor given gtsam documentation, returns it
    CombinedImuFactor imufac(X(frame - 1), V(frame - 1), X(frame),
                                V(frame), B(frame - 1), B(frame),
                                preint_imu_combined);
    // cout << "got here too" << endl;
    return imufac;

}

void VO_IMU_ISAM2::sendTfs(double timestep){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    Point3 t;  
    Rot3 r;

    //Send gtsam tf
    t = prev_state.pose().translation();
    r = prev_state.pose().rotation();
    transform.setOrigin(tf::Vector3(t(0), t(1), t(2)));
    q.setRPY(r.roll(), r.pitch(), r.yaw());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time(timestep), "world", "Body_EST_VIO"));

    // //Send ground truth tf
    // t = gtPose.translation();
    // r = gtPose.rotation();
    // transform.setOrigin(tf::Vector3(t(0), t(1), t(2)));
    // q.setRPY(r.roll(), r.pitch(), r.yaw());
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time(timestep), "world", "True_Pose"));
  
    // //Send camera tf
    // t = bodyToSensor.translation();
    // r = bodyToSensor.rotation();
    // transform.setOrigin(tf::Vector3(t(0), t(1), t(2)));
    // q.setRPY(r.roll(), r.pitch(), r.yaw());
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time(timestep), "Body_EST", "LCAM"));
    // transform.setOrigin(tf::Vector3(0,baseline,0));
    // q.setRPY(0,0,0);
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time(timestep), "LCAM", "RCAM"));


    //Publish landmark PointCloud message (in world frame)
    landmark_cloud_msg_ptr->clear();
    landmark_cloud_msg_ptr->header.frame_id = "world";
    landmark_cloud_msg_ptr->height = 1;
    gtsam::Point3 point;
    gtsam::Point3 point2;
    for (const int i: landmarkIDs) {
        if (!currentEstimate.exists(L(i))) {continue;}
        else{
            point = currentEstimate.at<Point3>(L(i));  
            pcl::PointXYZRGB pcl_world_point = pcl::PointXYZRGB(200,100,0);
            if (i < 200000){
                pcl_world_point = pcl::PointXYZRGB(200,0,100);
            } else if (i > 300000){
                pcl_world_point = pcl::PointXYZRGB(100,0,200);
            }
 
            pcl_world_point.x = point.x();
            pcl_world_point.y = point.y();
            pcl_world_point.z = point.z();
            landmark_cloud_msg_ptr->points.push_back(pcl_world_point);
        }
    }

    landmark_cloud_msg_ptr->width = landmark_cloud_msg_ptr->points.size();
    point_pub.publish(landmark_cloud_msg_ptr);
 
     

    //Publish GT Trajectory
    geometry_msgs::PoseStamped poseStamped;
    // poseStamped.header.frame_id="/world";
    // poseStamped.header.stamp = ros::Time(timestep);
    // poseStamped.pose.position.x =  gtPose.x();
    // poseStamped.pose.position.y = gtPose.y();
    // poseStamped.pose.position.z = gtPose.z();
    // pathGT.header.frame_id = "world";
    // r = gtPose.rotation();
    // q.setRPY(r.roll(), r.pitch(), r.yaw());
    // poseStamped.pose.orientation.x = q.x();
    // poseStamped.pose.orientation.y = q.y();
    // poseStamped.pose.orientation.z = q.z();
    // poseStamped.pose.orientation.w = q.w();
    // // pathGT.poses.push_back(poseStamped);
    // pathGT.header.stamp = poseStamped.header.stamp;
    // pathGT_pub.publish(poseStamped);
 
    gtsam::Pose3 pose = prev_state.pose(); 
    poseStamped.pose.position.x =  pose.x();
    poseStamped.pose.position.y = pose.y();
    poseStamped.pose.position.z = pose.z();
    pathOPTI.header.frame_id = "world";
    pathOPTI.poses.push_back(poseStamped);
    pathOPTI.header.stamp = poseStamped.header.stamp;
    pathOPTI_pub.publish(pathOPTI); 

    //Publish SLAM Trajectory
    // gtsam::Pose3 pose;
    // for (int i = 0; i <= frame-1; i ++){
         
    //     if (!currentEstimate.exists(X(i))) {continue;}
    //     pose = currentEstimate.at<Pose3>(X(i)); 
    //     // cout << pose.x() << endl;  
    //     poseStamped.pose.position.x =  pose.x();
    //     poseStamped.pose.position.y = pose.y();
    //     poseStamped.pose.position.z = pose.z();
    //     pathOPTI.header.frame_id = "world";
    //     pathOPTI.poses.push_back(poseStamped);
    //     pathOPTI.header.stamp = poseStamped.header.stamp;
    //     pathOPTI_pub.publish(pathOPTI); 
    // }
     
    
    //Publish Pose
    r = pose.rotation();
    q.setRPY(r.roll(), r.pitch(), r.yaw());
    poseStamped.pose.orientation.x = q.x();
    poseStamped.pose.orientation.y = q.y();
    poseStamped.pose.orientation.z = q.z();
    poseStamped.pose.orientation.w = q.w();
    pose_pub.publish(poseStamped);
    //pathOPTI.poses.clear();
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "loc");
    ros::NodeHandle nh("~"); 
    readParameters(nh);
    // ros::Duration(0.5).sleep();

    // cout << "fx " << fx<< endl;
    image_transport::ImageTransport it(nh);
    VO_IMU_ISAM2 node(nh,it);
 
    ros::Rate loop_rate(100);
    ros::spin();

    return 0;
}