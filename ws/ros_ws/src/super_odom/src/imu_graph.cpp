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

#include<nav_msgs/Odometry.h>

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

double init_time = 0.0;

class IMU_Graph{
	public:
		IMU_Graph(ros::NodeHandle nh){
			this->nh = nh;
			initializeSubsAndPubs();
			initializeFactorGraph();
		}

		void initializeSubsAndPubs(){
			VIO_Sub = nh.subscribe("/vins_estimator/odometry", 1, &IMU_Graph::voCallback, this);
			IMU_Sub = nh.subscribe("/imu0", 1, &IMU_Graph::imuCallback, this);

			IMU_Pub = nh.advertise<geometry_msgs::PoseStamped>("imu_graph/pose",1);
			// imuTimer = nh.createTimer(ros::Duration(0.1), &IMU_Graph::timerCallback);
		}



		void voCallback(const nav_msgs::Odometry &msg){
			// TODO: Handle coorindate transformations
            // if (!accel_init_done) {return;}

			auto frame_id = msg.header.frame_id;

			auto vo_msg = msg.pose.pose;

			gtsam::Pose3 voPrior = gtsam::Pose3(gtsam::Rot3::Quaternion(vo_msg.orientation.w, \
                                            vo_msg.orientation.x, vo_msg.orientation.y\
                                            , vo_msg.orientation.z),
                                            gtsam::Point3(vo_msg.position.x, vo_msg.position.y,vo_msg.position.z));

        
            noiseModel::Isotropic::shared_ptr pose_correction_noise = noiseModel::Isotropic::Sigma(6, 1.5);
	
            auto huberPrior = noiseModel::Robust::Create(
                noiseModel::mEstimator::Cauchy::Create(1.0), pose_correction_noise);
    
			double timestep = msg.header.stamp.toSec();
			cout <<"timestep: "<<  timestep - init_time << endl;


			if (frame == 0){
				
				cout << "yo" << endl;
				do_nominal_init();
				init_time = timestep;
				accel_init_done = true;
			
				newTimestamps[X(0)] = timestep - init_time;
				newTimestamps[V(0)] = timestep - init_time;
				newTimestamps[B(0)] = timestep - init_time;
				
				}
			else {
				std::cout << "Add IMU Factor" << std::endl;
				// Add Imu Factor
				CombinedImuFactor imufac = create_imu_factor(timestep - init_time);
				// cout << "created imufac" <<endl;
				graph.add(imufac);
				// cout << frame << endl;
				prop_state = preintegrated->predict(prev_state, prev_bias);
				cout << prop_state.v() << endl;
				initialEstimate.insert(X(frame), prop_state.pose());
				initialEstimate.insert(V(frame), prop_state.v());
				initialEstimate.insert(B(frame), prev_bias);   
				// cout << "Added IMU Factor" << endl;
				newTimestamps[X(frame)] = timestep - init_time;
				newTimestamps[V(frame)] = timestep - init_time; 
				newTimestamps[B(frame)] = timestep - init_time;

				// TODO: Change this to priors coming from VO


                gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(frame), voPrior, pose_correction_noise);

				graph.add(pose_factor);
				
				std::cout << "IMU CALLBACK" << std::endl;

                	

			// }

            smootherISAM2.update(graph, initialEstimate,newTimestamps);
            
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

            
            // cout << "in VIO callback" << endl;
            sendTfs(timestep - init_time);
			}
            frame ++;

            
		}

		void imuCallback(const sensor_msgs::Imu &imu_msg){
			// std::cout << "FDSAFDSA" << std::endl;

			if (!accel_init_done) {return;}
		    // ROS_INFO("imuCallback");
			geometry_msgs::Vector3 aV = imu_msg.angular_velocity;
			geometry_msgs::Vector3 lA = imu_msg.linear_acceleration;
			Vector3 measuredAcc(lA.x,lA.y,lA.z);
			Vector3 measuredOmega(aV.x,aV.y,aV.z);
			
			double timestep = imu_msg.header.stamp.toSec();
			// if (!accel_init_done){
			// 	cout << "yo" << endl;
			// 	do_nominal_init();
			// 	init_time = timestep;
			// }
			

			
			imu_times.push_back(timestep - init_time);
			imu_linaccs.push_back(measuredAcc);
			imu_angvel.push_back(measuredOmega);
		}

	private:


        void do_nominal_init() {
			ROS_INFO("Nominal Init");

			// Pose prior 
			auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
				(Vector(6) << Vector3::Constant(0.1), Vector3::Constant(1.0)).finished());
			graph.add(PriorFactor<Pose3>(X(0), priorPose, priorPoseNoise));
			initialEstimate.insert(X(0), priorPose);

			//Velocity Prior 
			auto velnoise = noiseModel::Diagonal::Sigmas(Vector3(1.0, 1.0,1.0));
			priorVelocity = Vector3(0.0, 0.0, 0.0);
			graph.add(PriorFactor<Vector3>(V(0), priorVelocity, velnoise));
			initialEstimate.insert(V(0), priorVelocity);
			
			//Bias Prior
			auto biasnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(50.0));
			graph.addPrior<imuBias::ConstantBias>(B(0), priorBias, biasnoise);
			initialEstimate.insert(B(0), priorBias);
			
			prev_state = gtsam::NavState(priorPose, priorVelocity);
			// cout << "prev_state" << prev_state << endl;
			prop_state = prev_state;
			prev_bias = priorBias;

			accel_init_done = true;
		}

		// Defining noise model for the IMU, where p is where this stuff is stored
		std::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imuParams() {

			// We use the sensor specs to build the noise model for the IMU factor.
			double accel_noise_sigma = 1.0;
			double gyro_noise_sigma = 1.0;
			double accel_bias_rw_sigma = 0.0001;
			double gyro_bias_rw_sigma = 0.0001;
			Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
			Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
			Matrix33 integration_error_cov =
				I_3x3 * .0001;  // error committed in integrating position from velocities
			Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
			Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
			Matrix66 bias_acc_omega_int =
				I_6x6 * .0001;  // error in the bias used for preintegration
			gtsam::Pose3 body_IMU_TF = Pose3(Rot3::Quaternion(0.013489769078155169, 0.8297116036556285, -0.009124618852643316, 0.5579546775681852), Point3(0.0,0.0,0.0));
			// gtsam::Pose3 body_IMU_TF = Pose3(Rot3::Ypr(0.0,0.0,0.0), Point3(0.0,0.0,0.0));
			
			auto p = PreintegratedCombinedMeasurements::Params::MakeSharedU(9.81);
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


		CombinedImuFactor create_imu_factor(double updatetime) {
			// cout << "create imu factor " << endl;
			int imucompound = 0;
			// This will add to the pre-integration through each time step until it reaches the timestep of the camera
			while(imu_times.size() > 1 && imu_times.at(1) <= updatetime) {
				double dt = imu_times.at(1) - imu_times.at(0); // new dt
				if (dt >= 0) {
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

		void initializeFactorGraph(){
			ROS_INFO("Initializing IMU Factor Graph");
				
			//SET ISAM2 PARAMS
			ISAM2Params parameters;
			double lag = 2.0;
			parameters.relinearizeThreshold = 0.003; // Set the relin threshold to zero such that the batch estimate is recovered
			parameters.relinearizeSkip = 1; // Relinearize every time
			smootherISAM2 = IncrementalFixedLagSmoother(lag, parameters);
			IMUparams = imuParams(); // Defined in last function
			preintegrated = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(IMUparams, priorBias);

			priorPose = Pose3(Rot3::Quaternion(0.013489769078155169, 0.8297116036556285, -0.009124618852643316, 0.5579546775681852), Point3(0,0,0));
			cout << "priorPose" << priorPose << endl;

			accel_init_done = false;
			// estimatorInit = false;
			frame = 0;

			
		}

		void sendTfs(double timestep){
		    tf::Transform transform;
    		tf::Quaternion q;


			auto t = prev_state.pose().translation();
			auto r = prev_state.pose().rotation();
			transform.setOrigin(tf::Vector3(t(0), t(1), t(2)));
			q.setRPY(r.roll(), r.pitch(), r.yaw());
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time(timestep), "world", "imu"));
		}
		ros::Subscriber VIO_Sub;
		ros::Subscriber IMU_Sub;
		ros::Publisher IMU_Pub;
		ros::NodeHandle nh;

		tf::TransformBroadcaster br;

		// IMU Parameters are the same as from the VIO 
    	std::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> IMUparams;

		std::shared_ptr<gtsam::PreintegrationType> preintegrated;
		std::deque<double> imu_times;
		std::deque<gtsam::Vector3> imu_linaccs;
		std::deque<gtsam::Vector3> imu_angvel;
		std::deque<gtsam::Vector3> imu_orientation;

		gtsam::Pose3 priorPose; 
		gtsam::Vector3 priorVelocity;
		gtsam::imuBias::ConstantBias priorBias; 
		gtsam::NavState prev_state;
		gtsam::NavState prop_state;
		gtsam::imuBias::ConstantBias prev_bias;

		bool accel_init_done;
    	int frame;


		//Initilize GTSAM Variables
		gtsam::IncrementalFixedLagSmoother smootherISAM2;
		gtsam::FixedLagSmoother::KeyTimestampMap newTimestamps;
		gtsam::NonlinearFactorGraph graph;
		gtsam::Values initialEstimate;
		gtsam::Values currentEstimate;



};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_graph");
	ros::NodeHandle nh("~"); 

	std::cout << "NODE" << std::endl;
	// TODO:
	IMU_Graph node(nh);

	std::cout << "RATE" << std::endl;

	ros::Rate loop_rate(10);
	ros::spin();


	std::cout << "INIT" << std::endl;

	return 0;
}