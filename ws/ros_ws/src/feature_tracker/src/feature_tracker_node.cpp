#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include "visualization.h"
#include <thread>
#include <mutex>
#include "feature_tracker.h"
#include <feature_tracker/CameraMeasurement.h>
#include <feature_tracker/TrackingInfo.h>
#include <feature_tracker/PhaseFrames.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf/transform_datatypes.h>
#include "opencv2/core.hpp"
#include <ros/console.h>
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
// #include "opencv2/xfeatures2d.hpp"
#include <assert.h> 

using namespace cv;
using namespace std;
//using namespace cv::xfeatures2d;



#define SHOW_UNDISTORTION 0


ros::Publisher pub_image_track1, pub_image_track2, pub_image_track3, feature_pub, pub_image_debug, pub_track1, pub_track2, pub_track3;

FeatureTracker featureTracker1(1);
FeatureTracker featureTracker2(2);
FeatureTracker featureTracker3(3);

FeatureTracker featureTracker4(4);
FeatureTracker featureTracker5(5);

double currPitch = 0.0;
double currPhase = 0.0;

Matrix3d ric[2];
Vector3d tic[2];
deque<sensor_msgs::ImageConstPtr> img0_buf;
deque<sensor_msgs::ImageConstPtr> img1_buf;
deque<double> phase_buf;

std::mutex m_buf;

cv::Mat img0_down;
cv::Mat img1_down;

cv::Mat img0_mid;
cv::Mat img1_mid;

cv::Mat img0_up;
cv::Mat img1_up;

double fx = 476.7030836014194;
double fy = 476.7030836014194;
double cx = 400.5;
double cy = 400.5;
int section = -1;

double baseline = .025;

double currVel = 0.0;


void getStereoPairs(const cv::Mat &imLeftprev, const cv::Mat &imRightprev,
                    const cv::Mat &imLeftcurr, const cv::Mat &imRightcurr,
                                   vector<cv::Point2f> &prevLeftPts, 
                                   vector<cv::Point2f> &prevRightPts,
                                   vector<cv::Point2f> &currLeftPts,
                                   vector<cv::Point2f> &currRightPts){

        if(!prevLeftPts.empty())
        {
            //printf("stereo image; track feature on right image\n");
            vector<uchar> status1, status2;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(imLeftprev, imRightprev, prevLeftPts, prevRightPts, status1, err, cv::Size(21, 21), 3);

            reduceVector(prevLeftPts, status1);
            reduceVector(prevRightPts, status1);
            reduceVector(currLeftPts, status1);

            cv::calcOpticalFlowPyrLK(imLeftcurr, imRightcurr, currLeftPts, currRightPts, status2, err, cv::Size(21, 21), 3);


            reduceVector(prevLeftPts, status2);
            reduceVector(prevRightPts, status2);
            reduceVector(currLeftPts, status2);
            reduceVector(currRightPts, status2);
        }

        assert (prevLeftPts.size() == currRightPts.size());
        
}



void pubTrackImage1(const cv::Mat &imgTrack, const double t, int section)
{
    if (section == -1){
        return;
    }

    std_msgs::Header header;
    header.frame_id = std::to_string(section);
    header.stamp = ros::Time(t);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    pub_image_track1.publish(imgTrackMsg);

}

void pubTrackImage2(const cv::Mat &imgTrack, const double t, int section)
{
    if (section == -1){
        return;
    }

    std_msgs::Header header;
    header.frame_id = std::to_string(section);
    header.stamp = ros::Time(t);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    pub_image_track2.publish(imgTrackMsg);

}

void pubTrackImage3(const cv::Mat &imgTrack, const double t, int section)
{
    if (section == -1){
        return;
    }

    std_msgs::Header header;
    header.frame_id = std::to_string(section);
    header.stamp = ros::Time(t);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    pub_image_track3.publish(imgTrackMsg);

}

void pubStereoFeatures(map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame, const double t, int section){
    
    // Publish features.
    feature_tracker::CameraMeasurementPtr feature_msg_ptr(new feature_tracker::CameraMeasurement);
    feature_msg_ptr->header.stamp = ros::Time(t);
    feature_msg_ptr->section.data = section;
    
    if (section == -1){
        feature_pub.publish(feature_msg_ptr);
        // cout << "in if" << endl;
        return;
    }

    //xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
    int i = 0;
    for (auto const& feature : featureFrame){
        feature_msg_ptr->features.push_back(feature_tracker::FeatureMeasurement());

        feature_msg_ptr->features[i].id = feature.first;
        feature_msg_ptr->features[i].u0 = feature.second[0].second[0]*554.3827128 + 640/2 + .5;
        feature_msg_ptr->features[i].v0 = feature.second[0].second[1]*554.3827128 + 480/2 + .5;
        feature_msg_ptr->features[i].u1 = feature.second[1].second[0]*554.3827128 + 640/2 + .5;
        feature_msg_ptr->features[i].v1 = feature.second[1].second[1]*554.3827128 + 480/2 + .5;

        // feature_msg_ptr->features[i].id = feature.first;
        // feature_msg_ptr->features[i].u0 = feature.second[0].second[0]*476.7030836014194 + 400.5;
        // feature_msg_ptr->features[i].v0 = feature.second[0].second[1]*476.7030836014194 + 400.5;
        // feature_msg_ptr->features[i].u1 = feature.second[1].second[0]*476.7030836014194 + 400.5;
        // feature_msg_ptr->features[i].v1 = feature.second[1].second[1]*476.7030836014194 + 400.5;

        i++;
    }


    feature_pub.publish(feature_msg_ptr);

}

void pubTrackCount1(const int count){
    std_msgs::Int32 msg;
    msg.data = count;
    pub_track1.publish(msg); 
}

void pubTrackCount2(const int count){
    std_msgs::Int32 msg;
    msg.data = count;
    pub_track2.publish(msg); 
}

void pubTrackCount3(const int count){
    std_msgs::Int32 msg;
    msg.data = count;
    pub_track3.publish(msg); 
}

void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push_back(img_msg);
    // cout << "0" << img_msg << endl;
    phase_buf.push_back(currPhase);
    m_buf.unlock();
}


void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push_back(img_msg);
    // cout << "1" << img_msg << endl;
    m_buf.unlock();
}


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
        // cout << "img " << img << endl;

    return img;
}


void feature_frame_push(double t, const cv::Mat &_img, const cv::Mat &_img1, double phase)
{
    
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    TicToc featureTrackerTime;
    double pitch = currPitch;
    
    cv::Mat imgTrack;

    cout << "phase in feature frame push" << phase << endl;
    // cout << "currPhase " << phase << endl;

    if(_img1.empty()){
        std::cout << "in if" << endl;
        featureFrame = featureTracker1.trackImage(t, _img);}
    else{
        if (pitch < -0.15){  // -0.15 //magic number is .43  or .23 or .4  //Fast For -.23
            if (img0_down.empty()){
                img0_down = _img;
                img1_down = _img1;
                //img_match(4);
                 
            }
            cout << "pitch:   " << pitch << "  phase  " << phase << endl;
            if (section == 1){
            pubStereoFeatures(featureTracker1.trackImage(t, _img, _img1),t, 1);
            pubTrackImage1(featureTracker1.getTrackImage(), t, 1);
            pubTrackCount1(featureTracker1.trackedNum);    
            }
            // else{
            //     void release(_img); 	// auto _img.release();
            //     auto _img1.release();
            // // featureFrame = featureTracker1.trackImage(t, _img);
            // // pubTrackCount1(featureTracker1.trackedNum);
            // }
            section = 1;
            // pubStereoFeatures(featureTracker1.trackImage(t, _img, _img1),t, 1);
            // pubTrackImage1(featureTracker1.getTrackImage(), t, 1);
            pubTrackCount1(featureTracker1.trackedNum);
            
        } else if (pitch > -0.15 && pitch < 0.07){ //Fast For -.11 to -.07
            cout << "in phase0 if" << endl;
            if (img0_mid.empty()){
                img0_mid = _img;
                img1_mid = _img1;
                cout << "pitch:   " << pitch << "  phase  " << phase << endl;
            }
            cout << "pitch:   " << pitch << "  phase  " << phase << endl;
            section = 2;
            pubStereoFeatures(featureTracker2.trackImage(t, _img, _img1),t, 2);
            pubTrackImage2(featureTracker2.getTrackImage(), t, 2);
            pubTrackCount2(featureTracker2.trackedNum);
            
        } else if (pitch > 0.07){  // 0.07 //Fast For .1
            if (img0_up.empty()){
                img0_up = _img;
                img1_up = _img1;
                cout << "pitch:   " << pitch << "  phase  " << phase << endl;
                //img_match(5);
                
            }
            cout << "pitch:   " << pitch << "  phase  " << phase << endl;
            section = 3;
            pubStereoFeatures(featureTracker3.trackImage(t, _img, _img1),t, 3);
            pubTrackImage3(featureTracker3.getTrackImage(), t, 3);
            pubTrackCount3(featureTracker3.trackedNum);
        }
        
         else{
            section = -1;
            pubStereoFeatures(featureFrame, t, -1);
            pubTrackImage1(featureTracker1.getTrackImage(), t, -1);
        }
    } 
        

    

}

void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time;
            double phase;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003s sync tolerance
                //cout << time0 << " " << time1 << endl;
       
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop_front();
                    printf("throw img0\n");
                    cout << "throw img0" << endl;
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop_front();
                    printf("throw img1\n");
                    cout << "throw img1" << image0 << endl;
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop_front();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop_front();
                    phase = phase_buf.front();
                    phase_buf.pop_front();
                    // cout << "aft getImageFromMsg" << endl;
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty())                
                feature_frame_push(time, image0, image1, phase);
         
        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop_front();
            }
            m_buf.unlock();
            // if(!image.empty())
            //     estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
   }
}

// void gazCallback(const gazebo_msgs::LinkStates &msgs){
//     int idx = 51;
//     tf::Quaternion q(msgs.pose[idx].orientation.w, msgs.pose[idx].orientation.x, msgs.pose[idx].orientation.y,msgs.pose[idx].orientation.z);
//     tf::Matrix3x3 m(q);
//     double roll, pitch, yaw;
//     m.getRPY(roll, pitch, yaw);
//     currPitch = pitch;
//     cout << pitch << endl;
// }

void desCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currPitch = pitch;
    cout << pitch << endl;
}
// void estCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
//     double roll, pitch, yaw;
//     currPitch = msg->pose.orientation.y;
//     cout << pitch << endl;
// }

void gtCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    cout << "roll " << roll << endl;
    currPhase = roll;
    // cout << currPitch << endl;
}

void phaseCallback(const std_msgs::Float64 &msgs){
    currPhase = msgs.data;
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    currVel = ry;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);
    readParameters(config_file);
 
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        cout << " exitrinsic cam " << i << endl  << ric[i] << endl << tic[i].transpose() << endl;
    }

    featureTracker1.readIntrinsicParameter(CAM_NAMES);
    featureTracker2.readIntrinsicParameter(CAM_NAMES);
    featureTracker3.readIntrinsicParameter(CAM_NAMES);
    featureTracker4.readIntrinsicParameter(CAM_NAMES);
    featureTracker5.readIntrinsicParameter(CAM_NAMES);

   
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    // ros::Subscriber gazSUB = n.subscribe("/gazebo/link_states", 1000, gazCallback);
    ros::Subscriber phaseSUB = n.subscribe("/mobile_robot/phase", 1000, phaseCallback); 
    ros::Subscriber gtSUB = n.subscribe("/mocap_node/Robot_1/pose", 1000, gtCallback);
    ros::Subscriber desSUB = n.subscribe("/des_pos", 1000, desCallback);
    // ros::Subscriber estSUB = n.subscribe("/period_shaky/vo/pose", 1000, estCallback);
    
    //ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());


    pub_image_track1 = n.advertise<sensor_msgs::Image>("feature_img1",1000);
    pub_image_track2 = n.advertise<sensor_msgs::Image>("feature_img2",1000);
    pub_image_track3 = n.advertise<sensor_msgs::Image>("feature_img3",1000);
    
    pub_track1 = n.advertise<std_msgs::Int32>("/track_count1", 1000);
    pub_track2 = n.advertise<std_msgs::Int32>("/track_count2", 1000);
    pub_track3 = n.advertise<std_msgs::Int32>("/track_count3", 1000);
    pub_image_debug = n.advertise<sensor_msgs::Image>("debugging_img",1000);

    feature_pub = n.advertise<feature_tracker::CameraMeasurement>("/features", 3);

    std::thread sync_thread{sync_process};


    ros::spin();
    return 0;
}