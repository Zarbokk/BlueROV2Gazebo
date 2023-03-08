//
// Created by tim-linux on 22.12.21.
//
#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "commonbluerovmsg/staterobotforevaluation.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

ros::Publisher publisherGazeboData, publisherStateEstimationData;

Eigen::Matrix4d transformationFromGazeboToNED = Eigen::Matrix4d::Identity();
bool currentGT = true;

Eigen::Vector3d getRollPitchYaw(Eigen::Quaterniond quat) {
    tf2::Quaternion tmp(quat.x(), quat.y(), quat.z(), quat.w());
    tf2::Matrix3x3 m(tmp);
    double r, p, y;
    m.getRPY(r, p, y);
    Eigen::Vector3d returnVector(r, p, y);
    return returnVector;
}

Eigen::Quaterniond getQuaternionFromRPY(double roll, double pitch, double yaw) {
//        tf2::Matrix3x3 m;
//        m.setRPY(roll,pitch,yaw);
//        Eigen::Matrix3d m2;
    tf2::Quaternion qtf2;
    qtf2.setRPY(roll, pitch, yaw);
    Eigen::Quaterniond q;
    q.x() = qtf2.x();
    q.y() = qtf2.y();
    q.z() = qtf2.z();
    q.w() = qtf2.w();

//        q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
//            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
//            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    return q;
}


void callbackGazebo(const gazebo_msgs::ModelStates::ConstPtr &msg) {
    if(currentGT){
        currentGT = false;
        return;
    }
    currentGT = true;

    int indexName = 0;
    for (auto &name: msg->name) {
        if (name == "uuv_bluerov2_heavy") {
            break;
        }
        indexName++;
    }



    Eigen::Quaterniond rotTMP(msg->pose[indexName].orientation.w, msg->pose[indexName].orientation.x, msg->pose[indexName].orientation.y,
                              msg->pose[indexName].orientation.z);

    Eigen::Matrix4d transformationOfPose = Eigen::Matrix4d::Zero();
    transformationOfPose.block<3, 3>(0, 0) = rotTMP.toRotationMatrix();
    transformationOfPose(0, 3) = msg->pose[indexName].position.x;
    transformationOfPose(1, 3) = msg->pose[indexName].position.y;
    transformationOfPose(2, 3) = msg->pose[indexName].position.z;
    transformationOfPose(3, 3) = 1;

    transformationOfPose = transformationFromGazeboToNED*transformationOfPose;


    Eigen::Quaterniond newRot(transformationOfPose.block<3, 3>(0, 0));
    Eigen::Vector3d rpy= getRollPitchYaw(newRot);


    commonbluerovmsg::staterobotforevaluation publishingMsg;
    publishingMsg.header.stamp = ros::Time::now();
    publishingMsg.xPosition = transformationOfPose(0, 3);
    publishingMsg.yPosition = transformationOfPose(1, 3);
    publishingMsg.zPosition = transformationOfPose(2, 3);
    publishingMsg.roll = rpy[0]-M_PI;
    if(publishingMsg.roll<-M_PI){
        publishingMsg.roll+=2*M_PI;
    }
    publishingMsg.pitch = rpy[1];
    publishingMsg.yaw = rpy[2];
    publisherGazeboData.publish(publishingMsg);
}

void callbackEKF(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {

    commonbluerovmsg::staterobotforevaluation publishingMsg;
    publishingMsg.header.stamp = ros::Time::now();
    publishingMsg.xPosition = msg->pose.pose.position.x;
    publishingMsg.yPosition = msg->pose.pose.position.y;
    publishingMsg.zPosition = msg->pose.pose.position.z;
    Eigen::Quaterniond rotTMP(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                              msg->pose.pose.orientation.z);

    Eigen::Vector3d rpy = getRollPitchYaw(rotTMP);
    publishingMsg.roll = rpy[0];
    publishingMsg.pitch = rpy[1];
    publishingMsg.yaw = rpy[2];
    publisherStateEstimationData.publish(publishingMsg);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "gazebo2position");
    ros::start();
    ros::NodeHandle n_;

    Eigen::AngleAxisd rotX180(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
    //Eigen::AngleAxisd rotz90(90.0 / 180.0 * 3.14159, Eigen::Vector3d(0, 0, 1));
    transformationFromGazeboToNED.block<3, 3>(0, 0) = rotX180.toRotationMatrix();//rotz90.toRotationMatrix()*rotX180.toRotationMatrix();
    transformationFromGazeboToNED(3, 3) = 1;


    publisherGazeboData = n_.advertise<commonbluerovmsg::staterobotforevaluation>("positionGT", 1);
    publisherStateEstimationData = n_.advertise<commonbluerovmsg::staterobotforevaluation>("positionEstimated", 1);
    ros::Subscriber subscriberGazebo = n_.subscribe("gazebo/model_states", 1000, &callbackGazebo);
    ros::Subscriber subscriberStateEstimator = n_.subscribe("publisherPoseEkf", 1000, &callbackEKF);
    ros::spin();


    return (0);
}


