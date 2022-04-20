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
#include "geometry_msgs/Vector3Stamped.h"
#include "thread"
ros::Publisher publisherGazeboData, publisherStateEstimationData;

Eigen::Matrix4d transformationFromGazeboToNED = Eigen::Matrix4d::Identity();
Eigen::Vector3d linarVelocityBodyNED(0,0,0);

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
    int indexName = 0;
    for (auto &name: msg->name) {
        if (name == "uuv_bluerov2_heavy") {
            break;
        }
        indexName++;
    }


    Eigen::Quaterniond rotTMP(msg->pose[indexName].orientation.w, msg->pose[indexName].orientation.x,
                              msg->pose[indexName].orientation.y,
                              msg->pose[indexName].orientation.z);

    Eigen::Matrix4d transformationOfPose = Eigen::Matrix4d::Zero();
    transformationOfPose.block<3, 3>(0, 0) = rotTMP.toRotationMatrix();
    transformationOfPose(0, 3) = msg->pose[indexName].position.x;
    transformationOfPose(1, 3) = msg->pose[indexName].position.y;
    transformationOfPose(2, 3) = msg->pose[indexName].position.z;
    transformationOfPose(3, 3) = 1;


    Eigen::Vector3d linarVelocityWorld(msg->twist[indexName].linear.x, msg->twist[indexName].linear.y,
                                       msg->twist[indexName].linear.z);
    Eigen::Vector3d linarVelocityBody = rotTMP.inverse() * linarVelocityWorld;
    Eigen::AngleAxisd rotX180(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));


    linarVelocityBodyNED = rotX180.toRotationMatrix()*linarVelocityBody;
}

void init(){
    ros::spin();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "simulatedDVL");
    ros::start();
    ros::NodeHandle n_;

    Eigen::AngleAxisd rotX180(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
    //Eigen::AngleAxisd rotz90(90.0 / 180.0 * 3.14159, Eigen::Vector3d(0, 0, 1));
    transformationFromGazeboToNED.block<3, 3>(0,
                                              0) = rotX180.toRotationMatrix();//rotz90.toRotationMatrix()*rotX180.toRotationMatrix();
    transformationFromGazeboToNED(3, 3) = 1;


    publisherGazeboData = n_.advertise<geometry_msgs::Vector3Stamped>("simulatedDVL", 1000);
    ros::Subscriber subscriberGazebo = n_.subscribe("gazebo/model_states", 1000, &callbackGazebo);



    std::thread t1(init);


    ros::Rate ourRate = ros::Rate(5);
    while(ros::ok()){
        geometry_msgs::Vector3Stamped publishingMsg;
        publishingMsg.header.stamp = ros::Time::now();
        publishingMsg.vector.x = linarVelocityBodyNED.x();
        publishingMsg.vector.y = linarVelocityBodyNED.y();
        publishingMsg.vector.z = linarVelocityBodyNED.z();
        publisherGazeboData.publish(publishingMsg);
        ourRate.sleep();
    }

    return (0);
}


