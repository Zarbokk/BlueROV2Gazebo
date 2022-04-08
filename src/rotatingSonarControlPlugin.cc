//
// Created by tim on 05.03.21.
//

#ifndef SONAR_SENSOR_GAZEBO_ROTATINGSONARCONTROLPLUGIN_CC_H
#define SONAR_SENSOR_GAZEBO_ROTATINGSONARCONTROLPLUGIN_CC_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
//#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <cmath>

namespace gazebo {
    /// \brief A plugin to control a Velodyne sensor.
    class RotatingSonarControlPlugin : public ModelPlugin {
        /// \brief Constructor
    public:
        RotatingSonarControlPlugin() {}


        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
            double velocity = 0;//default velocity this is rad/s
            if (_sdf->HasElement("velocity"))
                velocity = _sdf->Get<double>("velocity");
            // Safety check
            if (_model->GetJointCount() == 0) {
                std::cerr << "Invalid joint count, Sonar plugin not loaded\n";
                return;
            }
            // Store the model pointer for convenience.
            this->model = _model;
            for (int i = 0; i < _model->GetJoints().size(); i++) {
                if(_model->GetJoints()[i]->GetName().find("rotating_sonar_base_to_top_joint") != std::string::npos){
                    this->joint=_model->GetJoints()[i];
                }
            }
            if(this->joint==NULL){
                std::cerr << "ERROR JOINT POINTER NULL\n";
            }


//            this->joint->SetParam("fmax",0, 100.0);
//            this->joint->SetParam("vel",0, velocity);

            boost::thread callback_laser_queue_thread_ = boost::thread(
                    boost::bind(&RotatingSonarControlPlugin::positionLaserPublishingThread, this));
        }

    private:


        /// \brief Pointer to the model.
        physics::ModelPtr model;

        /// \brief Pointer to the joint.
        physics::JointPtr joint;

        /// \brief A PID controller for the joint.
        common::PID pid;
        /// \brief A node used for transport

        void positionLaserPublishingThread() {
            std::cerr << "Starting Ros for current angle\n";
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "rotationAngleSonar");
            ros::start();
            ros::NodeHandle n_;
            ros::Publisher publisherAngleofSonar;
            publisherAngleofSonar = n_.advertise<std_msgs::Float64>("sonar/currentRelativeAngleSonar", 10);
            ros::Rate loop_rate(100);
            int howFast = 4;//(small faster) old was 4
            int numberOfSteps = 400;
            int currentPosition = 0;
            while (ros::ok()) {
                this->joint->SetPosition(0,2*M_PI*(1.0-(double)(currentPosition)/((double)(numberOfSteps*howFast))),true);


                double currentAngle = this->joint->Position();
                currentAngle = std::fmod(currentAngle, 2 * M_PI);
                std_msgs::Float64 currentAngleMsg;
                currentAngleMsg.data = 2 * M_PI-currentAngle;
                publisherAngleofSonar.publish(currentAngleMsg);
                ros::spinOnce();
                //std::cerr << "CurrentAngle: "<< currentAngle <<"\n";
                loop_rate.sleep();
                currentPosition++;
                if(currentPosition == numberOfSteps*howFast){
                    currentPosition = 0;
                }
            }
//            while (true) {
//                //std::cerr << "ConnectCb: "<< this->joint->Position() <<"\n";
//
//            }
        }
    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(RotatingSonarControlPlugin)
}
#endif