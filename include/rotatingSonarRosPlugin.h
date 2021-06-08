//
// Created by tim on 07.03.21.
//

#ifndef SONAR_SENSOR_GAZEBO_ROTATINGSONARROSPLUGIN_H
#define SONAR_SENSOR_GAZEBO_ROTATINGSONARROSPLUGIN_H

//#include <gazebo/plugins/GpuRayPlugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#if GAZEBO_GPU_RAY
#include <gazebo/plugins/GpuRayPlugin.hh>
#else
#include <gazebo/plugins/RayPlugin.hh>
#endif

#ifndef GAZEBO_GPU_RAY
#define GAZEBO_GPU_RAY 0
#endif
#if GAZEBO_GPU_RAY
#define RotatingSonarRosPlugin RotatingSonarRosGpuPlugin
#define RayPlugin GpuRayPlugin
#define RaySensorPtr GpuRaySensorPtr
#endif



#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/algorithm/string/trim.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace gazebo {
    /// \brief A plugin to control a Velodyne sensor.
    class RotatingSonarRosPlugin : public RayPlugin {
        /// \brief Constructor
    public:
        RotatingSonarRosPlugin() {};
        virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    private:
        gazebo::transport::NodePtr gazebo_node_;
        gazebo::transport::SubscriberPtr sub_;
        sensors::RaySensorPtr parent_ray_sensor_;
        std::string robot_namespace_;
        void OnScan(const ConstLaserScanStampedPtr &_msg);
        void ConnectCb();

        //SDF things
        double min_range_;
        double max_range_;
        std::string topic_name_;
        std::string frame_name_;

        //ROS things
        ros::Publisher pub_;
        ros::NodeHandle* nh_;
        ros::CallbackQueue laser_queue_;
        boost::thread callback_laser_queue_thread_;
        void laserQueueThread();
        boost::mutex lock_;
        bool organize_cloud_;
    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_SENSOR_PLUGIN(RotatingSonarRosPlugin)
}
#endif //SONAR_SENSOR_GAZEBO_ROTATINGSONARROSPLUGIN_H