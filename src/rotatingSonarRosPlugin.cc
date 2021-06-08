

#include <rotatingSonarRosPlugin.h>
#if GAZEBO_GPU_RAY
#define RaySensor GpuRaySensor
#endif

namespace gazebo {

    void RotatingSonarRosPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {

        RayPlugin::Load(_parent, _sdf);
        gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
        gazebo_node_->Init();
        // Get the parent ray sensor
#if GAZEBO_MAJOR_VERSION >= 7
        parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#else
        parent_ray_sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#endif
        if (!parent_ray_sensor_) {
            gzthrow("GazeboRosSonar" <<  "Laser controller requires a " <<  "Ray Sensor as its parent");
        }

        //SDF TESTS:
        if (!_sdf->HasElement("frameName")) {
            ROS_INFO("Rotating Sonar Ros plugin missing <frameName>, defaults to /world");
            frame_name_ = "world";
        } else {
            frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
        }
        robot_namespace_ = "/";
        if (_sdf->HasElement("robotNamespace")) {
            robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        }
        if (!_sdf->HasElement("min_range")) {
            ROS_INFO("Rotating Sonar Ros plugin missing <min_range>, defaults to 0");
            min_range_ = 0;
        } else {
            min_range_ = _sdf->GetElement("min_range")->Get<double>();
        }

        if (!_sdf->HasElement("max_range")) {
            ROS_INFO("Rotating Sonar Ros plugin missing <max_range>, defaults to infinity");
            max_range_ = INFINITY;
        } else {
            max_range_ = _sdf->GetElement("max_range")->Get<double>();
        }

        if (!_sdf->HasElement("topicName")) {
            ROS_INFO("Rotating Sonar Ros plugin missing <topicName>, defaults to /points");
            topic_name_ = "/points";
        } else {
            topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
        }


        //STARTING WITH ROS
        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client",
                      ros::init_options::NoSigintHandler);
        }

        // Create node handle
        nh_ = new ros::NodeHandle(robot_namespace_);

        if (topic_name_ != "") {
            std::cerr << "ros getting advertised \n";
            ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
                    topic_name_, 1,
                    boost::bind(&RotatingSonarRosPlugin::ConnectCb, this),
                    boost::bind(&RotatingSonarRosPlugin::ConnectCb, this),
                    ros::VoidPtr(), &laser_queue_);
            pub_ = nh_->advertise(ao);
        }

        parent_ray_sensor_->SetActive(false);
        // Start custom queue for laser
        callback_laser_queue_thread_ = boost::thread(boost::bind(&RotatingSonarRosPlugin::laserQueueThread, this));
#if GAZEBO_MAJOR_VERSION >= 7
        ROS_INFO("Velodyne laser plugin ready, %i lasers", parent_ray_sensor_->VerticalRangeCount());
        ROS_INFO("Velodyne laser plugin ready, %i lasers", parent_ray_sensor_->RayCount());
#else
        ROS_INFO("Velodyne %slaser plugin ready, %i lasers", STR_GPU_, parent_ray_sensor_->GetHorizontalRangeCount());
#endif

    }

    void RotatingSonarRosPlugin::ConnectCb() {
        std::cerr << "ConnectCb\n";
        boost::lock_guard <boost::mutex> lock(lock_);
        if (pub_.getNumSubscribers()) {
            if (!sub_) {
#if GAZEBO_MAJOR_VERSION >= 7
                sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(), &RotatingSonarRosPlugin::OnScan, this);
#else
                sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->GetTopic(), &GazeboRosVelodyneLaser::OnScan,
                                               this);
#endif
            }
            parent_ray_sensor_->SetActive(true);
        } else {
#if GAZEBO_MAJOR_VERSION >= 7
            if (sub_) {
                sub_->Unsubscribe();
                sub_.reset();
            }
#endif
            parent_ray_sensor_->SetActive(false);
        }
    }

    void RotatingSonarRosPlugin::OnScan(ConstLaserScanStampedPtr &_msg) {
#if GAZEBO_MAJOR_VERSION >= 7
        const ignition::math::Angle maxAngle = parent_ray_sensor_->AngleMax();
        const ignition::math::Angle minAngle = parent_ray_sensor_->AngleMin();

        const double maxRange = parent_ray_sensor_->RangeMax();
        const double minRange = parent_ray_sensor_->RangeMin();

        const int rayCount = parent_ray_sensor_->RayCount();
        const int rangeCount = parent_ray_sensor_->RangeCount();

        const int verticalRayCount = parent_ray_sensor_->VerticalRayCount();
        const int verticalRangeCount = parent_ray_sensor_->VerticalRangeCount();

        const ignition::math::Angle verticalMaxAngle = parent_ray_sensor_->VerticalAngleMax();
        const ignition::math::Angle verticalMinAngle = parent_ray_sensor_->VerticalAngleMin();
//        std::cerr << "rayCount: "<< rayCount<< "\n";
//        std::cerr << "rangeCount: "<< rangeCount<< "\n";
//        std::cerr << "verticalRayCount: "<< verticalRayCount<< "\n";
//        std::cerr << "verticalRangeCount: "<< verticalRangeCount<< "\n";

#else
        math::Angle maxAngle = parent_ray_sensor_->GetAngleMax();
        math::Angle minAngle = parent_ray_sensor_->GetAngleMin();

        const double maxRange = parent_ray_sensor_->GetRangeMax();
        const double minRange = parent_ray_sensor_->GetRangeMin();

        const int rayCount = parent_ray_sensor_->GetRayCount();
        const int rangeCount = parent_ray_sensor_->GetRangeCount();

        const int verticalRayCount = parent_ray_sensor_->GetVerticalRayCount();
        const int verticalRangeCount = parent_ray_sensor_->GetVerticalRangeCount();

        const math::Angle verticalMaxAngle = parent_ray_sensor_->GetVerticalAngleMax();
        const math::Angle verticalMinAngle = parent_ray_sensor_->GetVerticalAngleMin();
#endif

        const double yDiff = maxAngle.Radian() - minAngle.Radian();
        const double pDiff = verticalMaxAngle.Radian() - verticalMinAngle.Radian();

        const double MIN_RANGE = std::max(min_range_, minRange);
        const double MAX_RANGE = std::min(max_range_, maxRange);
        const double MIN_INTENSITY = 0;


        int i, j;

        pcl::PointCloud<pcl::PointXYZI> myCloud;
        for (i = 0; i < rangeCount; i++) {
            for (j = 0; j < verticalRangeCount; j++) {
                // Range
                double r = _msg->scan().ranges(i + j * rangeCount);
                // Intensity
                double intensity = _msg->scan().intensities(i + j * rangeCount);
                // Ignore points that lay outside range bands or optionally, beneath a
                // minimum intensity level.
                if ((MIN_RANGE >= r) || (r >= MAX_RANGE) || (intensity < MIN_INTENSITY)) {
                    if (!organize_cloud_) {
                        continue;
                    }
                }

                // Get angles of ray to get xyz for point
                double yAngle;
                double pAngle;

                if (rangeCount > 1) {
                    yAngle = i * yDiff / (rangeCount - 1) + minAngle.Radian();
                } else {
                    yAngle = minAngle.Radian();
                }

                if (verticalRayCount > 1) {
                    pAngle = j * pDiff / (verticalRangeCount - 1) + verticalMinAngle.Radian();
                } else {
                    pAngle = verticalMinAngle.Radian();
                }

                // pAngle is rotated by yAngle:
                pcl::PointXYZI newPoint;

                newPoint.x = r * cos(pAngle) * cos(yAngle);
                newPoint.y = r * cos(pAngle) * sin(yAngle);
                newPoint.z = r * sin(pAngle);
                newPoint.intensity = intensity;
                myCloud.points.push_back(newPoint);

            }
        }

        // Populate message with number of valid points
        sensor_msgs::PointCloud2 cloud_msg;

        pcl::toROSMsg(myCloud, cloud_msg);
        cloud_msg.header.frame_id = "rotating_sonar_top";
        cloud_msg.header.stamp = ros::Time::now();

        // Publish output
        pub_.publish(cloud_msg);
    }

    void RotatingSonarRosPlugin::laserQueueThread() {
        while (nh_->ok()) {
            laser_queue_.callAvailable(ros::WallDuration(0.01));
        }
    }

}
