//
// Created by tim on 08.03.21.
//

#include <stdio.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include "sensor_msgs/PointCloud2.h"
#include <vector>
#include "std_msgs/Float64.h"

class SonarScanComplete {
public:
    SonarScanComplete(const std::string &publishName, const std::string &subscribeAngleName,
                      const std::string &subscribePointCloudName) {
        //Topic you want to publish "cloud_topic";
        demoPublisher_ = n_.advertise<sensor_msgs::PointCloud2>(publishName, 10);
        lastAngle=0;
        fullScanCloud =  pcl::PointCloud<pcl::PointXYZ>();

        //Topic you want to subscribe
        subPointCloud_ = n_.subscribe(subscribePointCloudName, 1000, &SonarScanComplete::callbackPointcloud, this);
        subAngle_ = n_.subscribe(subscribeAngleName, 1000, &SonarScanComplete::callbackAngle, this);
    }

    void callbackPointcloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr newScanCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg,*newScanCloud);

        Eigen::AngleAxisf rotationVector90Degree(M_PI_2, Eigen::Vector3f(1, 0, 0));
        Eigen::Quaternionf quatRot90Degree(rotationVector90Degree.toRotationMatrix());
        Eigen::Matrix<float, 3, 1> shift(0, 0, 0);

        pcl::transformPointCloud(*newScanCloud, *newScanCloud, shift, quatRot90Degree);

        Eigen::AngleAxisf rotationVectorCurrentAngle(currentAngle, Eigen::Vector3f(0, 0, 1));
        Eigen::Quaternionf quatRotCurrentAngle(rotationVectorCurrentAngle.toRotationMatrix());

        //std::cout << myCloud->size() << std::endl;

        pcl::transformPointCloud(*newScanCloud, *newScanCloud, shift, quatRotCurrentAngle);
        pointsToRemove(*newScanCloud);
        fullScanCloud +=*newScanCloud;

        if(currentAngle-lastAngle<0){//@TODO evtl noch falsch sollte was cooles her
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(fullScanCloud, cloud_msg);
            cloud_msg.header.frame_id = "rotating_sonar_bot";
            cloud_msg.header.stamp = ros::Time::now();
            demoPublisher_.publish(cloud_msg);
            fullScanCloud = pcl::PointCloud<pcl::PointXYZ>();
//            if (abs(currentAngle-lastAngle+currentAngle-2*M_PI)<0.01 && currentAngle-lastAngle+currentAngle-2*M_PI<0){
//                currentAngle = 0;
//            }
        }
        lastAngle = currentAngle;

    }

    void pointsToRemove(pcl::PointCloud<pcl::PointXYZ> &pointCloud){
        int j = 0;
        float x,y;
        std::vector<float> xVector,yVector;
        for(int i = 0 ; i<pointCloud.size();i++){//calculating the mean
            double distance = sqrt(pow(pointCloud.points[i].x,2)+pow(pointCloud.points[i].y,2)+pow(pointCloud.points[i].z,2));
            if (distance<40||distance>0.5){//reject close and far away points
                j++;
                x+=pointCloud.points[i].x;
                y+=pointCloud.points[i].y;
                xVector.push_back(pointCloud.points[i].x);
                yVector.push_back(pointCloud.points[i].y);
            }
        }
        x = x/((float)j);
        y = y/((float)j);
        std::sort(xVector.begin(), xVector.end());
        std::sort(yVector.begin(), yVector.end());
        int position = (int)(xVector.size()/2.0);
        if(xVector.size()>0){
            x = xVector[position];
            y = yVector[position];
            pointCloud = pcl::PointCloud<pcl::PointXYZ>();

            pcl::PointXYZ newPoint;
            newPoint.x=x;
            newPoint.y=y;
            pointCloud.push_back(newPoint);
        }
    }

    void callbackAngle(const std_msgs::Float64::ConstPtr &msg) {
        currentAngle = msg->data;
    }

private:
    ros::NodeHandle n_;
    ros::Publisher demoPublisher_;
    ros::Subscriber subPointCloud_, subAngle_;
    double currentAngle, lastAngle;
    pcl::PointCloud<pcl::PointXYZ> fullScanCloud;

};//End of class SubscribeAndPublish





int main(int argc, char **argv) {
    // setup ros for this node and get handle to ros system
    ros::init(argc, argv, "pcl_sonar_publisher");
    ros::start();

    SonarScanComplete tmp = SonarScanComplete("sonar/full_scan", "sonar/currentRelativeAngleSonar", "sonar/points");


    ros::spin();
    return 0;
}