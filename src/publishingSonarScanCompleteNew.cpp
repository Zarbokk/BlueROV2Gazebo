//
// Created by tim on 08.03.21.
//

#include <stdio.h>
#include <ros/ros.h>
#include "ping360_sonar/SonarEcho.h"
#include <random>
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
        this->demoPublisher_ = n_.advertise<sensor_msgs::PointCloud2>(publishName, 10);
        this->intensityPublisher = n_.advertise<ping360_sonar::SonarEcho>("sonar/intensity", 10);
        this->lastAngle = 0;
        this->nextDesiredAngle = 0;
        this->numberOfAngles = 400;
        this->fullScanCloud = pcl::PointCloud<pcl::PointXYZ>();
        this->savePoints = false;
        this->mt = std::mt19937(rd());
        this->randomNumberOfErrors = std::uniform_real_distribution<double>(0, 5);
        this->randomNumberWhereError = std::uniform_real_distribution<double>(1.0, 40.0);
        //Topic you want to subscribe
        this->subPointCloud_ = n_.subscribe(subscribePointCloudName, 1000, &SonarScanComplete::callbackPointcloud,
                                            this);
        this->subAngle_ = n_.subscribe(subscribeAngleName, 1000, &SonarScanComplete::callbackAngle, this);
    }

    double gaussianOutput(double mean, double std, double position) {
        return 1 / std * exp(-0.5 * ((position - mean) * (position - mean)) / (std * std));
    }

    void callbackPointcloud(const sensor_msgs::PointCloud2ConstPtr &msg) {
        pcl::fromROSMsg(*msg, this->currentScannedPoints);
    }

    void pointsToRemove(pcl::PointCloud<pcl::PointXYZ> &pointCloud) {
        int j = 0;
        float x, y;
        std::vector<float> xVector, yVector;
        for (int i = 0; i < pointCloud.size(); i++) {//calculating the mean
            double distance = sqrt(
                    pow(pointCloud.points[i].x, 2) + pow(pointCloud.points[i].y, 2) + pow(pointCloud.points[i].z, 2));
            if (distance < 40 || distance > 0.5) {//reject close and far away points
                j++;
                x += pointCloud.points[i].x;
                y += pointCloud.points[i].y;
                xVector.push_back(pointCloud.points[i].x);
                yVector.push_back(pointCloud.points[i].y);
            }
        }
        x = x / ((float) j);
        y = y / ((float) j);
        std::sort(xVector.begin(), xVector.end());
        std::sort(yVector.begin(), yVector.end());
        int position = (int) (xVector.size() / 2.0);
        if (xVector.size() > 0) {
            x = xVector[position];
            y = yVector[position];
            pointCloud = pcl::PointCloud<pcl::PointXYZ>();

            pcl::PointXYZ newPoint;
            newPoint.x = x;
            newPoint.y = y;
            pointCloud.push_back(newPoint);
        }
    }


    void callbackAngle(const std_msgs::Float64::ConstPtr &msg) {
        this->currentAngle = msg->data;
//        std::cout << "the current desired Angle: " << this->nextDesiredAngle << "the current Angle: "
//                  << this->currentAngle << std::endl;
        if ((this->currentAngle - this->nextDesiredAngle) >= 0 &&
            abs((this->currentAngle - this->nextDesiredAngle)) < 2 * M_PI / this->numberOfAngles) {

            pcl::PointCloud<pcl::PointXYZ>::Ptr newScanCloud(new pcl::PointCloud<pcl::PointXYZ>);
            Eigen::AngleAxisf rotationVector90Degree(M_PI_2, Eigen::Vector3f(1, 0, 0));
            Eigen::Quaternionf quatRot90Degree(rotationVector90Degree.toRotationMatrix());
            Eigen::Matrix<float, 3, 1> shift(0, 0, 0);
            *newScanCloud = this->currentScannedPoints;
            pcl::transformPointCloud(*newScanCloud, *newScanCloud, shift, quatRot90Degree);

            publishingOfIntensitySonar(*newScanCloud, this->nextDesiredAngle);


            Eigen::AngleAxisf rotationVectorCurrentAngle(this->currentAngle, Eigen::Vector3f(0, 0, 1));
            Eigen::Quaternionf quatRotCurrentAngle(rotationVectorCurrentAngle.toRotationMatrix());

            //std::cout << myCloud->size() << std::endl;

            pcl::transformPointCloud(*newScanCloud, *newScanCloud, shift, quatRotCurrentAngle);
            this->pointsToRemove(*newScanCloud);
            fullScanCloud += *newScanCloud;

            if (abs(2 * M_PI - this->currentAngle) <=
                2 * M_PI / this->numberOfAngles + 0.001) {
//                std::cout << " Publish Scan " << std::endl;
                //publish complete scan
                sensor_msgs::PointCloud2 cloud_msg;
                pcl::toROSMsg(fullScanCloud, cloud_msg);
                cloud_msg.header.frame_id = "rotating_sonar_bot";
                cloud_msg.header.stamp = ros::Time::now();
                this->demoPublisher_.publish(cloud_msg);
                //std::cout << cloud_msg.row_step << std::endl;
                this->fullScanCloud = pcl::PointCloud<pcl::PointXYZ>();
            }
            this->nextDesiredAngle = this->nextDesiredAngle + 2 * M_PI / this->numberOfAngles;
            if (abs(this->nextDesiredAngle - 2 * M_PI) < 0.0001) {
                this->nextDesiredAngle = 0;
            }
        }
    }

    void publishingOfIntensitySonar(pcl::PointCloud<pcl::PointXYZ> inputPointCloud, double angle) {
        // array of 100
        //start intensity output
        //first copy cloud
        //calculate intensities by gaussian over every point. Add a few random gaussians somewhere
        // add gaussians at the beginning with random.

        double std = 0.4;


        double outputArray[200];
        int sizeOfArrays = 200;
        ping360_sonar::SonarEcho msg;
        msg.header.stamp = ros::Time::now();
        msg.range = 40;
        msg.number_of_samples = sizeOfArrays;
        msg.angle = angle/M_PI*200;


        int howOftenAddRandomPoints = (int)this->randomNumberOfErrors(this->mt);
        std::vector<int> addedRandomAtPos;
        for (int i = 0 ; i<howOftenAddRandomPoints ; i++){
            addedRandomAtPos.push_back((int)this->randomNumberWhereError(this->mt));
        }
        for (int i = 0; i < sizeOfArrays; i++) {
            outputArray[i] = 0;
            double currentPosition = ((double) i) * (((double) msg.range) / ((double) msg.number_of_samples));
            for (int j = 0; j < inputPointCloud.size(); j++) {

                double rangePoint = sqrt(pow(inputPointCloud.points[j].x, 2) + pow(inputPointCloud.points[j].y, 2) +
                                         pow(inputPointCloud.points[j].z, 2));
//                std::cout << "rangePoints"<< rangePoint << std::endl;
                outputArray[i] += 0.5 * this->gaussianOutput(rangePoint, std,
                                                             currentPosition);//scaling with 0.5 just for good visibility
            }
            outputArray[i] += this->gaussianOutput(0, std * 0.7, currentPosition);
            //add random gaussians
            for (auto& a : addedRandomAtPos) {
                outputArray[i] += this->gaussianOutput(a, std * 0.5, currentPosition);
            }

            outputArray[i]+=i*0.01;
            msg.intensities.push_back((unsigned char) ((int) outputArray[i]));
        }


        this->intensityPublisher.publish(msg);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher demoPublisher_, intensityPublisher;
    ros::Subscriber subPointCloud_, subAngle_;
    double currentAngle, lastAngle, nextDesiredAngle;
    bool savePoints;
    int numberOfAngles;
    pcl::PointCloud<pcl::PointXYZ> fullScanCloud;
    pcl::PointCloud<pcl::PointXYZ> currentScannedPoints;
    std::random_device rd;
    std::mt19937 mt;
    std::uniform_real_distribution<double> randomNumberOfErrors,randomNumberWhereError;

};//End of class SubscribeAndPublish





int main(int argc, char **argv) {
    // setup ros for this node and get handle to ros system
    ros::init(argc, argv, "pcl_sonar_publisher");
    ros::start();

    SonarScanComplete tmp = SonarScanComplete("sonar/full_scan", "sonar/currentRelativeAngleSonar", "sonar/points");


    ros::spin();
    return 0;
}