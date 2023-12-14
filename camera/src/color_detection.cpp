#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include "camera/CalibrateColor.h"
#include <std_srvs/Trigger.h>

class ColorDetectionNode {
public:
    ColorDetectionNode()
        : nh_(), it_(nh_), calibration_in_progress_(false) {
        // Initialize the parameters
        nh_.param("target_hue_min", target_hsv_min_[0], 0.0);
        nh_.param("target_hue_max", target_hsv_max_[0], 10.0);

        sub_ = it_.subscribe("camera_topic", 1, &ColorDetectionNode::imageCallback, this);
        position_pub_ = nh_.advertise<std_msgs::String>("object_position", 1000);
        image_pub_ = it_.advertise("processed_image", 1);
        start_calibration_service_ = nh_.advertiseService("start_calibration", &ColorDetectionNode::startCalibrationCallback, this);
        finish_calibration_service_ = nh_.advertiseService("finish_calibration", &ColorDetectionNode::finishCalibrationCallback, this);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            if (calibration_in_progress_) {
                calibrateColor(frame);
            } else {
                processFrame(frame);
            }

            sensor_msgs::ImagePtr processed_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            image_pub_.publish(processed_msg);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

private:
    void calibrateColor(cv::Mat& frame) {
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
	// Calculate the size of the ROI
	int roiWidth = frame.cols / 25;  // 1/25th of the frame's width
	int roiHeight = frame.rows / 20; // 1/20th of the frame's height

	// Calculate the starting point (top-left corner) of the ROI
	int roiX = (frame.cols - roiWidth) / 2;
	int roiY = (frame.rows - roiHeight) / 2;

	// Create the ROI centered in the frame
	cv::Rect roi(roiX, roiY, roiWidth, roiHeight);
	cv::rectangle(frame, roi, cv::Scalar(0, 0, 255), 2); // Optional: Draw the ROI for visualization
	cv::Mat hsvRoi = hsv(roi);

        

        target_hsv_min_ = computeMedianHSV(hsvRoi, -35);
        target_hsv_max_ = computeMedianHSV(hsvRoi, 35);

        ROS_INFO("Color calibration completed. New HSV Range: [%f, %f, %f] - [%f, %f, %f]",
                 target_hsv_min_[0], target_hsv_min_[1], target_hsv_min_[2],
                 target_hsv_max_[0], target_hsv_max_[1], target_hsv_max_[2]);
    }

    cv::Scalar computeMedianHSV(cv::Mat hsvRoi, int rangeOffset) {
        std::vector<uint8_t> hVals, sVals, vVals;
        for (int y = 0; y < hsvRoi.rows; ++y) {
            for (int x = 0; x < hsvRoi.cols; ++x) {
                cv::Vec3b pixel = hsvRoi.at<cv::Vec3b>(y, x);
                hVals.push_back(pixel[0]);
                sVals.push_back(pixel[1]);
                vVals.push_back(pixel[2]);
            }
        }

        std::nth_element(hVals.begin(), hVals.begin() + hVals.size()/2, hVals.end());
        std::nth_element(sVals.begin(), sVals.begin() + sVals.size()/2, sVals.end());
        std::nth_element(vVals.begin(), vVals.begin() + vVals.size()/2, vVals.end());

        return cv::Scalar(
            static_cast<double>(hVals[hVals.size()/2]) + rangeOffset,
            static_cast<double>(sVals[sVals.size()/2]) + rangeOffset,
            static_cast<double>(vVals[vVals.size()/2]) + rangeOffset
        );
    }

    void processFrame(cv::Mat& frame) {
    cv::Mat hsvImage;
    cv::cvtColor(frame, hsvImage, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsvImage, target_hsv_min_, target_hsv_max_, mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double minArea = frame.rows * frame.cols / 1000.0; // Minimum area to consider

    for (size_t i = 0; i < contours.size(); i++) {
        double contourArea = cv::contourArea(contours[i]);
        if (contourArea > minArea) {
            cv::Moments M = cv::moments(contours[i]);
            int cX = static_cast<int>(M.m10 / M.m00);
            int cY = static_cast<int>(M.m01 / M.m00);

            cv::drawContours(frame, contours, static_cast<int>(i), cv::Scalar(0, 255, 0), 2);
            cv::circle(frame, cv::Point(cX, cY), 7, cv::Scalar(255, 255, 255), -1);
            std::string object_position = "Object at (" + std::to_string(cX) + ", " + std::to_string(cY) + ")";
            cv::putText(frame, object_position, cv::Point(cX - 20, cY - 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
        }
    }
}


    bool startCalibrationCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        if (!calibration_in_progress_) {
            calibration_in_progress_ = true;
            ROS_INFO("Color calibration started.");
            res.success = true;
        } else {
            ROS_WARN("Color calibration is already in progress.");
            res.success = false;
        }
        return true;
    }

    bool finishCalibrationCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        if (calibration_in_progress_) {
            calibration_in_progress_ = false;
            ROS_INFO("Color calibration finished.");
            res.success = true;
        } else {
            ROS_WARN("No color calibration in progress.");
            res.success = false;
        }
        return true;
    }

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher position_pub_;
    ros::ServiceServer start_calibration_service_;
    ros::ServiceServer finish_calibration_service_;

    cv::Scalar target_hsv_min_;
    cv::Scalar target_hsv_max_;
    bool calibration_in_progress_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "color_detection_node");
    ColorDetectionNode colorDetectionNode;
    ros::spin();
    return 0;
}

