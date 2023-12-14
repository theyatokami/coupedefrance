#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h> // Include image_transport headers

image_transport::Publisher image_pub; // Publisher for the processed image

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        // IMAGE PROCESSING HERE
        //
        // Create an ArUco dictionary
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

        // Detect markers
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);

        // Draw detected markers
        if (!markerIds.empty()) {
            cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
        }

        // Convert the processed image to an ROS image message
        sensor_msgs::ImagePtr processed_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

        // Publish the processed image using image_transport
        image_pub.publish(processed_msg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_detection_node");
    ros::NodeHandle nh;

    // Create an image_transport object
    image_transport::ImageTransport it(nh);

    // Subscribe to the camera topic using image_transport
    image_transport::Subscriber sub = it.subscribe("camera_topic", 1, imageCallback);

    // Create a publisher for the processed image using image_transport
    image_pub = it.advertise("aruco_topic", 1);

    ros::spin();

    return 0;
}

