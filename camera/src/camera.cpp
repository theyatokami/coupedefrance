#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera_topic", 1);

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        ROS_ERROR("Error: Unable to open camera");
        return -1;
    }

    // Set the desired frame width and height for 720p resolution


    ros::Rate loop_rate(120); // Adjust the rate to match the desired frame rate

    while (ros::ok()) {
        cv::Mat frame, flipped_frame;
        cap >> frame;

        if (!frame.empty()) {
            cv::flip(frame, flipped_frame, 1);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", flipped_frame).toImageMsg();
            pub.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

