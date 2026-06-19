#include <ros/ros.h>

#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>

#include <opencv2/opencv.hpp>

#include <vector>
#include <string>
#include <algorithm>

class VisionNode
{
private:
    ros::NodeHandle nh_;

    ros::Subscriber image_sub_;
    ros::Subscriber capture_sub_;

    ros::Publisher contour_pub_;
    ros::Publisher contour_info_pub_;

    std::string image_topic_;

    double epsilon_factor_;
    double min_area_;

    int blur_size_;
    int kernel_size_;
    int morphology_iterations_;
    int display_width_;
    int display_height_;

    std::string morphology_mode_;
    std::string contour_mode_;

    bool capture_requested_;
    bool contour_locked_;
    bool show_windows_;

    std::string window_name_;

public:
    VisionNode()
    {
        ros::NodeHandle private_nh("~");

        private_nh.param<std::string>("image_topic", image_topic_, "/camera/image/compressed");

        private_nh.param<double>("epsilon_factor", epsilon_factor_, 0.002);
        private_nh.param<double>("min_area", min_area_, 500.0);

        private_nh.param<int>("blur_size", blur_size_, 5);
        private_nh.param<int>("kernel_size", kernel_size_, 3);
        private_nh.param<int>("morphology_iterations", morphology_iterations_, 1);
        private_nh.param<int>("display_width", display_width_, 400);
        private_nh.param<int>("display_height", display_height_, 400);

        private_nh.param<std::string>("morphology_mode", morphology_mode_, "close_open");
        private_nh.param<std::string>("contour_mode", contour_mode_, "none");

        private_nh.param<bool>("show_windows", show_windows_, true);

        capture_requested_ = false;
        contour_locked_ = false;

        window_name_ = "Resultat visio robot";

        if (show_windows_) {
            cv::startWindowThread();
            cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
            cv::resizeWindow(window_name_, display_width_ * 3, display_height_);
            ROS_INFO("Finestra OpenCV activada: %s", window_name_.c_str());
        }

        image_sub_ = nh_.subscribe(image_topic_, 1, &VisionNode::imageCallback, this);
        capture_sub_ = nh_.subscribe("/capture_contour", 1, &VisionNode::captureCallback, this);

        contour_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/contour_points", 1, true);
        contour_info_pub_ = nh_.advertise<geometry_msgs::Vector3>("/contour_info", 1, true);

        ROS_INFO("vision_node iniciat.");
        ROS_INFO("Subscrivint imatge: %s", image_topic_.c_str());
        ROS_INFO("Publicant contorn a: /contour_points");
        ROS_INFO("Publicant informacio de contorn a: /contour_info");
    }

    void captureCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (!msg->data) {
            return;
        }

        if (contour_locked_) {
            ROS_WARN("Ja hi ha un contorn capturat. No es tornara a publicar.");
            return;
        }

        capture_requested_ = true;
        ROS_INFO("Ordre de captura rebuda. Es publicara el proper contorn valid.");
    }

    void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
    {
        cv::Mat image = decodeCompressedImage(msg);

        if (image.empty()) {
            ROS_WARN_THROTTLE(2.0, "No s'ha pogut descodificar la imatge comprimida.");
            return;
        }

        if (contour_locked_) {
            cv::waitKey(1);
            return;
        }

        cv::Mat mask;
        std::vector<cv::Point> original_contour;
        std::vector<cv::Point> approx_contour;

        bool valid = processImage(image, mask, original_contour, approx_contour);

        showResult(image, mask, original_contour, approx_contour);

        if (!valid) {
            cv::waitKey(1);
            return;
        }

        cv::Rect bbox = cv::boundingRect(approx_contour);

        ROS_DEBUG("Bounding box: %d x %d px", bbox.width, bbox.height);
        ROS_DEBUG("Contorn original: %lu punts | simplificat: %lu punts",
                  original_contour.size(),
                  approx_contour.size());

        if (capture_requested_ && !contour_locked_) {
            publishContour(approx_contour, bbox);
            publishContourInfo(bbox);

            capture_requested_ = false;
            contour_locked_ = true;

            ROS_INFO("Contorn seleccionat i publicat amb %lu punts.", approx_contour.size());
            ROS_INFO("La captura queda bloquejada. Es fara servir aquest contorn per generar el dibuix.");
        }

        cv::waitKey(1);
    }

    cv::Mat decodeCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg)
    {
        try {
            cv::Mat data(msg->data);
            return cv::imdecode(data, cv::IMREAD_COLOR);
        }
        catch (const cv::Exception& e) {
            ROS_WARN("Error OpenCV descodificant imatge: %s", e.what());
            return cv::Mat();
        }
    }

    bool processImage(const cv::Mat& image,
                      cv::Mat& mask,
                      std::vector<cv::Point>& best_contour,
                      std::vector<cv::Point>& approx_contour)
    {
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        int blur = blur_size_;

        if (blur % 2 == 0) {
            blur += 1;
        }

        if (blur < 3) {
            blur = 3;
        }

        cv::GaussianBlur(gray, gray, cv::Size(blur, blur), 0);

        cv::threshold(gray, mask, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

        applyMorphology(mask);

        std::vector<std::vector<cv::Point> > contours;

        int chain_mode = cv::CHAIN_APPROX_NONE;

        if (contour_mode_ == "simple") {
            chain_mode = cv::CHAIN_APPROX_SIMPLE;
        }

        cv::findContours(mask, contours, cv::RETR_EXTERNAL, chain_mode);

        if (contours.empty()) {
            ROS_WARN_THROTTLE(2.0, "No s'ha detectat cap contorn.");
            return false;
        }

        double best_area = 0.0;
        int best_index = -1;

        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);

            if (area > min_area_ && area > best_area) {
                best_area = area;
                best_index = static_cast<int>(i);
            }
        }

        if (best_index < 0) {
            ROS_WARN_THROTTLE(2.0, "No hi ha cap contorn prou gran.");
            return false;
        }

        best_contour = contours[best_index];

        double perimeter = cv::arcLength(best_contour, true);
        double epsilon = epsilon_factor_ * perimeter;

        cv::approxPolyDP(best_contour, approx_contour, epsilon, true);

        if (approx_contour.size() < 3) {
            ROS_WARN_THROTTLE(2.0, "El contorn simplificat te massa pocs punts.");
            return false;
        }

        return true;
    }

    void applyMorphology(cv::Mat& mask)
    {
        int k = kernel_size_;

        if (k < 1 || morphology_mode_ == "none") {
            return;
        }

        if (k % 2 == 0) {
            k += 1;
        }

        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(k, k)
        );

        if (morphology_mode_ == "close") {
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), morphology_iterations_);
        }
        else if (morphology_mode_ == "open") {
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), morphology_iterations_);
        }
        else if (morphology_mode_ == "close_open") {
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), morphology_iterations_);
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), morphology_iterations_);
        }
        else if (morphology_mode_ == "open_close") {
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), morphology_iterations_);
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), morphology_iterations_);
        }
    }

    void publishContour(const std::vector<cv::Point>& contour, const cv::Rect& bbox)
    {
        geometry_msgs::PolygonStamped msg;

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "camera";

        for (const auto& p : contour) {
            geometry_msgs::Point32 point;

            point.x = static_cast<float>(p.x - bbox.x) / static_cast<float>(bbox.width);
            point.y = static_cast<float>(p.y - bbox.y) / static_cast<float>(bbox.height);
            point.z = 0.0;

            msg.polygon.points.push_back(point);
        }

        contour_pub_.publish(msg);
    }

    void publishContourInfo(const cv::Rect& bbox)
    {
        geometry_msgs::Vector3 info_msg;

        info_msg.x = bbox.width;
        info_msg.y = bbox.height;
        info_msg.z = static_cast<double>(bbox.width) / static_cast<double>(bbox.height);

        contour_info_pub_.publish(info_msg);
    }

    void showResult(const cv::Mat& image,
                    const cv::Mat& mask,
                    const std::vector<cv::Point>& original_contour,
                    const std::vector<cv::Point>& approx_contour)
    {
        if (!show_windows_) {
            return;
        }

        if (image.empty()) {
            return;
        }

        cv::Mat original_display = image.clone();
        cv::Mat mask_display;
        cv::Mat contour_display = image.clone();

        if (!mask.empty()) {
            cv::cvtColor(mask, mask_display, cv::COLOR_GRAY2BGR);
        }
        else {
            mask_display = cv::Mat::zeros(image.size(), image.type());
        }

        if (!original_contour.empty()) {
            std::vector<std::vector<cv::Point> > original_vector;
            original_vector.push_back(original_contour);
            cv::drawContours(contour_display, original_vector, -1, cv::Scalar(0, 255, 0), 2);
        }

        if (!approx_contour.empty()) {
            std::vector<std::vector<cv::Point> > approx_vector;
            approx_vector.push_back(approx_contour);
            cv::drawContours(contour_display, approx_vector, -1, cv::Scalar(0, 0, 255), 2);
        }

        cv::putText(original_display, "Original",
                    cv::Point(10, 25),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.7,
                    cv::Scalar(0, 0, 255),
                    2);

        cv::putText(mask_display, "Mascara",
                    cv::Point(10, 25),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.7,
                    cv::Scalar(0, 0, 255),
                    2);

        cv::putText(contour_display, "Verd: original | Vermell: simplificat",
                    cv::Point(10, 25),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.6,
                    cv::Scalar(0, 0, 255),
                    2);

        cv::resize(original_display, original_display, cv::Size(display_width_, display_height_));
        cv::resize(mask_display, mask_display, cv::Size(display_width_, display_height_));
        cv::resize(contour_display, contour_display, cv::Size(display_width_, display_height_));

        cv::Mat combined;
        cv::hconcat(original_display, mask_display, combined);
        cv::hconcat(combined, contour_display, combined);

        cv::imshow(window_name_, combined);
        cv::waitKey(1);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_node");

    VisionNode node;

    ros::spin();

    cv::destroyAllWindows();

    return 0;
}
