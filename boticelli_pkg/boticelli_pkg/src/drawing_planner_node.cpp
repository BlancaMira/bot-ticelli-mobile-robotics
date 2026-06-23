#include <ros/ros.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

/*
 * Node de planificacio del dibuix.
 *
 * Rep el contorn detectat per la visio i les dimensions desitjades en metres.
 * Amb aquesta informacio genera un nav_msgs/Path en el frame map, que despres
 * utilitzen el navigator i el follower.
 *
 * El raonament es:
 *  - La visio dona punts normalitzats de la figura.
 *  - La interfície dona mida fisica del dibuix.
 *  - Aquest node centra la figura respecte el robot/mapa i la converteix en trajectoria.
 *  - Tambe publica un Marker per veure el dibuix previst a RViz.
 */
struct PathPoint
{
    double x;
    double y;
};

class DrawingPlannerNode
{
private:
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;

    ros::Subscriber contour_sub_;
    ros::Subscriber dimensions_sub_;

    ros::Publisher path_pub_;
    ros::Publisher marker_pub_;

    geometry_msgs::PolygonStamped last_contour_;
    geometry_msgs::Vector3 last_dimensions_;

    // El path nomes es pot construir quan ja han arribat tant el contorn com les dimensions.
    bool has_contour_;
    bool has_dimensions_;

    // Frames de referencia. El path final es publica en global_frame_, normalment "map".
    std::string global_frame_;
    std::string robot_frame_;

    std::string contour_topic_;
    std::string dimensions_topic_;
    std::string path_topic_;
    std::string marker_topic_;
    std::string anchor_mode_;

    // Opcions de construccio: permeten centrar/alinear el dibuix respecte la pose actual.
    bool use_robot_pose_as_center_;
    bool align_with_robot_heading_;
    bool invert_y_;
    bool close_path_;

    // Parametres metriques i de simplificacio de punts.
    double drawing_scale_;
    double min_width_;
    double min_height_;

    double min_point_distance_;
    double simplify_tolerance_;
    double resample_distance_;
    double marker_line_width_;

    // Centre i orientacio final del dibuix en el mapa.
    double center_x_;
    double center_y_;
    double center_yaw_;

public:
    DrawingPlannerNode()
    {
        ros::NodeHandle private_nh("~");

        private_nh.param<std::string>("global_frame", global_frame_, "map");
        private_nh.param<std::string>("robot_frame", robot_frame_, "base_link");

        private_nh.param<std::string>("contour_topic", contour_topic_, "/contour_points");
        private_nh.param<std::string>("dimensions_topic", dimensions_topic_, "/drawing_dimensions");

        private_nh.param<std::string>("path_topic", path_topic_, "/drawing_path");
        private_nh.param<std::string>("marker_topic", marker_topic_, "/drawing_marker");

        private_nh.param<std::string>("anchor_mode", anchor_mode_, "center");

        private_nh.param<bool>("use_robot_pose_as_center", use_robot_pose_as_center_, true);
        private_nh.param<bool>("align_with_robot_heading", align_with_robot_heading_, true);
        private_nh.param<bool>("invert_y", invert_y_, true);
        private_nh.param<bool>("close_path", close_path_, true);

        private_nh.param<double>("drawing_scale", drawing_scale_, 1.0);
        private_nh.param<double>("min_width", min_width_, 0.30);
        private_nh.param<double>("min_height", min_height_, 0.30);

        private_nh.param<double>("min_point_distance", min_point_distance_, 0.03);
        private_nh.param<double>("simplify_tolerance", simplify_tolerance_, 0.03);
        private_nh.param<double>("resample_distance", resample_distance_, 0.04);
        private_nh.param<double>("marker_line_width", marker_line_width_, 0.025);

        has_contour_ = false;
        has_dimensions_ = false;

        center_x_ = 0.0;
        center_y_ = 0.0;
        center_yaw_ = 0.0;

        contour_sub_ = nh_.subscribe(contour_topic_, 1, &DrawingPlannerNode::contourCallback, this);
        dimensions_sub_ = nh_.subscribe(dimensions_topic_, 1, &DrawingPlannerNode::dimensionsCallback, this);

        path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 1, true);

        ROS_INFO("drawing_planner_node iniciat.");
        ROS_INFO("Centre del dibuix = posicio actual del robot: %s",
                 use_robot_pose_as_center_ ? "true" : "false");
        ROS_INFO("Esperant %s i %s", contour_topic_.c_str(), dimensions_topic_.c_str());
    }

    // Rep el contorn de la visio. Encara no genera path si falten dimensions.
    void contourCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg)
    {
        last_contour_ = *msg;
        has_contour_ = true;

        ROS_INFO("Contorn rebut: %lu punts.", last_contour_.polygon.points.size());
        tryBuildDrawing();
    }

    // Rep la mida en metres triada per l'usuari.
    void dimensionsCallback(const geometry_msgs::Vector3::ConstPtr& msg)
    {
        last_dimensions_ = *msg;
        has_dimensions_ = true;

        ROS_INFO("Dimensions rebudes: %.3f x %.3f m",
                 last_dimensions_.x,
                 last_dimensions_.y);

        tryBuildDrawing();
    }

    // Usa TF per situar el centre del dibuix a la pose actual del robot.
    // Aixo fa que el dibuix es planifiqui on realment esta el robot.
    bool updateCenterFromRobotPose()
    {
        tf::StampedTransform transform;

        try {
            tf_listener_.lookupTransform(global_frame_, robot_frame_, ros::Time(0), transform);
        } catch (tf::TransformException& ex) {
            ROS_WARN("No es pot obtenir TF %s -> %s: %s",
                     global_frame_.c_str(),
                     robot_frame_.c_str(),
                     ex.what());
            return false;
        }

        center_x_ = transform.getOrigin().x();
        center_y_ = transform.getOrigin().y();
        center_yaw_ = tf::getYaw(transform.getRotation());

        ROS_INFO("Centre del dibuix fixat al robot: x=%.3f, y=%.3f, yaw=%.3f",
                 center_x_,
                 center_y_,
                 center_yaw_);

        return true;
    }

    // Construeix el path final. Fa escalat, possible inversio d'eix, simplificacio,
    // re-mostreig i publicacio del Path i del Marker de RViz.
    void tryBuildDrawing()
    {
        if (!has_contour_ || !has_dimensions_) {
            return;
        }

        if (last_contour_.polygon.points.empty()) {
            ROS_WARN("No es pot generar path: contorn buit.");
            return;
        }

        if (last_dimensions_.x <= 0.0 || last_dimensions_.y <= 0.0) {
            ROS_WARN("No es pot generar path: dimensions invalides.");
            return;
        }

        if (use_robot_pose_as_center_) {
            if (!updateCenterFromRobotPose()) {
                ROS_WARN("No es pot generar path encara: no hi ha localitzacio en map.");
                return;
            }
        }

        std::vector<PathPoint> raw_points = contourToMapPoints();
        std::vector<PathPoint> filtered = removeClosePoints(raw_points, min_point_distance_);

        std::vector<PathPoint> simplified;

        if (filtered.size() >= 3) {
            rdpSimplify(filtered, simplify_tolerance_, simplified);
        } else {
            simplified = filtered;
        }

        std::vector<PathPoint> final_points = resamplePath(simplified, resample_distance_);

        if (close_path_ && final_points.size() > 1) {
            final_points.push_back(final_points.front());
        }

        nav_msgs::Path path_msg = buildPath(final_points);
        visualization_msgs::Marker marker_msg = buildMarker(final_points);

        path_pub_.publish(path_msg);
        marker_pub_.publish(marker_msg);

        ROS_INFO("Dibuix generat en frame %s:", global_frame_.c_str());
        ROS_INFO(" - punts originals:    %lu", raw_points.size());
        ROS_INFO(" - punts filtrats:     %lu", filtered.size());
        ROS_INFO(" - punts simplificats: %lu", simplified.size());
        ROS_INFO(" - punts finals:       %lu", final_points.size());
        ROS_INFO("Publicat a %s i %s", path_topic_.c_str(), marker_topic_.c_str());
    }

    std::vector<PathPoint> contourToMapPoints()
    {
        std::vector<PathPoint> points;

        double width = last_dimensions_.x * drawing_scale_;
        double height = last_dimensions_.y * drawing_scale_;

        if (width < min_width_) {
            double factor = min_width_ / width;
            width *= factor;
            height *= factor;
        }

        if (height < min_height_) {
            double factor = min_height_ / height;
            width *= factor;
            height *= factor;
        }

        ROS_INFO("Dimensions finals aplicades: %.3f x %.3f m", width, height);

        for (const auto& p : last_contour_.polygon.points) {
            double local_x = p.x * width;
            double local_y = p.y * height;

            if (invert_y_) {
                local_y = height - local_y;
            }

            double map_x;
            double map_y;

            if (anchor_mode_ == "bottom_left") {
                map_x = center_x_ + local_x;
                map_y = center_y_ + local_y;
            } else {
                double dx = local_x - width / 2.0;
                double dy = local_y - height / 2.0;

                if (align_with_robot_heading_ && use_robot_pose_as_center_) {
                    double c = std::cos(center_yaw_);
                    double s = std::sin(center_yaw_);

                    // dy positiu = part superior del full = davant del robot.
                    // dx positiu = dreta del full = dreta del robot.
                    double forward = dy;
                    double left = -dx;

                    map_x = center_x_ + c * forward - s * left;
                    map_y = center_y_ + s * forward + c * left;
                } else {
                    map_x = center_x_ + dx;
                    map_y = center_y_ + dy;
                }
            }

            PathPoint point;
            point.x = map_x;
            point.y = map_y;

            points.push_back(point);
        }

        return points;
    }

    std::vector<PathPoint> removeClosePoints(const std::vector<PathPoint>& input, double min_dist)
    {
        std::vector<PathPoint> output;

        if (input.empty()) {
            return output;
        }

        output.push_back(input.front());

        for (size_t i = 1; i < input.size(); ++i) {
            if (distance(input[i], output.back()) >= min_dist) {
                output.push_back(input[i]);
            }
        }

        if (output.size() == 1 && input.size() > 1) {
            output.push_back(input.back());
        }

        return output;
    }

    // Algoritme Ramer-Douglas-Peucker: redueix punts redundants mantenint la forma.
    // Es útil per evitar trajectòries massa denses o sorolloses.
    void rdpSimplify(const std::vector<PathPoint>& points,
                     double epsilon,
                     std::vector<PathPoint>& result)
    {
        if (points.size() < 3) {
            result = points;
            return;
        }

        double max_dist = 0.0;
        size_t index = 0;

        for (size_t i = 1; i < points.size() - 1; ++i) {
            double d = perpendicularDistance(points[i], points.front(), points.back());

            if (d > max_dist) {
                max_dist = d;
                index = i;
            }
        }

        if (max_dist > epsilon) {
            std::vector<PathPoint> left(points.begin(), points.begin() + index + 1);
            std::vector<PathPoint> right(points.begin() + index, points.end());

            std::vector<PathPoint> left_result;
            std::vector<PathPoint> right_result;

            rdpSimplify(left, epsilon, left_result);
            rdpSimplify(right, epsilon, right_result);

            result = left_result;
            result.pop_back();
            result.insert(result.end(), right_result.begin(), right_result.end());
        } else {
            result.clear();
            result.push_back(points.front());
            result.push_back(points.back());
        }
    }

    // Distancia d'un punt a un segment, utilitzada per decidir quins punts es poden eliminar.
    double perpendicularDistance(const PathPoint& p,
                                 const PathPoint& a,
                                 const PathPoint& b)
    {
        double dx = b.x - a.x;
        double dy = b.y - a.y;

        if (std::fabs(dx) < 1e-9 && std::fabs(dy) < 1e-9) {
            return distance(p, a);
        }

        double numerator = std::fabs(dy * p.x - dx * p.y + b.x * a.y - b.y * a.x);
        double denominator = std::sqrt(dx * dx + dy * dy);

        return numerator / denominator;
    }

    std::vector<PathPoint> resamplePath(const std::vector<PathPoint>& input, double step)
    {
        std::vector<PathPoint> output;

        if (input.size() < 2) {
            return input;
        }

        output.push_back(input.front());

        for (size_t i = 0; i < input.size() - 1; ++i) {
            PathPoint a = input[i];
            PathPoint b = input[i + 1];

            double segment_length = distance(a, b);

            if (segment_length < 1e-6) {
                continue;
            }

            int n = static_cast<int>(std::floor(segment_length / step));

            for (int j = 1; j <= n; ++j) {
                double t = (j * step) / segment_length;

                if (t >= 1.0) {
                    continue;
                }

                PathPoint p;
                p.x = a.x + t * (b.x - a.x);
                p.y = a.y + t * (b.y - a.y);

                output.push_back(p);
            }

            output.push_back(b);
        }

        return output;
    }

    nav_msgs::Path buildPath(const std::vector<PathPoint>& points)
    {
        nav_msgs::Path path_msg;

        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = global_frame_;

        for (const auto& p : points) {
            geometry_msgs::PoseStamped pose;

            pose.header = path_msg.header;
            pose.pose.position.x = p.x;
            pose.pose.position.y = p.y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;

            path_msg.poses.push_back(pose);
        }

        return path_msg;
    }

    visualization_msgs::Marker buildMarker(const std::vector<PathPoint>& points)
    {
        visualization_msgs::Marker marker;

        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = global_frame_;

        marker.ns = "drawing";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = marker_line_width_;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.pose.orientation.w = 1.0;

        for (const auto& p : points) {
            geometry_msgs::Point point;
            point.x = p.x;
            point.y = p.y;
            point.z = 0.02;

            marker.points.push_back(point);
        }

        return marker;
    }

    double distance(const PathPoint& a, const PathPoint& b)
    {
        double dx = a.x - b.x;
        double dy = a.y - b.y;

        return std::sqrt(dx * dx + dy * dy);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drawing_planner_node");

    DrawingPlannerNode node;

    ros::spin();

    return 0;
}