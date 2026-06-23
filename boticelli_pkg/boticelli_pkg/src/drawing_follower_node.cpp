#include <ros/ros.h>


#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>


#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>


#include <visualization_msgs/Marker.h>


#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>


struct PathPoint
{
    double x;
    double y;
};


class DrawingFollowerNode
{
private:
    enum State
    {
        WAITING,
        ALIGN_START,
        FOLLOWING,
        APPROACH_CORNER,
        ALIGN_CORNER,
        FINISHED
    };


    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;


    ros::Subscriber path_sub_;
    ros::Subscriber start_sub_;
    ros::Subscriber scan_sub_;


    ros::Publisher cmd_pub_;
    ros::Publisher executed_path_pub_;
    ros::Publisher lookahead_marker_pub_;
    ros::Publisher drawing_progress_pub_;
    ros::Publisher drawing_finished_pub_;


    std::vector<PathPoint> path_points_;
    std::vector<double> corner_angles_;


    nav_msgs::Path executed_path_;


    std::string global_frame_;
    std::string robot_frame_;
    std::string path_topic_;
    std::string cmd_vel_topic_;
    std::string obstacle_scan_topic_;


    State state_;


    bool has_path_;
    bool enabled_;
    bool require_start_command_;
    bool rotate_closed_path_to_nearest_;
    bool prealign_enabled_;
    bool use_tool_offset_control_;
    bool stop_at_corners_;
    bool closed_path_;
    bool reverse_closed_path_to_match_heading_;
    bool pause_on_obstacle_;
    bool obstacle_pause_active_;


    double robot_x_;
    double robot_y_;
    double robot_yaw_;
    double tool_x_;
    double tool_y_;


    double lookahead_distance_;
    double min_lookahead_distance_;


    double linear_speed_;
    double min_linear_speed_;
    double max_reverse_speed_;
    double curve_speed_factor_;
    double corner_speed_factor_;


    double max_angular_speed_;
    double angular_gain_;


    double goal_tolerance_;
    double waypoint_tolerance_;
    double max_start_distance_;


    double angle_tolerance_;
    double turn_in_place_angle_;


    double curve_angle_threshold_;
    double corner_angle_threshold_;


    double corner_detection_distance_;
    double corner_slowdown_distance_;
    double corner_stop_radius_;
    double corner_tangent_distance_;
    double tool_offset_x_;
    double tool_offset_y_;
    double tool_control_gain_;
    double closed_goal_tolerance_;
    double start_heading_weight_;
    double obstacle_stop_distance_;
    double obstacle_min_distance_;
    double obstacle_front_angle_deg_;


    int progress_index_;
    int progress_search_window_;
    int max_progress_index_step_;
    int current_corner_index_;
    int last_handled_corner_index_;
    int corner_back_search_;
    int corner_min_index_gap_;
    int closing_start_points_;
    int obstacle_detect_count_;
    int obstacle_clear_count_;


    double control_rate_;


public:
    DrawingFollowerNode()
    {
        ros::NodeHandle private_nh("~");


        private_nh.param<std::string>("global_frame", global_frame_, "odom");
        private_nh.param<std::string>("robot_frame", robot_frame_, "base_footprint");


        private_nh.param<std::string>("path_topic", path_topic_, "/drawing_path");
        private_nh.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
        private_nh.param<std::string>("obstacle_scan_topic", obstacle_scan_topic_, "/scan");


        private_nh.param<double>("lookahead_distance", lookahead_distance_, 0.13);
        private_nh.param<double>("min_lookahead_distance", min_lookahead_distance_, 0.07);


        private_nh.param<double>("linear_speed", linear_speed_, 0.045);
        private_nh.param<double>("min_linear_speed", min_linear_speed_, 0.018);
        private_nh.param<double>("max_reverse_speed", max_reverse_speed_, 0.012);
        private_nh.param<double>("curve_speed_factor", curve_speed_factor_, 0.65);
        private_nh.param<double>("corner_speed_factor", corner_speed_factor_, 0.40);


        private_nh.param<double>("max_angular_speed", max_angular_speed_, 0.65);
        private_nh.param<double>("angular_gain", angular_gain_, 1.8);


        private_nh.param<double>("goal_tolerance", goal_tolerance_, 0.06);
        private_nh.param<double>("waypoint_tolerance", waypoint_tolerance_, 0.055);
        private_nh.param<double>("max_start_distance", max_start_distance_, 0.35);


        private_nh.param<double>("angle_tolerance", angle_tolerance_, 0.10);
        private_nh.param<double>("turn_in_place_angle", turn_in_place_angle_, 0.80);


        private_nh.param<double>("curve_angle_threshold", curve_angle_threshold_, 0.35);
        private_nh.param<double>("corner_angle_threshold", corner_angle_threshold_, 0.80);


        private_nh.param<double>("corner_detection_distance", corner_detection_distance_, 0.30);
        private_nh.param<double>("corner_slowdown_distance", corner_slowdown_distance_, 0.20);
        private_nh.param<double>("corner_stop_radius", corner_stop_radius_, 0.065);
        private_nh.param<double>("corner_tangent_distance", corner_tangent_distance_, 0.08);


        private_nh.param<int>("progress_search_window", progress_search_window_, 70);
        private_nh.param<int>("max_progress_index_step", max_progress_index_step_, 4);
        private_nh.param<int>("corner_back_search", corner_back_search_, 5);
        private_nh.param<int>("corner_min_index_gap", corner_min_index_gap_, 4);


        private_nh.param<double>("control_rate", control_rate_, 20.0);


        private_nh.param<bool>("require_start_command", require_start_command_, true);
        private_nh.param<bool>("rotate_closed_path_to_nearest", rotate_closed_path_to_nearest_, true);
        private_nh.param<bool>("prealign_enabled", prealign_enabled_, true);
        private_nh.param<bool>("use_tool_offset_control", use_tool_offset_control_, true);
        private_nh.param<bool>("stop_at_corners", stop_at_corners_, false);
        private_nh.param<bool>("reverse_closed_path_to_match_heading", reverse_closed_path_to_match_heading_, true);
        private_nh.param<bool>("pause_on_obstacle", pause_on_obstacle_, true);
        private_nh.param<double>("tool_offset_x", tool_offset_x_, 0.07);
        private_nh.param<double>("tool_offset_y", tool_offset_y_, 0.0);
        private_nh.param<double>("tool_control_gain", tool_control_gain_, 1.4);
        private_nh.param<double>("closed_goal_tolerance", closed_goal_tolerance_, 0.035);
        private_nh.param<double>("start_heading_weight", start_heading_weight_, 0.08);
        private_nh.param<double>("obstacle_stop_distance", obstacle_stop_distance_, 0.30);
        private_nh.param<double>("obstacle_min_distance", obstacle_min_distance_, 0.12);
        private_nh.param<double>("obstacle_front_angle_deg", obstacle_front_angle_deg_, 20.0);
        private_nh.param<int>("closing_start_points", closing_start_points_, 3);


        has_path_ = false;
        closed_path_ = false;
        enabled_ = !require_start_command_;


        state_ = enabled_ ? FOLLOWING : WAITING;


        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_yaw_ = 0.0;
        tool_x_ = 0.0;
        tool_y_ = 0.0;
        obstacle_pause_active_ = false;
        obstacle_detect_count_ = 0;
        obstacle_clear_count_ = 0;


        progress_index_ = 0;
        current_corner_index_ = -1;
        last_handled_corner_index_ = -1000;


        path_sub_ = nh_.subscribe(path_topic_, 1, &DrawingFollowerNode::pathCallback, this);
        start_sub_ = nh_.subscribe("/start_drawing", 1, &DrawingFollowerNode::startCallback, this);
        scan_sub_ = nh_.subscribe(obstacle_scan_topic_, 1, &DrawingFollowerNode::scanCallback, this);


        executed_path_pub_ = nh_.advertise<nav_msgs::Path>("/executed_path", 1, true);
        lookahead_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/lookahead_marker", 1, true);
        drawing_progress_pub_ = nh_.advertise<geometry_msgs::Vector3>("/drawing_progress", 1, true);
        drawing_finished_pub_ = nh_.advertise<std_msgs::Bool>("/drawing_finished", 1, true);
        publishFinished(false);


        executed_path_.header.frame_id = global_frame_;


        ROS_INFO("drawing_follower_node iniciat.");
        ROS_INFO("Mode robust activat: seguiment del punt del retolador.");
        ROS_INFO("Retolador respecte %s: x=%.3f m, y=%.3f m",
                 robot_frame_.c_str(),
                 tool_offset_x_,
                 tool_offset_y_);
        ROS_INFO("TF: %s -> %s", global_frame_.c_str(), robot_frame_.c_str());
    }


    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        if (msg->poses.empty()) {
            ROS_WARN("Path buit rebut.");
            return;
        }


        if (enabled_ && state_ != FINISHED && state_ != WAITING) {
            ROS_WARN("S'ha rebut un path nou mentre s'esta dibuixant. S'ignora.");
            return;
        }


        path_points_.clear();


        for (const auto& pose : msg->poses) {
            PathPoint p;
            p.x = pose.pose.position.x;
            p.y = pose.pose.position.y;


            if (path_points_.empty() || distance(p, path_points_.back()) > 0.004) {
                path_points_.push_back(p);
            }
        }


        if (path_points_.size() < 3) {
            ROS_WARN("Path massa curt.");
            has_path_ = false;
            closed_path_ = false;
            return;
        }


        has_path_ = true;
        closed_path_ = isClosedPath(path_points_);
        enabled_ = !require_start_command_;
        state_ = enabled_ ? FOLLOWING : WAITING;


        if (!enabled_) {
            releaseCmdPublisher();
        }
        else {
            ensureCmdPublisher();
        }


        progress_index_ = 0;
        current_corner_index_ = -1;
        last_handled_corner_index_ = -1000;


        executed_path_.poses.clear();
        executed_path_.header.frame_id = global_frame_;

        publishFinished(false);
        publishProgress();

        computeCornerAngles();


        ROS_INFO("Path rebut amb %lu punts.", path_points_.size());
        ROS_INFO("Cantonades fortes detectades: %d", countStrongCorners());


        if (require_start_command_) {
            ROS_INFO("Esperant /start_drawing = true.");
        }
    }


    void startCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (!msg->data) {
            enabled_ = false;
            state_ = WAITING;
            publishStop();
            releaseCmdPublisher();
            ROS_WARN("STOP rebut. Robot aturat.");
            return;
        }


        if (!has_path_) {
            ROS_WARN("No es pot començar: encara no hi ha /drawing_path.");
            return;
        }


        if (!updateRobotPose(true)) {
            ROS_WARN("No es pot començar: no hi ha TF valid %s -> %s.",
                     global_frame_.c_str(),
                     robot_frame_.c_str());
            return;
        }


        int nearest_index = findNearestPathIndex(path_points_);
        double d = distanceToRobot(path_points_[nearest_index]);


        if (d > max_start_distance_) {
            ROS_WARN("No es comença el dibuix: el robot esta massa lluny del path.");
            ROS_WARN("Distancia al path: %.3f m | maxim permes: %.3f m", d, max_start_distance_);
            publishStop();
            return;
        }


        rotateClosedPathToNearestPoint();
        computeCornerAngles();


        progress_index_ = 0;
        current_corner_index_ = -1;
        last_handled_corner_index_ = -1000;


        executed_path_.poses.clear();


        enabled_ = true;
        state_ = prealign_enabled_ ? ALIGN_START : FOLLOWING;


        ensureCmdPublisher();
        publishStop();
        publishFinished(false);
        publishProgress();


        ROS_INFO("START rebut. Robot a %.3f m del path.", d);


        if (state_ == ALIGN_START) {
            ROS_INFO("Primer s'alineara amb la direccio inicial.");
        }
        else {
            ROS_INFO("Començant dibuix.");
        }
    }


    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        if (!pause_on_obstacle_) {
            obstacle_pause_active_ = false;
            obstacle_detect_count_ = 0;
            obstacle_clear_count_ = 0;
            return;
        }

        // Només s'aplica mentre s'està dibuixant.
        // Així evitem bloquejos abans de començar.
        if (!enabled_ || state_ == WAITING || state_ == FINISHED) {
            obstacle_pause_active_ = false;
            obstacle_detect_count_ = 0;
            obstacle_clear_count_ = 0;
            return;
        }

        bool detected_now = false;
        double max_angle = obstacle_front_angle_deg_ * M_PI / 180.0;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double range = msg->ranges[i];

            if (!std::isfinite(range)) {
                continue;
            }

            if (range < msg->range_min || range > msg->range_max) {
                continue;
            }

            // Ignorem lectures massa properes per evitar falsos positius del propi robot.
            if (range < obstacle_min_distance_) {
                continue;
            }

            double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;

            if (std::fabs(angle) <= max_angle && range < obstacle_stop_distance_) {
                detected_now = true;
                break;
            }
        }

        // Histeresi: no parem ni reprenem per una única lectura solta.
        if (detected_now) {
            obstacle_detect_count_++;
            obstacle_clear_count_ = 0;
        }
        else {
            obstacle_clear_count_++;
            obstacle_detect_count_ = 0;
        }

        if (obstacle_detect_count_ >= 3) {
            obstacle_pause_active_ = true;
        }

        if (obstacle_clear_count_ >= 5) {
            obstacle_pause_active_ = false;
        }
    }

    void controlLoop()
    {
        ros::Rate rate(control_rate_);


        while (ros::ok()) {
            ros::spinOnce();


            if (!enabled_ || !has_path_ || state_ == WAITING || state_ == FINISHED) {
                rate.sleep();
                continue;
            }


            if (!updateRobotPose(false)) {
                publishStop();
                rate.sleep();
                continue;
            }

            if (pause_on_obstacle_ && obstacle_pause_active_) {
                publishStop();
                ROS_WARN_THROTTLE(1.0, "Obstacle davant. Dibuix pausat fins que el cami quedi lliure.");
                rate.sleep();
                continue;
            }


            controlStep();


            rate.sleep();
        }


        publishStop();
    }


    void controlStep()
    {
        if (state_ == ALIGN_START) {
            double yaw = tangentYawFromIndex(0);
            alignToYaw(yaw, FOLLOWING, "Robot alineat. Comença el dibuix.");
            return;
        }


        if (state_ == ALIGN_CORNER) {
            double yaw = tangentYawFromIndex(current_corner_index_);
            alignToYaw(yaw, FOLLOWING, "Cantonada orientada. Continuant dibuix.");


            if (state_ == FOLLOWING) {
                last_handled_corner_index_ = current_corner_index_;
                progress_index_ = std::max(progress_index_, current_corner_index_);
                current_corner_index_ = -1;
            }


            return;
        }


        if (state_ == APPROACH_CORNER) {
            if (current_corner_index_ < 0 ||
                current_corner_index_ >= static_cast<int>(path_points_.size())) {
                state_ = FOLLOWING;
                current_corner_index_ = -1;
                return;
            }


            double d_corner = distanceToRobot(path_points_[current_corner_index_]);


            if (stop_at_corners_ && d_corner <= corner_stop_radius_) {
                publishStop();
                state_ = ALIGN_CORNER;


                ROS_INFO("Cantonada assolida a index %d. Girant quiet.",
                         current_corner_index_);
                return;
            }


            PathPoint target = path_points_[current_corner_index_];
            publishLookaheadMarker(target);
            publishCommandToTarget(target, corner_speed_factor_, min_lookahead_distance_);
            ROS_INFO_THROTTLE(
                0.8,
                "Apropant-se a cantonada fixada: index %d | dist %.3f m",
                current_corner_index_,
                d_corner
            );
            return;
        }


        updateProgressIndex();


        if (handleClosedPathFinish()) {
            return;
        }


        if (isGoalReached()) {
            publishStop();

            if (!path_points_.empty()) {
                progress_index_ = static_cast<int>(path_points_.size()) - 1;
            }

            publishProgress();
            publishFinished(true);

            releaseCmdPublisher();
            state_ = FINISHED;
            enabled_ = false;
            ROS_INFO("Dibuix finalitzat.");
            return;
        }


        int corner_index = -1;
        double corner_angle = 0.0;


        bool has_corner = findUpcomingCorner(corner_index, corner_angle);


        if (has_corner) {
            double d_corner = distanceToRobot(path_points_[corner_index]);


            if (stop_at_corners_ && d_corner <= corner_stop_radius_) {
                publishStop();


                current_corner_index_ = corner_index;
                state_ = ALIGN_CORNER;


                ROS_INFO("Cantonada detectada a index %d, angle %.1f graus. Girant quiet.",
                         corner_index,
                         corner_angle * 180.0 / M_PI);
                return;
            }


            if (stop_at_corners_ && d_corner <= corner_slowdown_distance_) {
                current_corner_index_ = corner_index;
                state_ = APPROACH_CORNER;
                ROS_INFO("Cantonada fixada a index %d, angle %.1f graus.",
                         corner_index,
                         corner_angle * 180.0 / M_PI);
                return;
            }
        }


        double local_angle = maxUpcomingAngle(0.22);
        double speed_factor = 1.0;
        double used_lookahead = lookahead_distance_;


        if (local_angle >= curve_angle_threshold_) {
            speed_factor = curve_speed_factor_;
            used_lookahead = std::max(min_lookahead_distance_, lookahead_distance_ * 0.75);
        }

        if (!stop_at_corners_ && has_corner) {
            speed_factor = std::min(speed_factor, corner_speed_factor_);
            used_lookahead = std::max(min_lookahead_distance_, lookahead_distance_ * 0.65);
        }


        PathPoint target = lookaheadTarget(used_lookahead);
        publishLookaheadMarker(target);
        publishCommandToTarget(target, speed_factor, used_lookahead);


        publishProgress();

        int percent = 0;
        if (path_points_.size() > 1) {
            percent = static_cast<int>(
                100.0 * static_cast<double>(progress_index_) /
                static_cast<double>(path_points_.size() - 1)
            );
        }

        if (percent < 0) {
            percent = 0;
        }
        if (percent > 100) {
            percent = 100;
        }

        ROS_INFO_THROTTLE(
            1.0,
            "Dibuix: %d/%lu punts (%d%%)",
            progress_index_ + 1,
            path_points_.size(),
            percent
        );
    }


    bool updateRobotPose(bool allow_log)
    {
        tf::StampedTransform transform;


        try {
            tf_listener_.lookupTransform(global_frame_, robot_frame_, ros::Time(0), transform);
        }
        catch (tf::TransformException& ex) {
            if (allow_log) {
                ROS_WARN("No es pot obtenir TF %s -> %s: %s",
                         global_frame_.c_str(),
                         robot_frame_.c_str(),
                         ex.what());
            }
            else {
                ROS_WARN_THROTTLE(2.0, "No es pot obtenir TF %s -> %s: %s",
                                  global_frame_.c_str(),
                                  robot_frame_.c_str(),
                                  ex.what());
            }
            return false;
        }


        robot_x_ = transform.getOrigin().x();
        robot_y_ = transform.getOrigin().y();
        robot_yaw_ = tf::getYaw(transform.getRotation());
        updateToolPoint();


        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = global_frame_;
        pose.pose.position.x = tool_x_;
        pose.pose.position.y = tool_y_;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_yaw_);


        executed_path_.header.stamp = ros::Time::now();
        executed_path_.poses.push_back(pose);
        executed_path_pub_.publish(executed_path_);


        return true;
    }


    void publishCommandToTarget(const PathPoint& target,
                                double speed_factor,
                                double used_lookahead)
    {
        geometry_msgs::Twist cmd;


        double dx = target.x - tool_x_;
        double dy = target.y - tool_y_;


        double target_yaw = std::atan2(dy, dx);
        double heading_error = normalizeAngle(target_yaw - robot_yaw_);


        double dist = std::sqrt(dx * dx + dy * dy);


        if (dist < 1e-5) {
            publishStop();
            return;
        }


        if (!use_tool_offset_control_ && std::fabs(heading_error) > turn_in_place_angle_) {
            cmd.linear.x = 0.0;
            cmd.angular.z = clamp(angular_gain_ * heading_error,
                                  -max_angular_speed_,
                                  max_angular_speed_);
            cmd_pub_.publish(cmd);
            return;
        }

        if (use_tool_offset_control_ && std::fabs(tool_offset_x_) > 0.02) {
            double desired_speed = linear_speed_ * speed_factor;

            if (dist < used_lookahead * 0.8) {
                desired_speed *= 0.70;
            }

            desired_speed = clamp(desired_speed, min_linear_speed_, linear_speed_);

            double ux = tool_control_gain_ * dx;
            double uy = tool_control_gain_ * dy;
            double u_norm = std::sqrt(ux * ux + uy * uy);

            if (u_norm > desired_speed) {
                double scale = desired_speed / u_norm;
                ux *= scale;
                uy *= scale;
            }

            double c = std::cos(robot_yaw_);
            double s = std::sin(robot_yaw_);
            double a = tool_offset_x_;
            double b = tool_offset_y_;

            double v = ((a * c - b * s) * ux + (a * s + b * c) * uy) / a;
            double w = (-s * ux + c * uy) / a;

            cmd.linear.x = clamp(v, -max_reverse_speed_, linear_speed_);
            cmd.angular.z = clamp(w, -max_angular_speed_, max_angular_speed_);

            cmd_pub_.publish(cmd);
            return;
        }


        double heading_factor = 1.0 - std::min(std::fabs(heading_error) / turn_in_place_angle_, 0.70);
        heading_factor = std::max(0.30, heading_factor);


        double v = linear_speed_ * speed_factor * heading_factor;


        if (dist < used_lookahead * 0.6) {
            v *= 0.65;
        }


        v = clamp(v, min_linear_speed_, linear_speed_);


        double w = angular_gain_ * heading_error;
        w = clamp(w, -max_angular_speed_, max_angular_speed_);


        cmd.linear.x = v;
        cmd.angular.z = w;


        cmd_pub_.publish(cmd);
    }


    void alignToYaw(double desired_yaw, State next_state, const std::string& done_msg)
    {
        double error = normalizeAngle(desired_yaw - robot_yaw_);


        if (std::fabs(error) < angle_tolerance_) {
            publishStop();
            state_ = next_state;
            ROS_INFO("%s", done_msg.c_str());
            return;
        }


        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = clamp(angular_gain_ * error,
                              -max_angular_speed_,
                              max_angular_speed_);


        cmd_pub_.publish(cmd);
    }

    void updateToolPoint()
    {
        double c = std::cos(robot_yaw_);
        double s = std::sin(robot_yaw_);

        tool_x_ = robot_x_ + c * tool_offset_x_ - s * tool_offset_y_;
        tool_y_ = robot_y_ + s * tool_offset_x_ + c * tool_offset_y_;
    }


    void updateProgressIndex()
    {
        if (path_points_.empty()) {
            return;
        }


        int last_index = static_cast<int>(path_points_.size()) - 1;


        int search_start = progress_index_;
        int search_end = std::min(last_index, progress_index_ + max_progress_index_step_);


        double best_dist = std::numeric_limits<double>::max();
        int best_index = progress_index_;


        for (int i = search_start; i <= search_end; ++i) {
            double d = distanceToRobot(path_points_[i]);


            if (d < best_dist) {
                best_dist = d;
                best_index = i;
            }
        }


        if (best_index > progress_index_) {
            progress_index_ = best_index;
        }


        while (progress_index_ < last_index - 1 &&
               distanceToRobot(path_points_[progress_index_]) < waypoint_tolerance_) {
            progress_index_++;
        }
    }


    PathPoint lookaheadTarget(double lookahead)
    {
        int last_index = static_cast<int>(path_points_.size()) - 1;


        if (progress_index_ >= last_index) {
            return path_points_.back();
        }


        double accumulated = 0.0;


        for (int i = progress_index_; i < last_index; ++i) {
            accumulated += distance(path_points_[i], path_points_[i + 1]);


            if (accumulated >= lookahead) {
                return path_points_[i + 1];
            }
        }


        return path_points_.back();
    }


    bool findUpcomingCorner(int& corner_index, double& angle)
    {
        if (path_points_.size() < 4) {
            return false;
        }


        int n = static_cast<int>(path_points_.size());
        int start = std::max(1, progress_index_ - corner_back_search_);
        int end = std::min(n - 2, progress_index_ + progress_search_window_);


        double accumulated = 0.0;


        for (int i = std::max(1, progress_index_); i <= end; ++i) {
            if (i > progress_index_) {
                accumulated += distance(path_points_[i - 1], path_points_[i]);
            }


            if (accumulated > corner_detection_distance_) {
                break;
            }
        }


        double path_distance = 0.0;


        for (int i = start; i <= end; ++i) {
            if (i > progress_index_) {
                path_distance += distance(path_points_[i - 1], path_points_[i]);
            }


            if (i > progress_index_ && path_distance > corner_detection_distance_) {
                break;
            }


            if (i <= last_handled_corner_index_ + corner_min_index_gap_) {
                continue;
            }


            if (corner_angles_[i] >= corner_angle_threshold_) {
                corner_index = i;
                angle = corner_angles_[i];
                return true;
            }
        }


        return false;
    }


    double maxUpcomingAngle(double max_distance)
    {
        if (corner_angles_.empty()) {
            return 0.0;
        }


        int n = static_cast<int>(path_points_.size());
        double accumulated = 0.0;
        double max_angle = 0.0;


        for (int i = progress_index_ + 1; i < n - 1; ++i) {
            accumulated += distance(path_points_[i - 1], path_points_[i]);


            if (accumulated > max_distance) {
                break;
            }


            max_angle = std::max(max_angle, corner_angles_[i]);
        }


        return max_angle;
    }


    bool isGoalReached()
    {
        if (path_points_.empty()) {
            return false;
        }

        if (closed_path_) {
            return false;
        }


        int last_index = static_cast<int>(path_points_.size()) - 1;


        if (progress_index_ < last_index - 3) {
            return false;
        }


        return distanceToRobot(path_points_.back()) < goal_tolerance_;
    }


    bool handleClosedPathFinish()
    {
        if (!closed_path_ || path_points_.size() < 4) {
            return false;
        }

        int last_index = static_cast<int>(path_points_.size()) - 1;

        if (progress_index_ < last_index - closing_start_points_) {
            return false;
        }

        PathPoint start = path_points_.front();
        double d_start = distanceToRobot(start);

        if (d_start <= closed_goal_tolerance_) {
            publishStop();

            progress_index_ = last_index;
            publishProgress();
            publishFinished(true);

            releaseCmdPublisher();
            state_ = FINISHED;
            enabled_ = false;
            ROS_INFO("Dibuix tancat. Distancia final a l'inici: %.3f m", d_start);
            return true;
        }

        publishLookaheadMarker(start);
        publishCommandToTarget(start, corner_speed_factor_, min_lookahead_distance_);
        ROS_INFO_THROTTLE(
            0.5,
            "Tancant path | progress=%d/%d | dist inici=%.3f m",
            progress_index_,
            last_index,
            d_start
        );
        return true;
    }


    void computeCornerAngles()
    {
        corner_angles_.assign(path_points_.size(), 0.0);


        if (path_points_.size() < 3) {
            return;
        }


        for (size_t i = 1; i + 1 < path_points_.size(); ++i) {
            PathPoint a = path_points_[i - 1];
            PathPoint b = path_points_[i];
            PathPoint c = path_points_[i + 1];


            double v1x = b.x - a.x;
            double v1y = b.y - a.y;
            double v2x = c.x - b.x;
            double v2y = c.y - b.y;


            double n1 = std::sqrt(v1x * v1x + v1y * v1y);
            double n2 = std::sqrt(v2x * v2x + v2y * v2y);


            if (n1 < 1e-6 || n2 < 1e-6) {
                corner_angles_[i] = 0.0;
                continue;
            }


            double dot = (v1x * v2x + v1y * v2y) / (n1 * n2);
            dot = clamp(dot, -1.0, 1.0);


            corner_angles_[i] = std::acos(dot);
        }
    }


    int countStrongCorners()
    {
        int count = 0;


        for (double a : corner_angles_) {
            if (a >= corner_angle_threshold_) {
                count++;
            }
        }


        return count;
    }


    double tangentYawFromIndex(int index)
    {
        if (path_points_.size() < 2) {
            return robot_yaw_;
        }


        int n = static_cast<int>(path_points_.size());
        index = clampInt(index, 0, n - 2);


        PathPoint p0 = path_points_[index];


        int next_index = std::min(index + 1, n - 1);


        for (int i = index + 1; i < n; ++i) {
            if (distance(p0, path_points_[i]) >= corner_tangent_distance_) {
                next_index = i;
                break;
            }
        }


        PathPoint p1 = path_points_[next_index];


        return std::atan2(p1.y - p0.y, p1.x - p0.x);
    }


    bool isClosedPath(const std::vector<PathPoint>& path)
    {
        if (path.size() < 3) {
            return false;
        }


        return distance(path.front(), path.back()) < goal_tolerance_;
    }


    int findNearestPathIndex(const std::vector<PathPoint>& path)
    {
        double best_dist = std::numeric_limits<double>::max();
        int best_index = 0;


        for (size_t i = 0; i < path.size(); ++i) {
            double d = distanceToRobot(path[i]);


            if (d < best_dist) {
                best_dist = d;
                best_index = static_cast<int>(i);
            }
        }


        return best_index;
    }


    void rotateClosedPathToNearestPoint()
    {
        if (!rotate_closed_path_to_nearest_) {
            return;
        }


        if (!isClosedPath(path_points_)) {
            ROS_INFO("Path obert: no es rota el punt inicial.");
            return;
        }


        std::vector<PathPoint> base_path = path_points_;


        if (distance(base_path.front(), base_path.back()) < goal_tolerance_) {
            base_path.pop_back();
        }


        if (base_path.size() < 3) {
            return;
        }


        bool reverse_selected = false;
        int nearest_index = findBestClosedPathStart(base_path, reverse_selected);

        if (reverse_selected && reverse_closed_path_to_match_heading_) {
            std::reverse(base_path.begin(), base_path.end());
            nearest_index = static_cast<int>(base_path.size()) - 1 - nearest_index;
            ROS_INFO("Path tancat invertit per evitar el primer gir brusc.");
        }


        std::vector<PathPoint> rotated_path;


        for (size_t i = nearest_index; i < base_path.size(); ++i) {
            rotated_path.push_back(base_path[i]);
        }


        for (int i = 0; i < nearest_index; ++i) {
            rotated_path.push_back(base_path[i]);
        }


        rotated_path.push_back(rotated_path.front());


        path_points_ = rotated_path;


        closed_path_ = true;

        ROS_INFO("Path tancat rotat. Nou inici = %d | reverse_selected=%s",
                 nearest_index,
                 reverse_selected ? "true" : "false");
    }


    int findBestClosedPathStart(const std::vector<PathPoint>& path, bool& reverse_selected)
    {
        double best_score = std::numeric_limits<double>::max();
        int best_index = findNearestPathIndex(path);
        reverse_selected = false;

        for (size_t i = 0; i < path.size(); ++i) {
            double d = distanceToRobot(path[i]);

            double fwd_error = std::fabs(normalizeAngle(tangentYawForPath(path, static_cast<int>(i), true) - robot_yaw_));
            double rev_error = std::fabs(normalizeAngle(tangentYawForPath(path, static_cast<int>(i), false) - robot_yaw_));

            double fwd_score = d + start_heading_weight_ * fwd_error;
            double rev_score = d + start_heading_weight_ * rev_error;

            if (fwd_score < best_score) {
                best_score = fwd_score;
                best_index = static_cast<int>(i);
                reverse_selected = false;
            }

            if (reverse_closed_path_to_match_heading_ && rev_score < best_score) {
                best_score = rev_score;
                best_index = static_cast<int>(i);
                reverse_selected = true;
            }
        }

        return best_index;
    }


    double tangentYawForPath(const std::vector<PathPoint>& path, int index, bool forward)
    {
        int n = static_cast<int>(path.size());

        if (n < 2) {
            return robot_yaw_;
        }

        index = clampInt(index, 0, n - 1);

        int other_index;

        if (forward) {
            other_index = (index + 1) % n;
            return std::atan2(path[other_index].y - path[index].y,
                              path[other_index].x - path[index].x);
        }

        other_index = (index - 1 + n) % n;
        return std::atan2(path[other_index].y - path[index].y,
                          path[other_index].x - path[index].x);
    }


    void publishLookaheadMarker(const PathPoint& target)
    {
        visualization_msgs::Marker marker;


        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = global_frame_;


        marker.ns = "lookahead";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;


        marker.pose.position.x = target.x;
        marker.pose.position.y = target.y;
        marker.pose.position.z = 0.05;
        marker.pose.orientation.w = 1.0;


        marker.scale.x = 0.08;
        marker.scale.y = 0.08;
        marker.scale.z = 0.08;


        marker.color.r = 1.0;
        marker.color.g = 0.6;
        marker.color.b = 0.0;
        marker.color.a = 1.0;


        lookahead_marker_pub_.publish(marker);
    }


    void publishProgress()
    {
        if (!drawing_progress_pub_) {
            return;
        }

        geometry_msgs::Vector3 msg;

        msg.x = static_cast<double>(progress_index_);
        msg.y = static_cast<double>(path_points_.size());

        if (path_points_.size() <= 1) {
            msg.z = 0.0;
        }
        else {
            msg.z =
                100.0 * static_cast<double>(progress_index_) /
                static_cast<double>(path_points_.size() - 1);
        }

        if (msg.z < 0.0) {
            msg.z = 0.0;
        }

        if (msg.z > 100.0) {
            msg.z = 100.0;
        }

        drawing_progress_pub_.publish(msg);
    }


    void publishFinished(bool finished)
    {
        if (!drawing_finished_pub_) {
            return;
        }

        std_msgs::Bool msg;
        msg.data = finished;
        drawing_finished_pub_.publish(msg);
    }


    void publishStop()
    {
        if (!cmd_pub_) {
            return;
        }


        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;


        cmd_pub_.publish(cmd);
    }


    void ensureCmdPublisher()
    {
        if (cmd_pub_) {
            return;
        }


        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
        ROS_INFO("Follower: publisher %s activat.", cmd_vel_topic_.c_str());
    }


    void releaseCmdPublisher()
    {
        if (!cmd_pub_) {
            return;
        }


        cmd_pub_.shutdown();
        ROS_INFO("Follower: publisher %s alliberat.", cmd_vel_topic_.c_str());
    }


    double distanceToRobot(const PathPoint& p)
    {
        double dx = p.x - tool_x_;
        double dy = p.y - tool_y_;


        return std::sqrt(dx * dx + dy * dy);
    }


    double distance(const PathPoint& a, const PathPoint& b)
    {
        double dx = a.x - b.x;
        double dy = a.y - b.y;


        return std::sqrt(dx * dx + dy * dy);
    }


    double normalizeAngle(double angle)
    {
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }


        while (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }


        return angle;
    }


    double clamp(double value, double min_value, double max_value)
    {
        return std::max(min_value, std::min(value, max_value));
    }


    int clampInt(int value, int min_value, int max_value)
    {
        return std::max(min_value, std::min(value, max_value));
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "drawing_follower_node");


    DrawingFollowerNode node;
    node.controlLoop();


    return 0;
}
