#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <cmath>
#include <string>
#include <limits>
#include <algorithm>

/*
 * Node de navegacio fins al punt inicial del dibuix.
 *
 * Aquest node no dibuixa. La seva feina es portar el robot fins a un punt adequat
 * del /drawing_path i orientar-lo amb la tangent de la trajectoria. Separar aquesta
 * etapa del follower evita que el retolador marqui el terra mentre el robot encara
 * s'esta posicionant.
 *
 * Entrada:
 *  - /drawing_path: trajectoria planificada.
 *  - /go_to_drawing: ordre de començar a anar al punt inicial.
 *
 * Sortida:
 *  - /cmd_vel: ordres de moviment.
 *  - /drawing_ready: true quan el robot ja esta preparat per baixar el retolador.
 */
class DrawingNavigatorNode
{
private:
    // Maquina d'estats simple: esperar, anar al punt, orientar-se i acabar.
    enum State
    {
        IDLE,
        GO_TO_POINT,
        ALIGN_TO_PATH,
        DONE
    };

    ros::NodeHandle nh_;

    ros::Subscriber path_sub_;
    ros::Subscriber go_sub_;

    ros::Publisher cmd_vel_pub_;
    ros::Publisher drawing_ready_pub_;
    ros::Publisher goal_marker_pub_;

    tf::TransformListener tf_listener_;

    nav_msgs::Path current_path_;

    // Frames i topics configurables des dels YAML.
    std::string global_frame_;
    std::string robot_frame_;
    std::string path_topic_;
    std::string go_topic_;
    std::string cmd_vel_topic_;
    std::string drawing_ready_topic_;

    // Flags interns de disponibilitat del path i publicacio de ready.
    bool has_path_;
    bool ready_published_;

    State state_;

    // Punt objectiu de navegacio i punt real del dibuix.
    geometry_msgs::Point goal_point_;
    geometry_msgs::Point drawing_goal_point_;

    int goal_index_;

    double final_yaw_;

    // Parametres de control proporcional i velocitats.
    double goal_tolerance_;
    double final_angle_tolerance_;

    double linear_speed_;
    double min_linear_speed_;

    double angular_speed_;
    double min_angular_speed_;

    double angular_correction_speed_;

    double kp_linear_;
    double kp_angular_;

    double min_goal_distance_;
    double tangent_distance_;

    double rotate_only_angle_;
    double control_rate_;
    double tool_offset_x_;
    double tool_offset_y_;

public:
    DrawingNavigatorNode()
    {
        ros::NodeHandle private_nh("~");

        private_nh.param<std::string>("global_frame", global_frame_, "odom");
        private_nh.param<std::string>("robot_frame", robot_frame_, "base_footprint");

        private_nh.param<std::string>("path_topic", path_topic_, "/drawing_path");
        private_nh.param<std::string>("go_topic", go_topic_, "/go_to_drawing");
        private_nh.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
        private_nh.param<std::string>("drawing_ready_topic", drawing_ready_topic_, "/drawing_ready");

        private_nh.param<double>("goal_tolerance", goal_tolerance_, 0.06);
        private_nh.param<double>("final_angle_tolerance", final_angle_tolerance_, 0.14);

        private_nh.param<double>("linear_speed", linear_speed_, 0.08);
        private_nh.param<double>("min_linear_speed", min_linear_speed_, 0.03);

        private_nh.param<double>("angular_speed", angular_speed_, 0.55);
        private_nh.param<double>("min_angular_speed", min_angular_speed_, 0.16);

        private_nh.param<double>("angular_correction_speed", angular_correction_speed_, 0.25);

        private_nh.param<double>("kp_linear", kp_linear_, 0.9);
        private_nh.param<double>("kp_angular", kp_angular_, 1.8);

        private_nh.param<double>("min_goal_distance", min_goal_distance_, 0.10);
        private_nh.param<double>("tangent_distance", tangent_distance_, 0.08);

        private_nh.param<double>("rotate_only_angle", rotate_only_angle_, 0.75);
        private_nh.param<double>("control_rate", control_rate_, 20.0);
        private_nh.param<double>("tool_offset_x", tool_offset_x_, 0.07);
        private_nh.param<double>("tool_offset_y", tool_offset_y_, 0.0);

        has_path_ = false;
        ready_published_ = false;
        state_ = IDLE;

        goal_index_ = 0;
        final_yaw_ = 0.0;

        path_sub_ = nh_.subscribe(path_topic_, 1, &DrawingNavigatorNode::pathCallback, this);
        go_sub_ = nh_.subscribe(go_topic_, 1, &DrawingNavigatorNode::goCallback, this);

        ensureCmdPublisher();
        drawing_ready_pub_ = nh_.advertise<std_msgs::Bool>(drawing_ready_topic_, 1, true);

        goal_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
            "/navigator_goal_marker",
            1,
            true
        );

        publishReady(false);

        ROS_INFO("drawing_navigator_node iniciat.");
        ROS_INFO("Mode simple robust: anar al punt inicial i orientar-se.");
        ROS_INFO("TF: %s -> %s", global_frame_.c_str(), robot_frame_.c_str());
    }

    // Guarda la trajectoria rebuda i calcula el punt inicial candidat.
    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        current_path_ = *msg;
        has_path_ = !current_path_.poses.empty();

        state_ = IDLE;
        ready_published_ = false;

        publishReady(false);
        stopRobot();

        if (has_path_) {
            ROS_INFO("Navigator: path rebut amb %lu punts.", current_path_.poses.size());
        }
        else {
            ROS_WARN("Navigator: path buit rebut.");
        }
    }

    // Ordre externa per començar la navegacio al punt inicial.
    void goCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (!msg->data) {
            return;
        }

        publishReady(false);
        ready_published_ = false;

        if (!has_path_) {
            ROS_WARN("Navigator: no hi ha /drawing_path encara.");
            return;
        }

        double robot_x;
        double robot_y;
        double robot_yaw;

        if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
            ROS_WARN("Navigator: no es pot obtenir la pose del robot.");
            return;
        }

        double tool_x;
        double tool_y;
        computeToolPoint(robot_x, robot_y, robot_yaw, tool_x, tool_y);

        if (!selectGoal(tool_x, tool_y)) {
            ROS_WARN("Navigator: no s'ha pogut seleccionar objectiu.");
            return;
        }

        ensureCmdPublisher();
        computeFinalYaw();
        applyToolOffsetToGoal();
        publishGoalMarker();

        state_ = GO_TO_POINT;

        ROS_INFO("Navigator: objectiu seleccionat:");
        ROS_INFO("  index = %d", goal_index_);
        ROS_INFO("  punt retolador x = %.3f, y = %.3f", drawing_goal_point_.x, drawing_goal_point_.y);
        ROS_INFO("  objectiu base x = %.3f, y = %.3f", goal_point_.x, goal_point_.y);
        ROS_INFO("  dist inicial = %.3f m", distance(robot_x, robot_y, goal_point_.x, goal_point_.y));
        ROS_INFO("Navigator: començant moviment cap al punt inicial.");
    }

    // Bucle de control a frequencia fixa. Segons l'estat, navega o orienta el robot.
    void controlLoop()
    {
        ros::Rate rate(control_rate_);

        while (ros::ok()) {
            ros::spinOnce();

            if (state_ == GO_TO_POINT || state_ == ALIGN_TO_PATH) {
                controlStep();
            }

            rate.sleep();
        }

        stopRobot();
    }

    // Decideix quina accio toca executar en funcio de l'estat actual.
    void controlStep()
    {
        double robot_x;
        double robot_y;
        double robot_yaw;

        if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
            stopRobot();
            return;
        }

        if (state_ == GO_TO_POINT) {
            goToPointStep(robot_x, robot_y, robot_yaw);
            return;
        }

        if (state_ == ALIGN_TO_PATH) {
            alignToPathStep(robot_yaw);
            return;
        }
    }

    // Control proporcional cap al punt objectiu. Combina avanç lineal i correccio angular.
    void goToPointStep(double robot_x, double robot_y, double robot_yaw)
    {
        double dx = goal_point_.x - robot_x;
        double dy = goal_point_.y - robot_y;

        double d = std::sqrt(dx * dx + dy * dy);

        if (d < goal_tolerance_) {
            stopRobot();

            state_ = ALIGN_TO_PATH;

            ROS_INFO("Navigator: punt inicial assolit. Ara orientant-se amb el dibuix.");
            return;
        }

        double target_yaw = std::atan2(dy, dx);
        double heading_error = normalizeAngle(target_yaw - robot_yaw);

        geometry_msgs::Twist cmd;

        if (std::fabs(heading_error) > rotate_only_angle_) {
            cmd.linear.x = 0.0;
            cmd.angular.z = kp_angular_ * heading_error;
            cmd.angular.z = clamp(cmd.angular.z, -angular_speed_, angular_speed_);
            cmd.angular.z = applyMinAngular(cmd.angular.z);
        }
        else {
            double v = kp_linear_ * d;
            v = clamp(v, min_linear_speed_, linear_speed_);

            if (d < 0.15) {
                v = std::min(v, 0.045);
            }

            double w = kp_angular_ * heading_error;
            w = clamp(w, -angular_correction_speed_, angular_correction_speed_);
            w = applyMinAngular(w);

            cmd.linear.x = v;
            cmd.angular.z = w;
        }

        cmd_vel_pub_.publish(cmd);

        ROS_INFO_THROTTLE(
            0.5,
            "Navigator GO_TO_POINT | dist=%.3f | err=%.2f rad | v=%.3f | w=%.3f",
            d,
            heading_error,
            cmd.linear.x,
            cmd.angular.z
        );
    }

    // Quan el robot ja es al punt inicial, gira sobre si mateix fins alinear-se amb el path.
    void alignToPathStep(double robot_yaw)
    {
        double error = normalizeAngle(final_yaw_ - robot_yaw);

        if (std::fabs(error) < final_angle_tolerance_) {
            stopRobot();

            state_ = DONE;

            if (!ready_published_) {
                publishReady(true);
                ready_published_ = true;

                ROS_INFO("Navigator: robot preparat al punt inicial.");
                ROS_INFO("Navigator: publicat /drawing_ready = true.");
            }

            releaseCmdPublisher();
            return;
        }

        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = kp_angular_ * error;
        cmd.angular.z = clamp(cmd.angular.z, -angular_speed_, angular_speed_);
        cmd.angular.z = applyMinAngular(cmd.angular.z);

        cmd_vel_pub_.publish(cmd);

        ROS_INFO_THROTTLE(
            0.5,
            "Navigator ALIGN_TO_PATH | err=%.2f rad | w=%.3f",
            error,
            cmd.angular.z
        );
    }

    // Obte la pose actual amb TF global_frame -> robot_frame.
    bool getRobotPose(double& x, double& y, double& yaw)
    {
        tf::StampedTransform transform;

        try {
            tf_listener_.lookupTransform(
                global_frame_,
                robot_frame_,
                ros::Time(0),
                transform
            );
        }
        catch (tf::TransformException& ex) {
            ROS_WARN_THROTTLE(
                1.0,
                "Navigator TF error %s -> %s: %s",
                global_frame_.c_str(),
                robot_frame_.c_str(),
                ex.what()
            );
            return false;
        }

        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
        yaw = tf::getYaw(transform.getRotation());

        return true;
    }

    // Escull un punt inicial del path que no estigui massa enganxat al robot i permeti
    // començar el traç amb una orientacio coherent.
    bool selectGoal(double robot_x, double robot_y)
    {
        if (current_path_.poses.empty()) {
            return false;
        }

        int n = static_cast<int>(current_path_.poses.size());

        if (n >= 2 && samePoint(0, n - 1)) {
            n = n - 1;
        }

        double best_distance = std::numeric_limits<double>::max();
        int best_index = -1;

        double fallback_distance = std::numeric_limits<double>::max();
        int fallback_index = -1;

        for (int i = 0; i < n; ++i) {
            double px = current_path_.poses[i].pose.position.x;
            double py = current_path_.poses[i].pose.position.y;

            double d = distance(robot_x, robot_y, px, py);

            if (d < fallback_distance) {
                fallback_distance = d;
                fallback_index = i;
            }

            if (d >= min_goal_distance_ && d < best_distance) {
                best_distance = d;
                best_index = i;
            }
        }

        if (best_index < 0) {
            best_index = fallback_index;
        }

        if (best_index < 0) {
            return false;
        }

        goal_index_ = best_index;
        drawing_goal_point_ = current_path_.poses[goal_index_].pose.position;
        goal_point_ = drawing_goal_point_;

        return true;
    }

    // Calcula la tangent final del path al punt inicial escollit.
    void computeFinalYaw()
    {
        int n = static_cast<int>(current_path_.poses.size());

        if (n < 2) {
            final_yaw_ = 0.0;
            return;
        }

        if (n >= 2 && samePoint(0, n - 1)) {
            n = n - 1;
        }

        double x0 = current_path_.poses[goal_index_].pose.position.x;
        double y0 = current_path_.poses[goal_index_].pose.position.y;

        int next_index = goal_index_;

        for (int step = 1; step < n; ++step) {
            int idx = (goal_index_ + step) % n;

            double x1 = current_path_.poses[idx].pose.position.x;
            double y1 = current_path_.poses[idx].pose.position.y;

            double d = distance(x0, y0, x1, y1);

            if (d >= tangent_distance_) {
                next_index = idx;
                break;
            }
        }

        double x1 = current_path_.poses[next_index].pose.position.x;
        double y1 = current_path_.poses[next_index].pose.position.y;

        final_yaw_ = std::atan2(y1 - y0, x1 - x0);

        ROS_INFO("Navigator: orientacio final abans de dibuixar = %.3f rad", final_yaw_);
    }

    // Si el retolador no coincideix amb el centre del robot, compensa l'objectiu.
    void applyToolOffsetToGoal()
    {
        double c = std::cos(final_yaw_);
        double s = std::sin(final_yaw_);

        goal_point_.x = drawing_goal_point_.x - c * tool_offset_x_ + s * tool_offset_y_;
        goal_point_.y = drawing_goal_point_.y - s * tool_offset_x_ - c * tool_offset_y_;
        goal_point_.z = drawing_goal_point_.z;
    }

    void computeToolPoint(double robot_x,
                          double robot_y,
                          double robot_yaw,
                          double& tool_x,
                          double& tool_y)
    {
        double c = std::cos(robot_yaw);
        double s = std::sin(robot_yaw);

        tool_x = robot_x + c * tool_offset_x_ - s * tool_offset_y_;
        tool_y = robot_y + s * tool_offset_x_ + c * tool_offset_y_;
    }

    bool samePoint(int i, int j)
    {
        double x1 = current_path_.poses[i].pose.position.x;
        double y1 = current_path_.poses[i].pose.position.y;

        double x2 = current_path_.poses[j].pose.position.x;
        double y2 = current_path_.poses[j].pose.position.y;

        return distance(x1, y1, x2, y2) < 0.03;
    }

    // Publica si el robot ja esta preparat per iniciar el dibuix.
    void publishReady(bool value)
    {
        std_msgs::Bool msg;
        msg.data = value;
        drawing_ready_pub_.publish(msg);
    }

    // Marker de RViz per visualitzar el punt on el robot anira abans de dibuixar.
    void publishGoalMarker()
    {
        visualization_msgs::Marker marker;

        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = global_frame_;

        marker.ns = "navigator_goal";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = goal_point_.x;
        marker.pose.position.y = goal_point_.y;
        marker.pose.position.z = 0.08;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.12;
        marker.scale.y = 0.12;
        marker.scale.z = 0.12;

        marker.color.r = 0.0;
        marker.color.g = 0.4;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        goal_marker_pub_.publish(marker);
    }

    void stopRobot()
    {
        if (!cmd_vel_pub_) {
            return;
        }

        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;

        cmd_vel_pub_.publish(cmd);
    }

    void ensureCmdPublisher()
    {
        if (cmd_vel_pub_) {
            return;
        }

        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
        ROS_INFO("Navigator: publisher %s activat.", cmd_vel_topic_.c_str());
    }

    void releaseCmdPublisher()
    {
        if (!cmd_vel_pub_) {
            return;
        }

        cmd_vel_pub_.shutdown();
        ROS_INFO("Navigator: publisher %s alliberat.", cmd_vel_topic_.c_str());
    }

    double applyMinAngular(double w)
    {
        if (std::fabs(w) < 1e-6) {
            return 0.0;
        }

        if (std::fabs(w) < min_angular_speed_) {
            if (w > 0.0) {
                return min_angular_speed_;
            }
            else {
                return -min_angular_speed_;
            }
        }

        return w;
    }

    double distance(double x1, double y1, double x2, double y2)
    {
        double dx = x1 - x2;
        double dy = y1 - y2;

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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drawing_navigator_node");

    DrawingNavigatorNode node;
    node.controlLoop();

    return 0;
}