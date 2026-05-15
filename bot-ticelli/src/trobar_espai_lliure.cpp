#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <cmath>
#include <vector>
#include <algorithm>
#include <sstream>
#include <queue>
#include <limits>

class FindFreeSpaceExplorer
{
private:
    enum Mode
    {
        EXPLORING,
        COMPUTING_GOAL,
        GOING_TO_GOAL,
        FINISHED
    };

    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Publisher state_pub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber map_sub_;

    tf::TransformListener tf_listener_;

    nav_msgs::OccupancyGrid::ConstPtr map_;

    double drawing_width_;
    double drawing_height_;
    double safety_margin_;
    double min_explore_time_;
    double goal_tolerance_;
    double max_linear_speed_;
    double max_angular_speed_;

    int occupied_threshold_;

    bool found_space_;
    bool goal_available_;

    double goal_x_;
    double goal_y_;

    int turn_direction_;

    ros::Time start_time_;
    Mode mode_;

public:
    FindFreeSpaceExplorer()
        : nh_("~"),
          found_space_(false),
          goal_available_(false),
          goal_x_(0.0),
          goal_y_(0.0),
          turn_direction_(1),
          mode_(EXPLORING)
    {
        nh_.param("drawing_width", drawing_width_, 0.60);
        nh_.param("drawing_height", drawing_height_, 0.40);
        nh_.param("safety_margin", safety_margin_, 0.25);

        // Ara aquest paràmetre vol dir:
        // temps mínim que el robot explora abans de buscar la millor zona del mapa.
        nh_.param("min_explore_time", min_explore_time_, 35.0);

        nh_.param("goal_tolerance", goal_tolerance_, 0.18);
        nh_.param("max_linear_speed", max_linear_speed_, 0.06);
        nh_.param("max_angular_speed", max_angular_speed_, 0.35);

        nh_.param("occupied_threshold", occupied_threshold_, 20);

        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        state_pub_ = nh_.advertise<std_msgs::String>("/debug_state", 10);

        scan_sub_ = nh_.subscribe("/scan", 1000, &FindFreeSpaceExplorer::scanCallback, this);
        map_sub_ = nh_.subscribe("/map", 1, &FindFreeSpaceExplorer::mapCallback, this);

        start_time_ = ros::Time::now();

        ROS_INFO("FindFreeSpaceExplorer iniciat.");
        ROS_INFO("Mida dibuix: %.2f x %.2f m, marge: %.2f m",
                 drawing_width_, drawing_height_, safety_margin_);
        ROS_INFO("Temps d'exploracio abans de buscar zona: %.2f s", min_explore_time_);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        map_ = msg;
    }

    void publishState(const std::string& text)
    {
        std_msgs::String msg;
        msg.data = text;
        state_pub_.publish(msg);
    }

    void stopRobot()
    {
        geometry_msgs::Twist vel;
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
        vel_pub_.publish(vel);
    }

    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    float cleanRange(float r, float range_max)
    {
        if (std::isnan(r) || std::isinf(r)) {
            return range_max;
        }
        return r;
    }

    bool angleInside(double angle, double start, double end)
    {
        while (angle < 0.0) angle += 2.0 * M_PI;
        while (angle >= 2.0 * M_PI) angle -= 2.0 * M_PI;

        while (start < 0.0) start += 2.0 * M_PI;
        while (start >= 2.0 * M_PI) start -= 2.0 * M_PI;

        while (end < 0.0) end += 2.0 * M_PI;
        while (end >= 2.0 * M_PI) end -= 2.0 * M_PI;

        if (start <= end) {
            return angle >= start && angle <= end;
        } else {
            return angle >= start || angle <= end;
        }
    }

    float minRangeDeg(const sensor_msgs::LaserScan::ConstPtr& msg, float start_deg, float end_deg)
    {
        if (msg->ranges.empty()) {
            return 0.0;
        }

        float range_max = msg->range_max;

        if (!std::isfinite(range_max) || range_max <= 0.0) {
            range_max = 3.5;
        }

        double start = start_deg * M_PI / 180.0;
        double end = end_deg * M_PI / 180.0;

        float min_value = range_max;

        for (int i = 0; i < static_cast<int>(msg->ranges.size()); ++i)
        {
            double angle = msg->angle_min + i * msg->angle_increment;

            if (angleInside(angle, start, end))
            {
                float r = cleanRange(msg->ranges[i], range_max);

                if (r > 0.0 && r < min_value) {
                    min_value = r;
                }
            }
        }

        return min_value;
    }

    bool worldToMap(double wx, double wy, int& mx, int& my)
    {
        if (!map_) return false;

        double origin_x = map_->info.origin.position.x;
        double origin_y = map_->info.origin.position.y;
        double origin_yaw = tf::getYaw(map_->info.origin.orientation);
        double res = map_->info.resolution;

        double dx = wx - origin_x;
        double dy = wy - origin_y;

        double c = std::cos(-origin_yaw);
        double s = std::sin(-origin_yaw);

        double local_x = c * dx - s * dy;
        double local_y = s * dx + c * dy;

        mx = static_cast<int>(std::floor(local_x / res));
        my = static_cast<int>(std::floor(local_y / res));

        if (mx < 0 || my < 0) return false;
        if (mx >= static_cast<int>(map_->info.width)) return false;
        if (my >= static_cast<int>(map_->info.height)) return false;

        return true;
    }

    bool mapToWorld(int mx, int my, double& wx, double& wy)
    {
        if (!map_) return false;

        double origin_x = map_->info.origin.position.x;
        double origin_y = map_->info.origin.position.y;
        double origin_yaw = tf::getYaw(map_->info.origin.orientation);
        double res = map_->info.resolution;

        double local_x = (mx + 0.5) * res;
        double local_y = (my + 0.5) * res;

        double c = std::cos(origin_yaw);
        double s = std::sin(origin_yaw);

        wx = origin_x + c * local_x - s * local_y;
        wy = origin_y + s * local_x + c * local_y;

        return true;
    }

    bool isCellFree(int mx, int my)
    {
        if (!map_) return false;

        int width = static_cast<int>(map_->info.width);
        int height = static_cast<int>(map_->info.height);

        if (mx < 0 || my < 0 || mx >= width || my >= height) {
            return false;
        }

        int index = my * width + mx;
        int value = map_->data[index];

        // -1 és desconegut. Ho considerem no vàlid.
        if (value < 0) return false;

        // 0 sol ser lliure, 100 ocupat.
        if (value > occupied_threshold_) return false;

        return true;
    }

    bool getRobotPose(double& x, double& y, double& yaw)
    {
        tf::StampedTransform transform;

        try {
            tf_listener_.lookupTransform("map", "base_footprint", ros::Time(0), transform);
        }
        catch (tf::TransformException& ex1) {
            try {
                tf_listener_.lookupTransform("map", "base_link", ros::Time(0), transform);
            }
            catch (tf::TransformException& ex2) {
                ROS_WARN_THROTTLE(1.0, "No puc obtenir TF map -> base_footprint/base_link.");
                return false;
            }
        }

        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
        yaw = tf::getYaw(transform.getRotation());

        return true;
    }

    bool findBestFreeSpace(double& best_x, double& best_y)
    {
        if (!map_) {
            return false;
        }

        int width = static_cast<int>(map_->info.width);
        int height = static_cast<int>(map_->info.height);
        double res = map_->info.resolution;

        if (width <= 0 || height <= 0 || res <= 0.0) {
            return false;
        }

        double required_width = drawing_width_ + 2.0 * safety_margin_;
        double required_height = drawing_height_ + 2.0 * safety_margin_;

        int half_w_cells = static_cast<int>(std::ceil((required_width / 2.0) / res));
        int half_h_cells = static_cast<int>(std::ceil((required_height / 2.0) / res));

        ROS_WARN("Analitzant mapa per trobar la millor zona lliure...");
        ROS_WARN("Rectangle requerit: %.2f x %.2f m", required_width, required_height);

        // Integral image de cel·les dolentes.
        // Cel·la dolenta = ocupada o desconeguda.
        std::vector<int> integral((width + 1) * (height + 1), 0);

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                int idx = y * width + x;
                int value = map_->data[idx];

                int bad = 0;

                if (value < 0 || value > occupied_threshold_) {
                    bad = 1;
                }

                int I = (y + 1) * (width + 1) + (x + 1);
                int I_up = y * (width + 1) + (x + 1);
                int I_left = (y + 1) * (width + 1) + x;
                int I_diag = y * (width + 1) + x;

                integral[I] = bad + integral[I_up] + integral[I_left] - integral[I_diag];
            }
        }

        auto rectBadCount = [&](int x0, int y0, int x1, int y1) -> int
        {
            int A = y0 * (width + 1) + x0;
            int B = y0 * (width + 1) + (x1 + 1);
            int C = (y1 + 1) * (width + 1) + x0;
            int D = (y1 + 1) * (width + 1) + (x1 + 1);

            return integral[D] - integral[B] - integral[C] + integral[A];
        };

        // valid_center[x,y] = 1 si el rectangle del dibuix cap centrat en aquesta cel·la.
        std::vector<unsigned char> valid_center(width * height, 0);

        for (int y = half_h_cells; y < height - half_h_cells; ++y)
        {
            for (int x = half_w_cells; x < width - half_w_cells; ++x)
            {
                int x0 = x - half_w_cells;
                int x1 = x + half_w_cells;
                int y0 = y - half_h_cells;
                int y1 = y + half_h_cells;

                int bad_count = rectBadCount(x0, y0, x1, y1);

                if (bad_count == 0) {
                    valid_center[y * width + x] = 1;
                }
            }
        }

        // Busquem components connexes de centres vàlids.
        std::vector<unsigned char> visited(width * height, 0);

        int best_count = 0;
        int best_mx = -1;
        int best_my = -1;

        int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                int idx = y * width + x;

                if (!valid_center[idx] || visited[idx]) {
                    continue;
                }

                std::queue<int> q;
                std::vector<int> component;

                visited[idx] = 1;
                q.push(idx);

                long long sum_x = 0;
                long long sum_y = 0;

                while (!q.empty())
                {
                    int current = q.front();
                    q.pop();

                    component.push_back(current);

                    int cx = current % width;
                    int cy = current / width;

                    sum_x += cx;
                    sum_y += cy;

                    for (int k = 0; k < 8; ++k)
                    {
                        int nx = cx + dx[k];
                        int ny = cy + dy[k];

                        if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                            continue;
                        }

                        int nidx = ny * width + nx;

                        if (valid_center[nidx] && !visited[nidx])
                        {
                            visited[nidx] = 1;
                            q.push(nidx);
                        }
                    }
                }

                int count = static_cast<int>(component.size());

                if (count > best_count)
                {
                    best_count = count;

                    double centroid_x = static_cast<double>(sum_x) / count;
                    double centroid_y = static_cast<double>(sum_y) / count;

                    double best_dist = std::numeric_limits<double>::max();

                    for (int cell_idx : component)
                    {
                        int cx = cell_idx % width;
                        int cy = cell_idx / width;

                        double d = std::pow(cx - centroid_x, 2.0) + std::pow(cy - centroid_y, 2.0);

                        if (d < best_dist)
                        {
                            best_dist = d;
                            best_mx = cx;
                            best_my = cy;
                        }
                    }
                }
            }
        }

        if (best_count <= 0 || best_mx < 0 || best_my < 0)
        {
            ROS_WARN("No s'ha trobat cap zona on hi càpiga el dibuix.");
            return false;
        }

        mapToWorld(best_mx, best_my, best_x, best_y);

        double valid_area = best_count * res * res;

        ROS_WARN("Millor zona trobada.");
        ROS_WARN("Area aproximada de centres valids: %.2f m2", valid_area);
        ROS_WARN("Objectiu centre: x = %.2f, y = %.2f", best_x, best_y);

        return true;
    }

    void chooseTurnDirection(float left_clear, float right_clear)
    {
        const float DIFF = 0.10;

        if (left_clear > right_clear + DIFF) {
            turn_direction_ = 1;
        }
        else if (right_clear > left_clear + DIFF) {
            turn_direction_ = -1;
        }
        // Si són quasi iguals, mantenim la direcció anterior.
        // Això evita que es quedi fent esquerra-dreta-esquerra-dreta.
    }

    void obstacleAvoidanceMove(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        geometry_msgs::Twist vel;

        if (msg->ranges.empty()) {
            stopRobot();
            return;
        }

        float front = minRangeDeg(msg, 315.0, 45.0);
        float front_left = minRangeDeg(msg, 20.0, 80.0);
        float front_right = minRangeDeg(msg, 280.0, 340.0);
        float left = minRangeDeg(msg, 70.0, 120.0);
        float right = minRangeDeg(msg, 240.0, 290.0);

        const float SAFE_DISTANCE = 0.70;
        const float VERY_CLOSE = 0.40;
        const float SIDE_DISTANCE = 0.55;

        float left_clear = std::min(front_left, left);
        float right_clear = std::min(front_right, right);

        bool very_close =
            front < VERY_CLOSE ||
            front_left < VERY_CLOSE ||
            front_right < VERY_CLOSE;

        bool obstacle_ahead =
            front < SAFE_DISTANCE ||
            front_left < SAFE_DISTANCE ||
            front_right < SAFE_DISTANCE;

        if (very_close)
        {
            chooseTurnDirection(left_clear, right_clear);

            vel.linear.x = 0.0;
            vel.angular.z = turn_direction_ * 0.35;

            ROS_WARN_THROTTLE(1.0,
                              "Obstacle molt a prop. front=%.2f front_left=%.2f front_right=%.2f",
                              front, front_left, front_right);
            publishState("exploring: obstacle molt a prop, giro mantenint direccio");
        }
        else if (obstacle_ahead)
        {
            chooseTurnDirection(left_clear, right_clear);

            vel.linear.x = 0.0;
            vel.angular.z = turn_direction_ * 0.25;

            ROS_INFO_THROTTLE(1.0,
                              "Obstacle davant. front=%.2f front_left=%.2f front_right=%.2f",
                              front, front_left, front_right);
            publishState("exploring: obstacle davant, giro");
        }
        else
        {
            vel.linear.x = 0.05;
            vel.angular.z = 0.0;

            if (left < SIDE_DISTANCE && right < SIDE_DISTANCE)
            {
                if (left < right) {
                    vel.angular.z = -0.20;
                } else {
                    vel.angular.z = 0.20;
                }

                publishState("exploring: passadis estret, centrant-se");
            }
            else if (left < SIDE_DISTANCE)
            {
                vel.angular.z = -0.20;
                publishState("exploring: massa a prop esquerra");
            }
            else if (right < SIDE_DISTANCE)
            {
                vel.angular.z = 0.20;
                publishState("exploring: massa a prop dreta");
            }
            else
            {
                publishState("exploring: avançant");
            }

            ROS_INFO_THROTTLE(1.0,
                              "Explorant. front=%.2f left=%.2f right=%.2f",
                              front, left, right);
        }

        vel_pub_.publish(vel);
    }

    void goToGoalMove(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        double robot_x, robot_y, robot_yaw;

        if (!getRobotPose(robot_x, robot_y, robot_yaw))
        {
            publishState("go_to_goal: no puc obtenir pose, paro");
            stopRobot();
            return;
        }

        double dx = goal_x_ - robot_x;
        double dy = goal_y_ - robot_y;

        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance < goal_tolerance_)
        {
            found_space_ = true;
            mode_ = FINISHED;
            stopRobot();

            std::stringstream ss;
            ss << "FINISHED: robot al centre de la millor zona. x="
               << robot_x << " y=" << robot_y;
            publishState(ss.str());

            ROS_WARN("Robot arribat a la millor zona lliure.");
            ROS_WARN("Posicio final: x = %.2f, y = %.2f", robot_x, robot_y);
            return;
        }

        float front = minRangeDeg(msg, 315.0, 45.0);
        float front_left = minRangeDeg(msg, 20.0, 80.0);
        float front_right = minRangeDeg(msg, 280.0, 340.0);

        // Si pel camí apareix un obstacle, fem evitació reactiva.
        if (front < 0.45 || front_left < 0.35 || front_right < 0.35)
        {
            publishState("go_to_goal: obstacle al cami, evitant");
            obstacleAvoidanceMove(msg);
            return;
        }

        double target_angle = std::atan2(dy, dx);
        double angle_error = normalizeAngle(target_angle - robot_yaw);

        geometry_msgs::Twist vel;

        if (std::fabs(angle_error) > 0.30)
        {
            vel.linear.x = 0.0;

            double angular_speed = std::min(max_angular_speed_, 0.8 * std::fabs(angle_error));

            if (angle_error > 0.0) {
                vel.angular.z = angular_speed;
            } else {
                vel.angular.z = -angular_speed;
            }

            publishState("go_to_goal: orientant-se cap al centre");
        }
        else
        {
            vel.linear.x = std::min(max_linear_speed_, 0.35 * distance);

            if (vel.linear.x < 0.025) {
                vel.linear.x = 0.025;
            }

            double angular_cmd = 1.0 * angle_error;

            if (angular_cmd > max_angular_speed_) angular_cmd = max_angular_speed_;
            if (angular_cmd < -max_angular_speed_) angular_cmd = -max_angular_speed_;

            vel.angular.z = angular_cmd;

            publishState("go_to_goal: avançant cap al centre");
        }

        ROS_INFO_THROTTLE(1.0,
                          "Anant al goal. dist=%.2f angle_error=%.2f goal=(%.2f, %.2f)",
                          distance, angle_error, goal_x_, goal_y_);

        vel_pub_.publish(vel);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        if (mode_ == FINISHED || found_space_)
        {
            stopRobot();
            return;
        }

        double elapsed = (ros::Time::now() - start_time_).toSec();

        if (mode_ == EXPLORING)
        {
            if (elapsed < min_explore_time_)
            {
                obstacleAvoidanceMove(msg);
                return;
            }

            if (!map_)
            {
                ROS_WARN_THROTTLE(1.0, "Encara no hi ha /map. Continuo explorant.");
                publishState("waiting: no hi ha mapa");
                obstacleAvoidanceMove(msg);
                return;
            }

            mode_ = COMPUTING_GOAL;
            stopRobot();

            ROS_WARN("Exploracio acabada. Ara busco la millor zona del mapa.");
            publishState("computing_goal: buscant millor zona lliure");
        }

        if (mode_ == COMPUTING_GOAL)
        {
            bool ok = findBestFreeSpace(goal_x_, goal_y_);

            if (!ok)
            {
                ROS_WARN("No hi ha zona bona encara. Exploro una mica mes.");
                publishState("computing_goal: no hi ha zona, continuo explorant");

                mode_ = EXPLORING;
                start_time_ = ros::Time::now();
                return;
            }

            goal_available_ = true;
            mode_ = GOING_TO_GOAL;

            std::stringstream ss;
            ss << "GOAL_FOUND: x=" << goal_x_ << " y=" << goal_y_;
            publishState(ss.str());

            ROS_WARN("Objectiu seleccionat. El robot anira al centre de la millor zona.");
            return;
        }

        if (mode_ == GOING_TO_GOAL)
        {
            if (!goal_available_)
            {
                publishState("go_to_goal: no hi ha goal disponible");
                stopRobot();
                return;
            }

            goToGoalMove(msg);
            return;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trobar_espai_lliure");

    FindFreeSpaceExplorer node;

    ros::spin();

    return 0;
}


