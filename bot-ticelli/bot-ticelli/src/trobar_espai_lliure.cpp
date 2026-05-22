#include <ros/ros.h>


#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>


#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>


#include <std_msgs/String.h>
#include <std_msgs/Bool.h>


#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


#include <cmath>
#include <vector>
#include <algorithm>
#include <sstream>
#include <queue>
#include <limits>
#include <cstdlib>
#include <string>
#include <fstream>
#include <iomanip>


class FindFreeSpaceExplorer
{
private:
    struct Landmark
    {
        std::string type;
        geometry_msgs::PoseStamped pose;
        double score;
    };


    struct ScanPoint
    {
        double x;
        double y;
        double angle;
        double range;
        int index;
    };


    struct WallSegment
    {
        double x;
        double y;
        double yaw;
        double length;
        double error;
        int start_index;
        int end_index;
    };


    enum Mode
    {
        EXPLORING,
        RETURNING_TO_START,
        COMPUTING_GOAL,
        GOING_TO_GOAL,
        FINISHED
    };


    ros::NodeHandle nh_;


    ros::Publisher vel_pub_;
    ros::Publisher state_pub_;
    ros::Publisher drawing_center_pub_;
    ros::Publisher free_area_info_pub_;
    ros::Publisher navigation_done_pub_;
    ros::Publisher current_phase_pub_;


    ros::Subscriber scan_sub_;
    ros::Subscriber map_sub_;


    tf::TransformListener tf_listener_;


    nav_msgs::OccupancyGrid::ConstPtr map_;


    double drawing_width_;
    double drawing_height_;
    double safety_margin_;


    double min_explore_time_;
    double max_explore_time_;
    double map_stable_time_;
    double map_growth_epsilon_;


    double goal_tolerance_;
    double max_linear_speed_;
    double max_angular_speed_;


    int occupied_threshold_;


    bool auto_save_map_;
    bool map_saved_;
    std::string map_output_file_;
    std::string landmarks_output_file_;


    bool found_space_;
    bool goal_available_;


    double goal_x_;
    double goal_y_;


    bool start_pose_set_;
    double start_x_;
    double start_y_;
    double start_yaw_;


    double landmark_min_distance_;
    double landmark_min_time_;
    double feature_landmark_min_distance_;
    double feature_landmark_min_time_;
    double obstacle_landmark_max_range_;
    double corner_landmark_max_range_;
    double corner_jump_threshold_;
    double wall_landmark_max_range_;
    double wall_split_jump_;
    double wall_min_length_;
    double wall_max_fit_error_;
    double wall_corner_angle_threshold_;
    int wall_min_points_;
    int max_corner_landmarks_;
    int max_obstacle_landmarks_;
    int max_wall_landmarks_;
    double last_landmark_x_;
    double last_landmark_y_;
    ros::Time last_landmark_time_;
    ros::Time last_feature_landmark_time_;


    std::vector<Landmark> landmarks_;


    int turn_direction_;


    ros::Time start_time_;
    ros::Time last_map_growth_time_;


    Mode mode_;


    double known_area_;
    double free_area_;
    double occupied_area_;
    double best_known_area_;


public:
    FindFreeSpaceExplorer()
        : nh_("~"),
          auto_save_map_(true),
          map_saved_(false),
          found_space_(false),
          goal_available_(false),
          goal_x_(0.0),
          goal_y_(0.0),
          start_pose_set_(false),
          start_x_(0.0),
          start_y_(0.0),
          start_yaw_(0.0),
          landmark_min_distance_(0.50),
          landmark_min_time_(5.0),
          feature_landmark_min_distance_(0.45),
          feature_landmark_min_time_(1.0),
          obstacle_landmark_max_range_(1.20),
          corner_landmark_max_range_(1.80),
          corner_jump_threshold_(1.00),
          wall_landmark_max_range_(2.50),
          wall_split_jump_(0.20),
          wall_min_length_(0.45),
          wall_max_fit_error_(0.04),
          wall_corner_angle_threshold_(0.70),
          wall_min_points_(8),
          max_corner_landmarks_(12),
          max_obstacle_landmarks_(8),
          max_wall_landmarks_(20),
          last_landmark_x_(0.0),
          last_landmark_y_(0.0),
          turn_direction_(1),
          mode_(EXPLORING),
          known_area_(0.0),
          free_area_(0.0),
          occupied_area_(0.0),
          best_known_area_(0.0)
    {
        nh_.param("drawing_width", drawing_width_, 0.40);
        nh_.param("drawing_height", drawing_height_, 0.30);
        nh_.param("safety_margin", safety_margin_, 0.10);


        nh_.param("min_explore_time", min_explore_time_, 90.0);
        nh_.param("max_explore_time", max_explore_time_, 180.0);
        nh_.param("map_stable_time", map_stable_time_, 12.0);
        nh_.param("map_growth_epsilon", map_growth_epsilon_, 0.15);


        nh_.param("goal_tolerance", goal_tolerance_, 0.18);
        nh_.param("max_linear_speed", max_linear_speed_, 0.06);
        nh_.param("max_angular_speed", max_angular_speed_, 0.35);


        nh_.param("occupied_threshold", occupied_threshold_, 20);


        nh_.param("auto_save_map", auto_save_map_, true);
        nh_.param<std::string>("map_output_file", map_output_file_, "/tmp/mapa_boticelli_real");
        nh_.param<std::string>("landmarks_output_file", landmarks_output_file_, map_output_file_ + "_landmarks.txt");
        nh_.param("landmark_min_distance", landmark_min_distance_, 0.50);
        nh_.param("landmark_min_time", landmark_min_time_, 5.0);
        nh_.param("feature_landmark_min_distance", feature_landmark_min_distance_, 0.45);
        nh_.param("feature_landmark_min_time", feature_landmark_min_time_, 1.0);
        nh_.param("obstacle_landmark_max_range", obstacle_landmark_max_range_, 1.20);
        nh_.param("corner_landmark_max_range", corner_landmark_max_range_, 1.80);
        nh_.param("corner_jump_threshold", corner_jump_threshold_, 1.00);
        nh_.param("wall_landmark_max_range", wall_landmark_max_range_, 2.50);
        nh_.param("wall_split_jump", wall_split_jump_, 0.20);
        nh_.param("wall_min_length", wall_min_length_, 0.45);
        nh_.param("wall_max_fit_error", wall_max_fit_error_, 0.04);
        nh_.param("wall_corner_angle_threshold", wall_corner_angle_threshold_, 0.70);
        nh_.param("wall_min_points", wall_min_points_, 8);
        nh_.param("max_corner_landmarks", max_corner_landmarks_, 12);
        nh_.param("max_obstacle_landmarks", max_obstacle_landmarks_, 8);
        nh_.param("max_wall_landmarks", max_wall_landmarks_, 20);


        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        state_pub_ = nh_.advertise<std_msgs::String>("/debug_state", 10);


        drawing_center_pub_ =
            nh_.advertise<geometry_msgs::PoseStamped>("/boticelli/drawing_center", 1, true);


        free_area_info_pub_ =
            nh_.advertise<std_msgs::String>("/boticelli/free_area_info", 1, true);


        navigation_done_pub_ =
            nh_.advertise<std_msgs::Bool>("/boticelli/navigation_done", 1, true);


        current_phase_pub_ =
            nh_.advertise<std_msgs::String>("/boticelli/current_phase", 1, true);


        scan_sub_ = nh_.subscribe("/scan", 1000, &FindFreeSpaceExplorer::scanCallback, this);
        map_sub_ = nh_.subscribe("/map", 1, &FindFreeSpaceExplorer::mapCallback, this);


        start_time_ = ros::Time::now();
        last_map_growth_time_ = start_time_;


        publishNavigationDone(false);
        publishPhase("SLAM_EXPLORING");


        ROS_WARN("=================================================");
        ROS_WARN("[BOT-TICELLI] Node trobar_espai_lliure iniciat.");
        ROS_WARN("[BOT-TICELLI] Fase 1: SLAM autonom + exploracio + mapa + zona optima.");
        ROS_WARN("[PARAMS] Mida dibuix: %.2f x %.2f m", drawing_width_, drawing_height_);
        ROS_WARN("[PARAMS] Marge de seguretat: %.2f m", safety_margin_);
        ROS_WARN("[PARAMS] Temps minim exploracio: %.2f s", min_explore_time_);
        ROS_WARN("[PARAMS] Temps maxim exploracio: %.2f s", max_explore_time_);
        ROS_WARN("[PARAMS] Temps mapa estable: %.2f s", map_stable_time_);
        ROS_WARN("[PARAMS] Creixement minim mapa: %.2f m2", map_growth_epsilon_);
        ROS_WARN("[PARAMS] Auto guardar mapa: %s", auto_save_map_ ? "true" : "false");
        ROS_WARN("[PARAMS] Fitxer mapa: %s", map_output_file_.c_str());
        ROS_WARN("[PARAMS] Fitxer landmarks: %s", landmarks_output_file_.c_str());
        ROS_WARN("[PARAMS] Separacio minima landmarks: %.2f m | temps minim: %.2f s",
                 landmark_min_distance_, landmark_min_time_);
        ROS_WARN("[PARAMS] Separacio landmarks mapa: %.2f m | temps minim features: %.2f s",
                 feature_landmark_min_distance_, feature_landmark_min_time_);
        ROS_WARN("[PARAMS] Obstacle max: %.2f m | cantonada max: %.2f m | salt cantonada: %.2f m",
                 obstacle_landmark_max_range_, corner_landmark_max_range_, corner_jump_threshold_);
        ROS_WARN("[PARAMS] Paret max: %.2f m | split: %.2f m | min length: %.2f m | fit error: %.2f m | min punts: %d",
                 wall_landmark_max_range_, wall_split_jump_, wall_min_length_,
                 wall_max_fit_error_, wall_min_points_);
        ROS_WARN("[PARAMS] Max walls: %d | max corners: %d | max obstacles: %d",
                 max_wall_landmarks_, max_corner_landmarks_, max_obstacle_landmarks_);
        ROS_WARN("[PARAMS] Llindar ocupacio: %d", occupied_threshold_);
        ROS_WARN("=================================================");
    }


    int countLandmarksOfType(const std::string& type)
    {
        int count = 0;


        for (size_t i = 0; i < landmarks_.size(); ++i)
        {
            if (landmarks_[i].type == type) {
                count++;
            }
        }


        return count;
    }


    int addLandmark(const std::string& type,
                    double x,
                    double y,
                    double yaw,
                    double score,
                    bool allow_near_duplicate)
    {
        if (type == "corner" && countLandmarksOfType("corner") >= max_corner_landmarks_)
        {
            ROS_INFO_THROTTLE(5.0,
                              "[LANDMARKS] Ignoro cantonada nova: ja he arribat al maxim de %d corners.",
                              max_corner_landmarks_);
            return -1;
        }


        if (type == "obstacle" && countLandmarksOfType("obstacle") >= max_obstacle_landmarks_)
        {
            ROS_INFO_THROTTLE(5.0,
                              "[LANDMARKS] Ignoro obstacle nou: ja he arribat al maxim de %d obstacles.",
                              max_obstacle_landmarks_);
            return -1;
        }


        if (type == "wall" && countLandmarksOfType("wall") >= max_wall_landmarks_)
        {
            ROS_INFO_THROTTLE(5.0,
                              "[LANDMARKS] Ignoro paret nova: ja he arribat al maxim de %d walls.",
                              max_wall_landmarks_);
            return -1;
        }


        if (!allow_near_duplicate)
        {
            for (size_t i = 0; i < landmarks_.size(); ++i)
            {
                if (landmarks_[i].type != type) {
                    continue;
                }


                double dx = x - landmarks_[i].pose.pose.position.x;
                double dy = y - landmarks_[i].pose.pose.position.y;


                if (std::sqrt(dx * dx + dy * dy) < feature_landmark_min_distance_)
                {
                    ROS_WARN("[LANDMARKS] He tornat a veure landmark #%zu (%s) a x=%.2f y=%.2f. Possible associacio per tancar el bucle.",
                             i,
                             landmarks_[i].type.c_str(),
                             landmarks_[i].pose.pose.position.x,
                             landmarks_[i].pose.pose.position.y);
                    publishState("landmarks: landmark conegut revisitat");
                    return static_cast<int>(i);
                }
            }
        }


        Landmark landmark;
        landmark.type = type;
        landmark.score = score;
        landmark.pose.header.stamp = ros::Time::now();
        landmark.pose.header.frame_id = "map";
        landmark.pose.pose.position.x = x;
        landmark.pose.pose.position.y = y;
        landmark.pose.pose.position.z = 0.0;
        landmark.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);


        landmarks_.push_back(landmark);


        size_t index = landmarks_.size() - 1;


        ROS_WARN("[LANDMARKS] Acabo de veure %s -> landmark #%zu. x=%.2f y=%.2f yaw=%.2f score=%.2f",
                 type.c_str(),
                 index,
                 x, y, yaw, score);


        std::stringstream ss;
        ss << "landmarks: nou " << type << " -> landmark #" << index;
        publishState(ss.str());


        return static_cast<int>(index);
    }


    bool captureStartPoseIfNeeded()
    {
        if (start_pose_set_) {
            return true;
        }


        double x, y, yaw;


        if (!getRobotPose(x, y, yaw)) {
            return false;
        }


        start_pose_set_ = true;
        start_x_ = x;
        start_y_ = y;
        start_yaw_ = yaw;
        last_landmark_x_ = x;
        last_landmark_y_ = y;
        last_landmark_time_ = ros::Time::now();


        int idx = addLandmark("start", x, y, yaw, 1.0, true);


        ROS_WARN("[LANDMARKS] Punt inicial capturat com landmark #%d: x=%.2f y=%.2f yaw=%.2f",
                 idx, x, y, yaw);
        publishState("landmarks: punt inicial capturat");
        return true;
    }


    void recordLandmarkIfNeeded()
    {
        if (!start_pose_set_) {
            return;
        }


        double x, y, yaw;


        if (!getRobotPose(x, y, yaw)) {
            return;
        }


        ros::Time now = ros::Time::now();
        double elapsed = (now - last_landmark_time_).toSec();
        double moved = std::sqrt((x - last_landmark_x_) * (x - last_landmark_x_) +
                                 (y - last_landmark_y_) * (y - last_landmark_y_));


        if (!landmarks_.empty() && moved < landmark_min_distance_ && elapsed < landmark_min_time_) {
            return;
        }


        int idx = addLandmark("path", x, y, yaw, moved, true);
        last_landmark_x_ = x;
        last_landmark_y_ = y;
        last_landmark_time_ = now;


        ROS_INFO("[LANDMARKS] Recorregut actualitzat -> landmark #%d. x=%.2f y=%.2f yaw=%.2f",
                 idx, x, y, yaw);
    }


    void recordCurrentPoseAsLandmark()
    {
        double x, y, yaw;


        if (!getRobotPose(x, y, yaw)) {
            return;
        }


        int idx = addLandmark("loop_closure", x, y, yaw, 1.0, true);


        ROS_WARN("[LANDMARKS] He tornat al punt inicial -> landmark #%d de tancament del bucle. x=%.2f y=%.2f yaw=%.2f",
                 idx, x, y, yaw);
    }


    bool scanPointToMap(double robot_x,
                        double robot_y,
                        double robot_yaw,
                        double scan_angle,
                        double range,
                        double& map_x,
                        double& map_y)
    {
        if (!std::isfinite(range) || range <= 0.0) {
            return false;
        }


        double global_angle = robot_yaw + scan_angle;
        map_x = robot_x + range * std::cos(global_angle);
        map_y = robot_y + range * std::sin(global_angle);
        return true;
    }


    bool fitWallSegment(const std::vector<ScanPoint>& points,
                        int begin,
                        int end,
                        WallSegment& wall)
    {
        int n = end - begin;


        if (n < wall_min_points_) {
            return false;
        }


        double mean_x = 0.0;
        double mean_y = 0.0;


        for (int i = begin; i < end; ++i)
        {
            mean_x += points[i].x;
            mean_y += points[i].y;
        }


        mean_x /= n;
        mean_y /= n;


        double cov_xx = 0.0;
        double cov_xy = 0.0;
        double cov_yy = 0.0;


        for (int i = begin; i < end; ++i)
        {
            double dx = points[i].x - mean_x;
            double dy = points[i].y - mean_y;


            cov_xx += dx * dx;
            cov_xy += dx * dy;
            cov_yy += dy * dy;
        }


        cov_xx /= n;
        cov_xy /= n;
        cov_yy /= n;


        double yaw = 0.5 * std::atan2(2.0 * cov_xy, cov_xx - cov_yy);
        double dir_x = std::cos(yaw);
        double dir_y = std::sin(yaw);
        double normal_x = -dir_y;
        double normal_y = dir_x;


        double min_projection = std::numeric_limits<double>::max();
        double max_projection = -std::numeric_limits<double>::max();
        double error_sum = 0.0;


        for (int i = begin; i < end; ++i)
        {
            double dx = points[i].x - mean_x;
            double dy = points[i].y - mean_y;
            double projection = dx * dir_x + dy * dir_y;
            double residual = dx * normal_x + dy * normal_y;


            min_projection = std::min(min_projection, projection);
            max_projection = std::max(max_projection, projection);
            error_sum += residual * residual;
        }


        double length = max_projection - min_projection;
        double error = std::sqrt(error_sum / n);


        if (length < wall_min_length_ || error > wall_max_fit_error_) {
            return false;
        }


        wall.x = mean_x;
        wall.y = mean_y;
        wall.yaw = yaw;
        wall.length = length;
        wall.error = error;
        wall.start_index = points[begin].index;
        wall.end_index = points[end - 1].index;
        return true;
    }


    void addWallCornerIfNeeded(const WallSegment& previous,
                               const WallSegment& current,
                               const std::vector<ScanPoint>& points)
    {
        double angle_diff = std::fabs(normalizeAngle(current.yaw - previous.yaw));


        if (angle_diff > M_PI / 2.0) {
            angle_diff = M_PI - angle_diff;
        }


        if (angle_diff < wall_corner_angle_threshold_) {
            return;
        }


        int best_index = -1;
        double best_score = std::numeric_limits<double>::max();


        for (size_t i = 0; i < points.size(); ++i)
        {
            double d = std::abs(points[i].index - previous.end_index) +
                       std::abs(points[i].index - current.start_index);


            if (d < best_score)
            {
                best_score = d;
                best_index = static_cast<int>(i);
            }
        }


        if (best_index < 0) {
            return;
        }


        addLandmark("corner",
                    points[best_index].x,
                    points[best_index].y,
                    current.yaw,
                    angle_diff,
                    false);
    }


    void addObstacleClusterIfNeeded(const std::vector<ScanPoint>& points,
                                    int begin,
                                    int end)
    {
        int count = 0;
        double mean_x = 0.0;
        double mean_y = 0.0;
        double mean_range = 0.0;


        for (int i = begin; i < end; ++i)
        {
            if (points[i].range > obstacle_landmark_max_range_) {
                continue;
            }


            mean_x += points[i].x;
            mean_y += points[i].y;
            mean_range += points[i].range;
            count++;
        }


        if (count < std::max(4, wall_min_points_ / 2)) {
            return;
        }


        mean_x /= count;
        mean_y /= count;
        mean_range /= count;


        addLandmark("obstacle", mean_x, mean_y, 0.0, mean_range, false);
    }


    void recordFeatureLandmarks(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        if (!msg || msg->ranges.empty()) {
            return;
        }


        ros::Time now = ros::Time::now();


        if (!last_feature_landmark_time_.isZero() &&
            (now - last_feature_landmark_time_).toSec() < feature_landmark_min_time_) {
            return;
        }


        double robot_x, robot_y, robot_yaw;


        if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
            return;
        }


        last_feature_landmark_time_ = now;


        std::vector<ScanPoint> wall_points;
        std::vector<WallSegment> walls;


        for (int i = 0; i < static_cast<int>(msg->ranges.size()); ++i)
        {
            float range = msg->ranges[i];


            if (!std::isfinite(range) ||
                range <= msg->range_min ||
                range > wall_landmark_max_range_) {
                continue;
            }


            double angle = msg->angle_min + i * msg->angle_increment;
            double x, y;


            if (scanPointToMap(robot_x, robot_y, robot_yaw, angle, range, x, y))
            {
                ScanPoint point;
                point.x = x;
                point.y = y;
                point.angle = angle;
                point.range = range;
                point.index = i;
                wall_points.push_back(point);
            }
        }


        int begin = 0;


        while (begin < static_cast<int>(wall_points.size()))
        {
            int end = begin + 1;


            while (end < static_cast<int>(wall_points.size()))
            {
                bool contiguous = wall_points[end].index == wall_points[end - 1].index + 1;
                double range_jump = std::fabs(wall_points[end].range - wall_points[end - 1].range);


                if (!contiguous || range_jump > wall_split_jump_) {
                    break;
                }


                end++;
            }


            WallSegment wall;


            if (fitWallSegment(wall_points, begin, end, wall))
            {
                walls.push_back(wall);
                addLandmark("wall", wall.x, wall.y, wall.yaw, wall.length, false);
            }
            else
            {
                addObstacleClusterIfNeeded(wall_points, begin, end);
            }


            begin = end;
        }


        for (size_t i = 1; i < walls.size(); ++i)
        {
            addWallCornerIfNeeded(walls[i - 1], walls[i], wall_points);
        }
    }


    bool saveLandmarksIfNeeded()
    {
        if (landmarks_.empty()) {
            ROS_WARN("[LANDMARKS] No hi ha landmarks per guardar.");
            return true;
        }


        std::string dir = parentDirectory(landmarks_output_file_);


        if (!dir.empty())
        {
            std::string mkdir_cmd = "mkdir -p " + shellQuote(dir);
            int mkdir_result = std::system(mkdir_cmd.c_str());


            if (mkdir_result != 0) {
                ROS_WARN("[LANDMARKS] No s'ha pogut crear el directori: %s", dir.c_str());
            }
        }


        std::ofstream out(landmarks_output_file_.c_str());


        if (!out.is_open()) {
            ROS_WARN("[LANDMARKS] No puc obrir el fitxer: %s", landmarks_output_file_.c_str());
            return false;
        }


        out << std::fixed << std::setprecision(3);
        out << "# idx type stamp_sec x y yaw score\n";


        for (size_t i = 0; i < landmarks_.size(); ++i)
        {
            const geometry_msgs::PoseStamped& p = landmarks_[i].pose;
            double yaw = tf::getYaw(p.pose.orientation);
            out << i << " "
                << landmarks_[i].type << " "
                << p.header.stamp.toSec() << " "
                << p.pose.position.x << " "
                << p.pose.position.y << " "
                << yaw << " "
                << landmarks_[i].score << "\n";
        }


        out.close();


        ROS_WARN("[LANDMARKS] Guardats %zu landmarks a %s",
                 landmarks_.size(),
                 landmarks_output_file_.c_str());
        publishState("landmarks: desats correctament");
        return true;
    }


    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        map_ = msg;


        updateMapStats(msg);


        double stable_for = (ros::Time::now() - last_map_growth_time_).toSec();


        ROS_INFO_THROTTLE(3.0,
                          "[SLAM] Mapa rebut: %u x %u cel.les, res %.3f m/cel.la | coneguda %.2f m2 | lliure %.2f m2 | ocupada %.2f m2 | estable %.1f s",
                          map_->info.width,
                          map_->info.height,
                          map_->info.resolution,
                          known_area_,
                          free_area_,
                          occupied_area_,
                          stable_for);
    }


    void updateMapStats(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        if (!msg) {
            return;
        }


        double res = msg->info.resolution;
        double cell_area = res * res;


        long long known_count = 0;
        long long free_count = 0;
        long long occupied_count = 0;


        for (size_t i = 0; i < msg->data.size(); ++i)
        {
            int value = msg->data[i];


            if (value >= 0)
            {
                known_count++;


                if (value <= occupied_threshold_) {
                    free_count++;
                } else {
                    occupied_count++;
                }
            }
        }


        known_area_ = known_count * cell_area;
        free_area_ = free_count * cell_area;
        occupied_area_ = occupied_count * cell_area;


        if (known_area_ > best_known_area_ + map_growth_epsilon_)
        {
            best_known_area_ = known_area_;
            last_map_growth_time_ = ros::Time::now();


            ROS_WARN("[SLAM] El mapa ha crescut. Area coneguda actual: %.2f m2", known_area_);
        }
    }


    void publishState(const std::string& text)
    {
        std_msgs::String msg;
        msg.data = text;
        state_pub_.publish(msg);
    }


    void publishPhase(const std::string& phase)
    {
        std_msgs::String msg;
        msg.data = phase;
        current_phase_pub_.publish(msg);
    }


    void publishNavigationDone(bool done)
    {
        std_msgs::Bool msg;
        msg.data = done;
        navigation_done_pub_.publish(msg);
    }


    void publishDrawingCenter(double x, double y)
    {
        geometry_msgs::PoseStamped msg;


        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";


        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.position.z = 0.0;


        msg.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);


        drawing_center_pub_.publish(msg);


        ROS_WARN("[AREA] Centre de dibuix publicat a /boticelli/drawing_center: x=%.2f y=%.2f",
                 x, y);
    }


    void publishFreeAreaInfo(double area, double x, double y)
    {
        std_msgs::String msg;
        std::stringstream ss;


        ss << "Area lliure trobada. "
           << "area_m2=" << area
           << " center_x=" << x
           << " center_y=" << y
           << " drawing_width=" << drawing_width_
           << " drawing_height=" << drawing_height_
           << " safety_margin=" << safety_margin_;


        msg.data = ss.str();


        free_area_info_pub_.publish(msg);


        ROS_WARN("[AREA] %s", msg.data.c_str());
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
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }


        while (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }


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
        while (angle < 0.0) {
            angle += 2.0 * M_PI;
        }


        while (angle >= 2.0 * M_PI) {
            angle -= 2.0 * M_PI;
        }


        while (start < 0.0) {
            start += 2.0 * M_PI;
        }


        while (start >= 2.0 * M_PI) {
            start -= 2.0 * M_PI;
        }


        while (end < 0.0) {
            end += 2.0 * M_PI;
        }


        while (end >= 2.0 * M_PI) {
            end -= 2.0 * M_PI;
        }


        if (start <= end) {
            return angle >= start && angle <= end;
        }


        return angle >= start || angle <= end;
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


    bool mapToWorld(int mx, int my, double& wx, double& wy)
    {
        if (!map_) {
            return false;
        }


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
                ROS_WARN_THROTTLE(1.0,
                                  "[TF] No puc obtenir TF map -> base_footprint/base_link.");
                return false;
            }
        }


        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
        yaw = tf::getYaw(transform.getRotation());


        return true;
    }


    std::string shellQuote(const std::string& input)
    {
        std::string output = "'";


        for (size_t i = 0; i < input.size(); ++i)
        {
            if (input[i] == '\'') {
                output += "'\\''";
            } else {
                output += input[i];
            }
        }


        output += "'";
        return output;
    }


    std::string parentDirectory(const std::string& path)
    {
        size_t pos = path.find_last_of('/');


        if (pos == std::string::npos) {
            return "";
        }


        if (pos == 0) {
            return "/";
        }


        return path.substr(0, pos);
    }


    bool saveMapIfNeeded()
    {
        if (!auto_save_map_) {
            ROS_WARN("[MAP_SAVE] Auto save desactivat. No es guarda el mapa automaticament.");
            return true;
        }


        if (map_saved_) {
            ROS_WARN("[MAP_SAVE] El mapa ja estava guardat.");
            return true;
        }


        if (!map_) {
            ROS_WARN("[MAP_SAVE] No puc guardar mapa perque encara no hi ha /map.");
            return false;
        }


        stopRobot();


        ROS_WARN("=================================================");
        ROS_WARN("[MAP_SAVE] Guardant mapa amb map_saver...");
        ROS_WARN("[MAP_SAVE] Sortida: %s.yaml i %s.pgm",
                 map_output_file_.c_str(),
                 map_output_file_.c_str());
        ROS_WARN("=================================================");


        std::string dir = parentDirectory(map_output_file_);


        if (!dir.empty())
        {
            std::string mkdir_cmd = "mkdir -p " + shellQuote(dir);
            int mkdir_result = std::system(mkdir_cmd.c_str());


            if (mkdir_result != 0) {
                ROS_WARN("[MAP_SAVE] No s'ha pogut crear el directori: %s", dir.c_str());
            }
        }


        std::string cmd = "rosrun map_server map_saver -f " + shellQuote(map_output_file_);


        int result = std::system(cmd.c_str());


        if (result == 0)
        {
            map_saved_ = true;
            ROS_WARN("[MAP_SAVE] Mapa guardat correctament.");
            publishState("map_save: mapa guardat correctament");
            publishPhase("MAP_SAVED");
            return true;
        }


        ROS_WARN("[MAP_SAVE] Error guardant el mapa. Codi retornat: %d", result);
        publishState("map_save: error guardant mapa");
        return false;
    }


    bool findBestFreeSpace(double& best_x, double& best_y)
    {
        if (!map_) {
            ROS_WARN("[AREA] Encara no hi ha mapa disponible.");
            return false;
        }


        int width = static_cast<int>(map_->info.width);
        int height = static_cast<int>(map_->info.height);
        double res = map_->info.resolution;


        if (width <= 0 || height <= 0 || res <= 0.0) {
            ROS_WARN("[AREA] Mapa invalid: width=%d height=%d res=%.3f", width, height, res);
            return false;
        }


        double required_width = drawing_width_ + 2.0 * safety_margin_;
        double required_height = drawing_height_ + 2.0 * safety_margin_;


        int half_w_cells = static_cast<int>(std::ceil((required_width / 2.0) / res));
        int half_h_cells = static_cast<int>(std::ceil((required_height / 2.0) / res));


        ROS_WARN("=================================================");
        ROS_WARN("[AREA] Analitzant mapa per trobar una zona lliure...");
        ROS_WARN("[AREA] Mapa: %d x %d cel.les, resolucio %.3f m/cel.la", width, height, res);
        ROS_WARN("[AREA] Area coneguda: %.2f m2 | area lliure: %.2f m2 | area ocupada: %.2f m2",
                 known_area_, free_area_, occupied_area_);
        ROS_WARN("[AREA] Rectangle requerit: %.2f x %.2f m", required_width, required_height);
        ROS_WARN("=================================================");


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


                        double d = std::pow(cx - centroid_x, 2.0) +
                                   std::pow(cy - centroid_y, 2.0);


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
            ROS_WARN("[AREA] No s'ha trobat cap zona on hi capiga el dibuix.");
            return false;
        }


        mapToWorld(best_mx, best_my, best_x, best_y);


        double valid_area = best_count * res * res;


        ROS_WARN("=================================================");
        ROS_WARN("[AREA] Millor zona trobada.");
        ROS_WARN("[AREA] Area aproximada de centres valids: %.2f m2", valid_area);
        ROS_WARN("[AREA] Centre seleccionat: x = %.2f, y = %.2f", best_x, best_y);
        ROS_WARN("=================================================");


        publishDrawingCenter(best_x, best_y);
        publishFreeAreaInfo(valid_area, best_x, best_y);


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
    }


    void obstacleAvoidanceMove(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        geometry_msgs::Twist vel;


        if (msg->ranges.empty()) {
            stopRobot();
            return;
        }


        float front = minRangeDeg(msg, 330.0, 30.0);
        float front_left = minRangeDeg(msg, 20.0, 80.0);
        float front_right = minRangeDeg(msg, 280.0, 340.0);
        float left = minRangeDeg(msg, 70.0, 120.0);
        float right = minRangeDeg(msg, 240.0, 290.0);


        const float SAFE_DISTANCE = 0.50;
        const float VERY_CLOSE = 0.30;
        const float SIDE_DISTANCE = 0.35;


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


            vel.linear.x = -0.025;
            vel.angular.z = turn_direction_ * 0.35;


            ROS_WARN_THROTTLE(1.0,
                              "[EXPLORACIO] Massa a prop. Escapant. front=%.2f front_left=%.2f front_right=%.2f left=%.2f right=%.2f dir=%d",
                              front, front_left, front_right, left, right, turn_direction_);


            publishState("exploracio: massa a prop, reculo i giro");
        }
        else if (obstacle_ahead)
        {
            chooseTurnDirection(left_clear, right_clear);


            vel.linear.x = 0.025;
            vel.angular.z = turn_direction_ * 0.30;


            ROS_INFO_THROTTLE(1.0,
                              "[EXPLORACIO] Obstacle davant, avanco lent i giro. front=%.2f front_left=%.2f front_right=%.2f left=%.2f right=%.2f dir=%d",
                              front, front_left, front_right, left, right, turn_direction_);


            publishState("exploracio: obstacle davant, avanco lent i giro");
        }
        else
        {
            vel.linear.x = 0.055;
            vel.angular.z = 0.0;


            if (left < SIDE_DISTANCE && right < SIDE_DISTANCE)
            {
                if (left < right) {
                    vel.angular.z = -0.15;
                } else {
                    vel.angular.z = 0.15;
                }


                publishState("exploracio: passadis estret, centrant-se");
            }
            else if (left < SIDE_DISTANCE)
            {
                vel.angular.z = -0.15;
                publishState("exploracio: massa a prop esquerra");
            }
            else if (right < SIDE_DISTANCE)
            {
                vel.angular.z = 0.15;
                publishState("exploracio: massa a prop dreta");
            }
            else
            {
                publishState("exploracio: avancant lliure");
            }


            ROS_INFO_THROTTLE(1.0,
                              "[EXPLORACIO] Avancant. front=%.2f left=%.2f right=%.2f",
                              front, left, right);
        }


        vel_pub_.publish(vel);
    }


    bool goToGoalMove(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        double robot_x, robot_y, robot_yaw;


        if (!getRobotPose(robot_x, robot_y, robot_yaw))
        {
            publishState("go_to_goal: no puc obtenir pose, paro");
            stopRobot();
            return false;
        }


        double dx = goal_x_ - robot_x;
        double dy = goal_y_ - robot_y;


        double distance = std::sqrt(dx * dx + dy * dy);


        if (mode_ == RETURNING_TO_START)
        {
            ROS_INFO_THROTTLE(1.0,
                              "[RETURN] Navegant cap al punt inicial. dist=%.2f goal=(%.2f, %.2f) robot=(%.2f, %.2f)",
                              distance, goal_x_, goal_y_, robot_x, robot_y);
            publishState("return_home: navegant cap al punt inicial");
        }


        if (distance < goal_tolerance_)
        {
            stopRobot();


            if (mode_ == RETURNING_TO_START)
            {
                ROS_WARN("=================================================");
                ROS_WARN("[RETURN] He tornat a veure el punt inicial. Tancant el bucle.");
                ROS_WARN("[RETURN] Posicio actual: x = %.2f, y = %.2f", robot_x, robot_y);
                ROS_WARN("[RETURN] Tanco el bucle, guardo landmarks i continuo amb el mapa.");
                ROS_WARN("=================================================");
                publishState("return_home: punt inicial revisitat, tancant bucle");
                return true;
            }


            found_space_ = true;
            mode_ = FINISHED;


            std::stringstream ss;
            ss << "FINISHED: robot al centre de la millor zona. x="
               << robot_x << " y=" << robot_y;


            publishState(ss.str());
            publishPhase("READY_FOR_VISION");
            publishNavigationDone(true);


            ROS_WARN("=================================================");
            ROS_WARN("[FINAL] Robot arribat al centre de la millor zona lliure.");
            ROS_WARN("[FINAL] Posicio final: x = %.2f, y = %.2f", robot_x, robot_y);
            ROS_WARN("[FINAL] Fase 1 acabada: mapa guardat i zona optima assolida.");
            ROS_WARN("=================================================");
            return true;
        }


        float front = minRangeDeg(msg, 330.0, 30.0);
        float front_left = minRangeDeg(msg, 20.0, 80.0);
        float front_right = minRangeDeg(msg, 280.0, 340.0);


        if (front < 0.40 || front_left < 0.32 || front_right < 0.32)
        {
            publishState("go_to_goal: obstacle al cami, evitant");
            obstacleAvoidanceMove(msg);
            return false;
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


            if (angular_cmd > max_angular_speed_) {
                angular_cmd = max_angular_speed_;
            }


            if (angular_cmd < -max_angular_speed_) {
                angular_cmd = -max_angular_speed_;
            }


            vel.angular.z = angular_cmd;


            publishState("go_to_goal: avancant cap al centre");
        }


        ROS_INFO_THROTTLE(1.0,
                          "[NAVEGACIO] Anant al centre. dist=%.2f angle_error=%.2f goal=(%.2f, %.2f)",
                          distance, angle_error, goal_x_, goal_y_);


        vel_pub_.publish(vel);
        return false;
    }


    bool shouldFinishExploration(double elapsed, std::string& reason)
    {
        if (max_explore_time_ > 0.0 && elapsed >= max_explore_time_)
        {
            reason = "temps maxim d'exploracio assolit";
            return true;
        }


        if (elapsed < min_explore_time_) {
            reason = "encara no s'ha superat min_explore_time";
            return false;
        }


        if (!map_) {
            reason = "encara no hi ha mapa";
            return false;
        }


        double stable_for = (ros::Time::now() - last_map_growth_time_).toSec();


        if (stable_for >= map_stable_time_)
        {
            reason = "mapa estable durant prou temps";
            return true;
        }


        std::stringstream ss;
        ss << "mapa encara creix o no porta prou temps estable. stable_for="
           << stable_for << " s";


        reason = ss.str();
        return false;
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
            if (!start_pose_set_)
            {
                captureStartPoseIfNeeded();
            }


            recordLandmarkIfNeeded();
            recordFeatureLandmarks(msg);


            double stable_for = (ros::Time::now() - last_map_growth_time_).toSec();


            ROS_INFO_THROTTLE(2.0,
                              "[SLAM] Explorant. temps=%.1f s | min=%.1f | max=%.1f | coneguda=%.2f m2 | lliure=%.2f m2 | estable=%.1f/%.1f s",
                              elapsed,
                              min_explore_time_,
                              max_explore_time_,
                              known_area_,
                              free_area_,
                              stable_for,
                              map_stable_time_);


            std::string reason;
            bool finish_exploration = shouldFinishExploration(elapsed, reason);


            if (!finish_exploration)
            {
                ROS_INFO_THROTTLE(4.0, "[SLAM] Continuo explorant: %s", reason.c_str());
                obstacleAvoidanceMove(msg);
                return;
            }


            stopRobot();


            ROS_WARN("=================================================");
            ROS_WARN("[SLAM] Exploracio acabada.");
            ROS_WARN("[SLAM] Motiu: %s", reason.c_str());
            ROS_WARN("[SLAM] Area coneguda final: %.2f m2", known_area_);
            ROS_WARN("[SLAM] Area lliure final: %.2f m2", free_area_);
            ROS_WARN("[SLAM] Ara guardo el mapa abans de calcular la zona optima.");
            ROS_WARN("=================================================");


            publishState("slam: exploracio acabada, retornant al punt inicial");


            if (!start_pose_set_ && !captureStartPoseIfNeeded())
            {
                ROS_WARN("[RETURN] No he pogut capturar el punt inicial. Continuo amb la posicio actual.");
            }


            if (start_pose_set_)
            {
                goal_x_ = start_x_;
                goal_y_ = start_y_;
                goal_available_ = true;
                mode_ = RETURNING_TO_START;


                ROS_WARN("=================================================");
                ROS_WARN("[RETURN] Navegant cap al punt inicial per tancar el bucle.");
                ROS_WARN("[RETURN] Objectiu: x = %.2f, y = %.2f, yaw = %.2f",
                         start_x_, start_y_, start_yaw_);
                ROS_WARN("=================================================");


                publishPhase("RETURNING_TO_START");
                publishNavigationDone(false);
                publishState("return_home: tornant al punt inicial");
                return;
            }


            publishState("return_home: no hi ha punt inicial, passo a guardar mapa");
            saveMapIfNeeded();
            mode_ = COMPUTING_GOAL;
            publishPhase("COMPUTING_FREE_AREA");
        }


        if (mode_ == RETURNING_TO_START)
        {
            bool arrived = goToGoalMove(msg);


            if (!arrived) {
                return;
            }


            recordCurrentPoseAsLandmark();
            saveLandmarksIfNeeded();
            saveMapIfNeeded();


            mode_ = COMPUTING_GOAL;


            ROS_WARN("=================================================");
            ROS_WARN("[AREA] Ara busco la millor zona lliure del mapa.");
            ROS_WARN("=================================================");


            publishState("computing_goal: buscant millor zona lliure");
            publishPhase("COMPUTING_FREE_AREA");
        }


        if (mode_ == COMPUTING_GOAL)
        {
            bool ok = findBestFreeSpace(goal_x_, goal_y_);


            if (!ok)
            {
                ROS_WARN("[AREA] No hi ha zona bona encara. Exploro una mica mes.");
                publishState("computing_goal: no hi ha zona, continuo explorant");
                publishNavigationDone(false);
                publishPhase("SLAM_EXPLORING");


                mode_ = EXPLORING;
                start_time_ = ros::Time::now();
                last_map_growth_time_ = ros::Time::now();
                best_known_area_ = known_area_;
                return;
            }


            goal_available_ = true;
            mode_ = GOING_TO_GOAL;
            publishPhase("GOING_TO_GOAL");


            std::stringstream ss;
            ss << "GOAL_FOUND: x=" << goal_x_ << " y=" << goal_y_;
            publishState(ss.str());


            ROS_WARN("=================================================");
            ROS_WARN("[NAVEGACIO] Objectiu seleccionat.");
            ROS_WARN("[NAVEGACIO] El robot anira al centre de la millor zona lliure.");
            ROS_WARN("[NAVEGACIO] Goal: x = %.2f, y = %.2f", goal_x_, goal_y_);
            ROS_WARN("=================================================");


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



