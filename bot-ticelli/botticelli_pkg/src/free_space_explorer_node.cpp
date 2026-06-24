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

/*
 * Node d'exploracio durant el SLAM.
 *
 * Gmapping s'encarrega de l'estimacio probabilistica del mapa a partir de laser i odometria.
 * Aquest node no substitueix gmapping: simplement mou el robot de forma segura mentre
 * gmapping construeix /map, observa el mapa d'ocupacio resultant i tria una zona lliure
 * prou gran per fer-hi el dibuix.
 *
 * Funcions principals:
 *  - Evitar obstacles amb el LaserScan mentre s'explora.
 *  - Guardar el mapa amb map_saver quan acaba l'exploracio.
 *  - Tornar prop del punt inicial per mantenir una pose coherent.
 *  - Buscar en el mapa una regio lliure suficient per al dibuix.
 *  - Publicar READY_FOR_VISION quan el sistema ja pot passar a visio i dibuix.
 *
 * Tambe guarda "features" geometriques simples del laser. No son necessaries per gmapping,
 * pero serveixen com a registre de punts/parets/cantonades observades durant l'exploracio.
 */
class FindFreeSpaceExplorer
{
private:
    // Feature geometrica registrada durant l'exploracio. Es guarda amb tipus, pose i score.
    struct Feature
    {
        std::string type;
        geometry_msgs::PoseStamped pose;
        double score;
    };

    // Punt del laser convertit a coordenades del mapa/robot per poder analitzar parets
    // i obstacles com geometria 2D.
    struct ScanPoint
    {
        double x;
        double y;
        double angle;
        double range;
        int index;
    };

    // Segment de paret aproximat a partir d'un grup de punts de laser.
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

    // Maquina d'estats de l'explorador. Cada estat correspon a una etapa clara del flux.
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

    // Dimensions minimes de la zona lliure que volem trobar per fer el dibuix.
    double drawing_width_;
    double drawing_height_;
    double safety_margin_;

    // Parametres temporals i de creixement del mapa. El final real de l'exploracio
    // el marca max_explore_time; map_stable_time queda com a informacio de diagnostic.
    double max_explore_time_;
    double map_stable_time_;
    double map_growth_epsilon_;

    // Parametres de moviment i tolerancia per arribar als objectius.
    double goal_tolerance_;
    double max_linear_speed_;
    double max_angular_speed_;

    // Llindar per interpretar cel·les del mapa d'ocupacio com a obstacles.
    int occupied_threshold_;

    // Fitxers de sortida: mapa guardat i registre de features.
    bool auto_save_map_;
    bool map_saved_;
    std::string map_output_file_;
    std::string features_output_file_;

    bool found_space_;
    bool goal_available_;

    // Objectiu actual de navegacio dins el mapa.
    double goal_x_;
    double goal_y_;

    // Pose inicial del robot. Serveix per tornar aproximadament al punt de sortida
    // abans de calcular la zona final de dibuix.
    bool start_pose_set_;
    double start_x_;
    double start_y_;
    double start_yaw_;

    // Parametres que limiten quantes features es guarden i cada quant es poden repetir.
    double feature_min_distance_;
    double feature_min_time_;
    double scan_feature_min_distance_;
    double scan_feature_min_time_;
    double obstacle_feature_max_range_;
    double corner_feature_max_range_;
    double corner_jump_threshold_;
    double wall_feature_max_range_;
    double wall_split_jump_;
    double wall_min_length_;
    double wall_max_fit_error_;
    double wall_corner_angle_threshold_;
    int wall_min_points_;
    int max_corner_features_;
    int max_obstacle_features_;
    int max_wall_features_;
    double last_feature_x_;
    double last_feature_y_;
    ros::Time last_feature_time_;
    ros::Time last_scan_feature_time_;

    std::vector<Feature> features_;

    // Direccio de gir preferida per evitar canvis bruscos esquerra/dreta.
    int turn_direction_;

    ros::Time start_time_;
    ros::Time last_map_growth_time_;

    Mode mode_;

    // Estadistiques del mapa d'ocupacio utilitzades per diagnostics i seleccio de zona.
    double known_area_;
    double free_area_;
    double occupied_area_;
    double best_known_area_;

    double last_distance_to_goal_ = 999.0;
    ros::Time last_progress_time_;
    int stall_counter_ = 0;
    bool first_laser_received_ = false;

    // Anti-oscillació: bloqueig de canvi de direcció de gir
    ros::Time last_turn_change_time_;
    const double MIN_TURN_LOCK_SEC = 1.5;

    // Detecció d'engancxament durant exploració
    ros::Time last_exploration_progress_time_;
    double last_exploration_x_ = 0.0;
    double last_exploration_y_ = 0.0;
    int exploration_stall_counter_ = 0;

    // ==========================================
    // PARÀMETRES DE GEOMETRIA DEL ROBOT (WAFFLE PI)
    // ==========================================
    const double ROBOT_RADIUS = 0.21;       // Radi físic del robot en metres (diagonal/2)
    const double ROBOT_SAFETY = 0.20;       // Marge d'evitació actiu augmentat per mapa gran (~0.41 total)
    const double CRITICAL_ZONE = ROBOT_RADIUS + 0.20; // ~0.41m. Col·lisió imminent
    const double WARNING_ZONE  = ROBOT_RADIUS + ROBOT_SAFETY + 0.15; // ~0.56m. Zona de frenada gradual

public:
    FindFreeSpaceExplorer()
        : nh_("~"),
          auto_save_map_(true),
          map_saved_(false),
          found_space_(false),
          goal_available_(false),
          first_laser_received_(false),
          goal_x_(0.0),
          goal_y_(0.0),
          start_pose_set_(false),
          start_x_(0.0),
          start_y_(0.0),
          start_yaw_(0.0),
          feature_min_distance_(0.50),
          feature_min_time_(5.0),
          scan_feature_min_distance_(0.45),
          scan_feature_min_time_(1.0),
          obstacle_feature_max_range_(1.20),
          corner_feature_max_range_(1.80),
          corner_jump_threshold_(1.00),
          wall_feature_max_range_(2.50),
          wall_split_jump_(0.20),
          wall_min_length_(0.45),
          wall_max_fit_error_(0.04),
          wall_corner_angle_threshold_(0.70),
          wall_min_points_(8),
          max_corner_features_(12),
          max_obstacle_features_(8),
          max_wall_features_(20),
          last_feature_x_(0.0),
          last_feature_y_(0.0),
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

        nh_.param("max_explore_time", max_explore_time_, 60.0);
        nh_.param("map_stable_time", map_stable_time_, 12.0);
        nh_.param("map_growth_epsilon", map_growth_epsilon_, 0.15);

        nh_.param("goal_tolerance", goal_tolerance_, 0.18);
        nh_.param("max_linear_speed", max_linear_speed_, 0.18);
        nh_.param("max_angular_speed", max_angular_speed_, 0.35);

        nh_.param("occupied_threshold", occupied_threshold_, 20);

        nh_.param("auto_save_map", auto_save_map_, true);
        nh_.param<std::string>("map_output_file", map_output_file_, "/tmp/mapa_boticelli_real");
        if (!nh_.getParam("features_output_file", features_output_file_)) {
            features_output_file_ = map_output_file_ + "_features.txt";
        }
        nh_.param("feature_min_distance", feature_min_distance_, 0.50);
        nh_.param("feature_min_time", feature_min_time_, 5.0);
        nh_.param("scan_feature_min_distance", scan_feature_min_distance_, 0.45);
        nh_.param("scan_feature_min_time", scan_feature_min_time_, 1.0);
        nh_.param("obstacle_feature_max_range", obstacle_feature_max_range_, 1.20);
        nh_.param("corner_feature_max_range", corner_feature_max_range_, 1.80);
        nh_.param("corner_jump_threshold", corner_jump_threshold_, 1.00);
        nh_.param("wall_feature_max_range", wall_feature_max_range_, 2.50);
        nh_.param("wall_split_jump", wall_split_jump_, 0.20);
        nh_.param("wall_min_length", wall_min_length_, 0.45);
        nh_.param("wall_max_fit_error", wall_max_fit_error_, 0.04);
        nh_.param("wall_corner_angle_threshold", wall_corner_angle_threshold_, 0.70);
        nh_.param("wall_min_points", wall_min_points_, 8);
        nh_.param("max_corner_features", max_corner_features_, 12);
        nh_.param("max_obstacle_features", max_obstacle_features_, 8);
        nh_.param("max_wall_features", max_wall_features_, 20);

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
        ROS_WARN("[BOT-TICELLI] Node free_space_explorer_node iniciat.");
        ROS_WARN("[MODEL] Configurat per a TurtleBot3 Waffle (Radi: %.2fm)", ROBOT_RADIUS);
        ROS_WARN("[PARAMS] Mida dibuix: %.2f x %.2f m", drawing_width_, drawing_height_);
        ROS_WARN("[PARAMS] Marge de seguretat: %.2f m", safety_margin_);
        ROS_WARN("[PARAMS] Temps maxim exploracio: %.2f s", max_explore_time_);
        ROS_WARN("[PARAMS] Temps mapa estable: %.2f s", map_stable_time_);
        ROS_WARN("[PARAMS] Creixement minim mapa: %.2f m2", map_growth_epsilon_);
        ROS_WARN("[PARAMS] Auto guardar mapa: %s", auto_save_map_ ? "true" : "false");
        ROS_WARN("[PARAMS] Fitxer mapa: %s", map_output_file_.c_str());
        ROS_WARN("[PARAMS] Fitxer features: %s", features_output_file_.c_str());
        ROS_WARN("[PARAMS] Separacio minima features: %.2f m | temps minim: %.2f s",
                 feature_min_distance_, feature_min_time_);
        ROS_WARN("[PARAMS] Separacio features mapa: %.2f m | temps minim features: %.2f s",
                 scan_feature_min_distance_, scan_feature_min_time_);
        ROS_WARN("[PARAMS] Obstacle max: %.2f m | cantonada max: %.2f m | salt cantonada: %.2f m",
                 obstacle_feature_max_range_, corner_feature_max_range_, corner_jump_threshold_);
        ROS_WARN("[PARAMS] Paret max: %.2f m | split: %.2f m | min length: %.2f m | fit error: %.2f m | min punts: %d",
                 wall_feature_max_range_, wall_split_jump_, wall_min_length_,
                 wall_max_fit_error_, wall_min_points_);
        ROS_WARN("[PARAMS] Max walls: %d | max corners: %d | max obstacles: %d",
                 max_wall_features_, max_corner_features_, max_obstacle_features_);
        ROS_WARN("[PARAMS] Llindar ocupacio: %d", occupied_threshold_);
        ROS_WARN("=================================================");
    }

    ~FindFreeSpaceExplorer()
    {
        stopRobot();
        ros::Duration(0.2).sleep();
    }

    // Comptador auxiliar per limitar el nombre de features de cada tipus.
    int countFeaturesOfType(const std::string& type)
    {
        int count = 0;
        for (size_t i = 0; i < features_.size(); ++i)
        {
            if (features_[i].type == type) {
                count++;
            }
        }
        return count;
    }

    // Afegeix una feature si no esta massa a prop d'una feature igual ja registrada.
    // Aixo evita omplir el fitxer amb moltes deteccions repetides del mateix obstacle.
    int addFeature(const std::string& type,
                    double x,
                    double y,
                    double yaw,
                    double score,
                    bool allow_near_duplicate)
    {
        if (type == "corner_feature" && countFeaturesOfType("corner_feature") >= max_corner_features_)
        {
            ROS_INFO_THROTTLE(5.0, "[FEATURES] Ignoro cantonada nova: ja he arribat al maxim de %d corners.", max_corner_features_);
            return -1;
        }

        if (type == "obstacle_cluster" && countFeaturesOfType("obstacle_cluster") >= max_obstacle_features_)
        {
            ROS_INFO_THROTTLE(5.0, "[FEATURES] Ignoro obstacle nou: ja he arribat al maxim de %d obstacles.", max_obstacle_features_);
            return -1;
        }

        if (type == "wall_feature" && countFeaturesOfType("wall_feature") >= max_wall_features_)
        {
            ROS_INFO_THROTTLE(5.0, "[FEATURES] Ignoro paret nova: ja he arribat al maxim de %d walls.", max_wall_features_);
            return -1;
        }

        if (!allow_near_duplicate)
        {
            for (size_t i = 0; i < features_.size(); ++i)
            {
                if (features_[i].type != type) {
                    continue;
                }

                double dx = x - features_[i].pose.pose.position.x;
                double dy = y - features_[i].pose.pose.position.y;

                if (std::sqrt(dx * dx + dy * dy) < scan_feature_min_distance_)
                {
                    ROS_WARN("[FEATURES] He tornat a veure feature #%zu (%s) a x=%.2f y=%.2f. Possible associacio amb una feature ja coneguda.",
                             i, features_[i].type.c_str(), features_[i].pose.pose.position.x, features_[i].pose.pose.position.y);
                    publishState("features: feature conegut revisitat");
                    return static_cast<int>(i);
                }
            }
        }

        Feature feature;
        feature.type = type;
        feature.score = score;
        feature.pose.header.stamp = ros::Time::now();
        feature.pose.header.frame_id = "map";
        feature.pose.pose.position.x = x;
        feature.pose.pose.position.y = y;
        feature.pose.pose.position.z = 0.0;
        feature.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        features_.push_back(feature);
        size_t index = features_.size() - 1;

        ROS_WARN("[FEATURES] Acabo de veure %s -> feature #%zu. x=%.2f y=%.2f yaw=%.2f score=%.2f",
                 type.c_str(), index, x, y, yaw, score);

        std::stringstream ss;
        ss << "features: nou " << type << " -> feature #" << index;
        publishState(ss.str());

        return static_cast<int>(index);
    }

    // Captura la pose inicial a partir de TF. Es fa una sola vegada i serveix com a
    // referencia per tornar a una zona coneguda abans de calcular el punt de dibuix.
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
        last_feature_x_ = x;
        last_feature_y_ = y;
        last_feature_time_ = ros::Time::now();

        int idx = addFeature("start_pose", x, y, yaw, 1.0, true);
        ROS_WARN("[FEATURES] Punt inicial capturat com feature #%d: x=%.2f y=%.2f yaw=%.2f", idx, x, y, yaw);
        publishState("features: punt inicial capturat");
        return true;
    }

    // Registra mostres de trajectoria del robot. No es fa a cada iteracio, sino amb
    // distancia i temps minims per obtenir un registre discret i llegible.
    void recordFeatureIfNeeded()
    {
        if (!start_pose_set_) {
            return;
        }

        double x, y, yaw;
        if (!getRobotPose(x, y, yaw)) {
            return;
        }

        ros::Time now = ros::Time::now();
        double elapsed = (now - last_feature_time_).toSec();
        double moved = std::sqrt((x - last_feature_x_) * (x - last_feature_x_) +
                                 (y - last_feature_y_) * (y - last_feature_y_));

        if (!features_.empty() && moved < feature_min_distance_ && elapsed < feature_min_time_) {
            return;
        }

        int idx = addFeature("trajectory_pose", x, y, yaw, moved, true);
        last_feature_x_ = x;
        last_feature_y_ = y;
        last_feature_time_ = now;

        ROS_INFO("[FEATURES] Recorregut actualitzat -> feature #%d. x=%.2f y=%.2f yaw=%.2f", idx, x, y, yaw);
    }

    // Guarda la pose actual com a retorn al punt inicial quan acaba l'exploracio.
    void recordCurrentPoseAsFeature()
    {
        double x, y, yaw;
        if (!getRobotPose(x, y, yaw)) {
            return;
        }

        int idx = addFeature("return_to_start", x, y, yaw, 1.0, true);
        ROS_WARN("[FEATURES] Retorn a l'inici registrat -> feature #%d de retorn a l'inici. x=%.2f y=%.2f yaw=%.2f", idx, x, y, yaw);
    }

    // Converteix una lectura del laser en un punt 2D del mapa. Aquesta transformacio
    // permet analitzar parets i obstacles en coordenades globals, no nomes relatives.
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

    // Ajusta un segment recte a un conjunt de punts del laser. Si l'error es petit,
    // el grup es considera una paret o tram recte de l'entorn.
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

    // Si dos segments de paret consecutius formen un canvi d'angle clar, es registra
    // una cantonada geometrica.
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

        addFeature("corner_feature", points[best_index].x, points[best_index].y, current.yaw, angle_diff, false);
    }

    // Detecta agrupacions compactes de punts que no encaixen com a paret i les desa
    // com a clusters d'obstacle.
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
            if (points[i].range > obstacle_feature_max_range_) {
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

        addFeature("obstacle_cluster", mean_x, mean_y, 0.0, mean_range, false);
    }

    // Analitza el LaserScan per extreure features geometriques simples: parets,
    // cantonades i obstacles compactes. Aquest registre es independent de gmapping.
    void recordScanFeatures(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        if (!msg || msg->ranges.empty()) {
            return;
        }

        ros::Time now = ros::Time::now();

        if (!last_scan_feature_time_.isZero() &&
            (now - last_scan_feature_time_).toSec() < scan_feature_min_time_) {
            return;
        }

        double robot_x, robot_y, robot_yaw;
        if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
            return;
        }

        last_scan_feature_time_ = now;

        std::vector<ScanPoint> wall_points;
        std::vector<WallSegment> walls;

        for (int i = 0; i < static_cast<int>(msg->ranges.size()); ++i)
        {
            float range = msg->ranges[i];

            if (!std::isfinite(range) || range <= msg->range_min || range > wall_feature_max_range_) {
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
                addFeature("wall_feature", wall.x, wall.y, wall.yaw, wall.length, false);
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

    // Desa el registre de features en un fitxer de text associat al mapa generat.
    bool saveFeaturesIfNeeded()
    {
        if (features_.empty()) {
            ROS_WARN("[FEATURES] No hi ha features per guardar.");
            return true;
        }

        std::string dir = parentDirectory(features_output_file_);
        if (!dir.empty())
        {
            std::string mkdir_cmd = "mkdir -p " + shellQuote(dir);
            int mkdir_result = std::system(mkdir_cmd.c_str());
            if (mkdir_result != 0) {
                ROS_WARN("[FEATURES] No s'ha pogut crear el directori: %s", dir.c_str());
            }
        }

        std::ofstream out(features_output_file_.c_str());
        if (!out.is_open()) {
            ROS_WARN("[FEATURES] No puc obrir el fitxer: %s", features_output_file_.c_str());
            return false;
        }

        out << std::fixed << std::setprecision(3);
        out << "# idx type stamp_sec x y yaw score\n";

        for (size_t i = 0; i < features_.size(); ++i)
        {
            const geometry_msgs::PoseStamped& p = features_[i].pose;
            double yaw = tf::getYaw(p.pose.orientation);
            out << i << " "
                << features_[i].type << " "
                << p.header.stamp.toSec() << " "
                << p.pose.position.x << " "
                << p.pose.position.y << " "
                << yaw << " "
                << features_[i].score << "\n";
        }
        out.close();

        ROS_WARN("[FEATURES] Guardats %zu features a %s", features_.size(), features_output_file_.c_str());
        publishState("features: desats correctament");
        return true;
    }

    // Callback del mapa publicat per gmapping. Cada mapa rebut actualitza estadistiques
    // de cel·les conegudes, lliures i ocupades.
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        map_ = msg;
        updateMapStats(msg);

        double stable_for = (ros::Time::now() - last_map_growth_time_).toSec();
        ROS_INFO_THROTTLE(3.0,
                          "[SLAM] Mapa rebut: %u x %u cel.les, res %.3f m/cel.la | coneguda %.2f m2 | lliure %.2f m2 | ocupada %.2f m2 | estable %.1f s",
                          map_->info.width, map_->info.height, map_->info.resolution, known_area_, free_area_, occupied_area_, stable_for);
    }

    // Calcula superficies conegudes/lliures/ocupades a partir de l'OccupancyGrid.
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

    // Topic de debug llegible per terminal/RViz: ajuda a saber en quina decisio esta el node.
    void publishState(const std::string& text)
    {
        std_msgs::String msg;
        msg.data = text;
        state_pub_.publish(msg);
    }

    // Fase de mes alt nivell que escolta la interfície. READY_FOR_VISION es el senyal
    // que permet passar del SLAM a la captura de la figura.
    void publishPhase(const std::string& phase)
    {
        std_msgs::String msg;
        msg.data = phase;
        current_phase_pub_.publish(msg);
    }

    // Indica si la navegacio interna de l'explorador ha acabat.
    void publishNavigationDone(bool done)
    {
        std_msgs::Bool msg;
        msg.data = done;
        navigation_done_pub_.publish(msg);
    }

    // Publica el centre de la zona on es dibuixara. Es latched perque el planner
    // o RViz el puguin veure encara que es connectin una mica tard.
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
        ROS_WARN("[AREA] Centre de dibuix publicat a /boticelli/drawing_center: x=%.2f y=%.2f", x, y);
    }

    // Publica informacio textual de la millor zona lliure trobada.
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

    // Ordre de seguretat: velocitat zero.
    void stopRobot()
    {
        geometry_msgs::Twist vel;
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
        vel_pub_.publish(vel);
    }

    // Normalitza angles a [-pi, pi] per fer comparacions de gir coherents.
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

    // Comprova si un angle cau dins un interval, tenint en compte el salt de -pi/pi.
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

    // Converteix coordenades de cel·la del mapa a metres en el frame map.
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

    // Obte la pose actual del robot usant TF map -> base_link.
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
                ROS_WARN_THROTTLE(1.0, "[TF] No puc obtenir TF map -> base_footprint/base_link.");
                return false;
            }
        }

        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
        yaw = tf::getYaw(transform.getRotation());
        return true;
    }

    // Protegeix rutes de fitxer quan es construeix la comanda map_saver.
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

    // Extreu la carpeta pare d'una ruta per crear-la abans de guardar fitxers.
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

    // Guarda el mapa de gmapping amb map_saver. Es fa una sola vegada per no
    // sobreescriure constantment el mateix fitxer.
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
        ROS_WARN("[MAP_SAVE] Sortida: %s.yaml i %s.pgm", map_output_file_.c_str(), map_output_file_.c_str());
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

    // Busca una regio lliure prou gran al mapa d'ocupacio. Es comproven cel·les al voltant
    // de cada candidat per garantir espai per al dibuix i marge de seguretat.
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
        ROS_WARN("[AREA] Area coneguda: %.2f m2 | area lliure: %.2f m2 | area ocupada: %.2f m2", known_area_, free_area_, occupied_area_);
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

        int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

        // --- Distància a obstacles (BFS des de cada cel·la ocupada/desconeguda) ---
        // Usem distància en cel·les; guardarem la distància mínima a paret per cada cel·la vàlida
        std::vector<int> dist_to_wall(width * height, std::numeric_limits<int>::max());
        {
            std::queue<int> bfs_q;
            for (int cy = 0; cy < height; ++cy)
            {
                for (int cx = 0; cx < width; ++cx)
                {
                    int val = map_->data[cy * width + cx];
                    if (val < 0 || val > occupied_threshold_) {
                        int cidx = cy * width + cx;
                        dist_to_wall[cidx] = 0;
                        bfs_q.push(cidx);
                    }
                }
            }
            while (!bfs_q.empty())
            {
                int cur = bfs_q.front(); bfs_q.pop();
                int cx = cur % width;
                int cy = cur / width;
                for (int k = 0; k < 8; ++k)
                {
                    int nx = cx + dx8[k];
                    int ny = cy + dy8[k];
                    if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
                    int nidx = ny * width + nx;
                    if (dist_to_wall[nidx] == std::numeric_limits<int>::max()) {
                        dist_to_wall[nidx] = dist_to_wall[cur] + 1;
                        bfs_q.push(nidx);
                    }
                }
            }
        }

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

                while (!q.empty())
                {
                    int current = q.front();
                    q.pop();
                    component.push_back(current);

                    int cx = current % width;
                    int cy = current / width;

                    for (int k = 0; k < 8; ++k)
                    {
                        int nx = cx + dx8[k];
                        int ny = cy + dy8[k];

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

                    // Triem la cel·la de la component amb MÀXIMA distància a la paret
                    // (el punt més allunyat de tot obstacle → el més central de l'espai lliure)
                    int best_clearance = -1;
                    for (int cell_idx : component)
                    {
                        int d = dist_to_wall[cell_idx];
                        if (d > best_clearance)
                        {
                            best_clearance = d;
                            best_mx = cell_idx % width;
                            best_my = cell_idx / width;
                        }
                    }
                    ROS_WARN("[AREA] Component seleccionada: %d cel·les, clearance màx: %d cel·les (%.2f m)",
                             count, best_clearance, best_clearance * res);
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

    // Tria cap a quin costat girar segons l'espai lliure mes gran. El bloqueig temporal
    // evita oscil·lacions esquerra/dreta quan les lectures son semblants.
    void chooseTurnDirection(float left_clear, float right_clear)
    {
        const float DIFF = 0.10;
        ros::Time now = ros::Time::now();

        // Bloqueig temporal: no canviem de sentit fins que no hagi passat MIN_TURN_LOCK_SEC
        if (!last_turn_change_time_.isZero() &&
            (now - last_turn_change_time_).toSec() < MIN_TURN_LOCK_SEC) {
            return;
        }

        int new_dir = turn_direction_;
        if (left_clear > right_clear + DIFF) {
            new_dir = 1;
        }
        else if (right_clear > left_clear + DIFF) {
            new_dir = -1;
        }
        // Si és simètric, mantenim la direcció actual

        if (new_dir != turn_direction_) {
            turn_direction_ = new_dir;
            last_turn_change_time_ = now;
        }
    }

    // Control reactiu d'exploracio: avança si davant esta lliure i gira si hi ha obstacle.
    // Es deliberadament simple per no competir amb gmapping; nomes cal moure's segur.
    void obstacleAvoidanceMove(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        geometry_msgs::Twist vel;
        if (msg->ranges.empty()) {
            stopRobot();
            return;
        }

        // --- Detecció d'engancxament durant exploració ---
        {
            double rx, ry, ryaw;
            if (getRobotPose(rx, ry, ryaw)) {
                ros::Time now = ros::Time::now();
                if (last_exploration_progress_time_.isZero()) {
                    last_exploration_x_ = rx;
                    last_exploration_y_ = ry;
                    last_exploration_progress_time_ = now;
                }
                else if ((now - last_exploration_progress_time_).toSec() > 2.0) {
                    double moved = std::hypot(rx - last_exploration_x_, ry - last_exploration_y_);
                    if (moved < 0.03) {
                        exploration_stall_counter_++;
                        ROS_WARN("[STUCK] Robot immòbil durant exploració (%.3f m). Counter: %d",
                                 moved, exploration_stall_counter_);
                    } else {
                        exploration_stall_counter_ = 0;
                    }
                    last_exploration_x_ = rx;
                    last_exploration_y_ = ry;
                    last_exploration_progress_time_ = now;
                }

                // Escape d'emergència: invertim el sentit de gir si portem massa temps aturats
                if (exploration_stall_counter_ > 5) {
                    exploration_stall_counter_ = 0;
                    turn_direction_ = -turn_direction_;
                    last_turn_change_time_ = ros::Time(0);
                    ROS_ERROR("[STUCK] Escape d'emergència! Invertint sentit de gir a %d.", turn_direction_);
                }
            }
        }

        // Zones de lectura làser (en graus)
        float front              = minRangeDeg(msg, 330.0,  30.0);
        float front_left         = minRangeDeg(msg,  20.0,  80.0);
        float front_right        = minRangeDeg(msg, 280.0, 340.0);
        float front_left_corner  = minRangeDeg(msg,  30.0,  60.0);  // diagonal esquerra
        float front_right_corner = minRangeDeg(msg, 300.0, 330.0);  // diagonal dreta
        float left               = minRangeDeg(msg,  70.0, 120.0);
        float right              = minRangeDeg(msg, 240.0, 290.0);

        float left_clear  = std::min(front_left, left);
        float right_clear = std::min(front_right, right);

        // ---- LÒGICA ADAPTADA AL MODEL DE WAFFLE ----
        bool very_close =
            front              < CRITICAL_ZONE ||
            front_left         < CRITICAL_ZONE ||
            front_right        < CRITICAL_ZONE ||
            front_left_corner  < (CRITICAL_ZONE - 0.05) ||
            front_right_corner < (CRITICAL_ZONE - 0.05);

        bool obstacle_ahead =
            front      < WARNING_ZONE ||
            front_left < WARNING_ZONE ||
            front_right< WARNING_ZONE;

        if (very_close)
        {
            chooseTurnDirection(left_clear, right_clear);

            // Reculem prou per sortir realment de la zona crítica
            vel.linear.x  = -0.06;
            vel.angular.z = turn_direction_ * 0.22;

            ROS_WARN_THROTTLE(1.0,
                "[EXPLORACIO] Zona crítica Waffle! Reculant. dir=%d front=%.2f fl=%.2f fr=%.2f",
                turn_direction_, front, front_left, front_right);
            publishState("exploracio: zona critica, reculant");
        }
        else if (obstacle_ahead)
        {
            chooseTurnDirection(left_clear, right_clear);

            vel.linear.x  = 0.020;
            vel.angular.z = turn_direction_ * 0.35;

            ROS_INFO_THROTTLE(1.0,
                "[EXPLORACIO] Obstacle a zona de seguretat. Girant. dir=%d", turn_direction_);
            publishState("exploracio: obstacle detectat a zona de seguretat");
        }
        else
        {
            // Velocitat proporcional a la distància de l'obstacle més proper al front
            // Com més a prop, més lent. Entre WARNING_ZONE i 1.5m frenada gradual.
            const double SLOW_ZONE = 1.50; // distància a partir de la qual ja comencem a frenar
            double closest_front = static_cast<double>(std::min({front, front_left, front_right,
                                                                  front_left_corner, front_right_corner}));
            double speed_factor = 1.0;
            if (closest_front < SLOW_ZONE) {
                // Interpolació lineal: 1.0 a SLOW_ZONE, 0.25 a WARNING_ZONE
                speed_factor = 0.25 + 0.75 * (closest_front - WARNING_ZONE) / (SLOW_ZONE - WARNING_ZONE);
                speed_factor = std::max(0.25, std::min(1.0, speed_factor));
            }
            vel.linear.x  = max_linear_speed_ * speed_factor;
            vel.angular.z = 0.0;

            // Correccions laterals suaus per evitar fregament
            if (left < ROBOT_RADIUS && right < ROBOT_RADIUS)
            {
                vel.angular.z = (left < right) ? -0.15 : 0.15;
                publishState("exploracio: passadis molt estret per al Waffle");
            }
            else if (left < ROBOT_RADIUS)
            {
                vel.angular.z = -0.15;
                publishState("exploracio: corregint paret esquerra");
            }
            else if (right < ROBOT_RADIUS)
            {
                vel.angular.z = 0.15;
                publishState("exploracio: corregint paret dreta");
            }
            else
            {
                publishState("exploracio: avancant lliure");
            }

            ROS_INFO_THROTTLE(2.0, "[EXPLORACIO] Segur. front=%.2f left=%.2f right=%.2f", front, left, right);
        }

        vel_pub_.publish(vel);
    }

    // Navegacio simple cap a un objectiu en el mapa. Manté evitacio basica d'obstacles
    // mentre intenta tornar al punt inicial o anar a la zona de dibuix.
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

	ros::Time now = ros::Time::now();
	if (last_progress_time_.isZero()) {
		last_progress_time_ = now;
		last_distance_to_goal_ = distance;
	}

	// Cada 2 segons comprovem si el robot progressa realment almenys 3 cm
        if ((now - last_progress_time_).toSec() > 2.0) {
            if (std::fabs(last_distance_to_goal_ - distance) < 0.03) {
                stall_counter_++;
                ROS_WARN("[SEGURETAT] El robot sembla immòbil (dist: %.2f). Incrementant stall_counter: %d", distance, stall_counter_);
            } else {
                stall_counter_ = 0;
            }
            last_distance_to_goal_ = distance; // sempre actualitzem, tant si es mou com si no
            last_progress_time_ = now;
        }

        // Si portem 4 comprovacions fallides seguides (8 segons), saltem la fase
        if (stall_counter_ > 4) {
            ROS_ERROR("[CRÍTIC] Robot bloquejat o SLAM congelat! Forçant canvi de mode a COMPUTING_GOAL per seguretat.");
            stopRobot();
            stall_counter_ = 0;
            last_progress_time_ = ros::Time(0);
            last_distance_to_goal_ = 999.0;

            if (mode_ == RETURNING_TO_START) {
                mode_ = COMPUTING_GOAL;
                publishPhase("COMPUTING_FREE_AREA");
                return true;
            }
            return false;
        }

        if (mode_ == RETURNING_TO_START)
        {
            ROS_INFO_THROTTLE(1.0, "[RETURN] Navegant cap al punt inicial. dist=%.2f", distance);
            publishState("return_home: navegant cap al punt inicial");
        }

        if (distance < goal_tolerance_)
        {
            stopRobot();

            if (mode_ == RETURNING_TO_START)
            {
                ROS_WARN("[RETURN] He arribat al punt inicial. Retorn a l'inici.");
                publishState("return_home: punt inicial revisitat, retorn a l'inici");
                return true;
            }

            found_space_ = true;
            mode_ = FINISHED;

            publishState("FINISHED: robot al centre de la millor zona.");
            publishPhase("READY_FOR_VISION");
            publishNavigationDone(true);
            return true;
        }

        float front = minRangeDeg(msg, 330.0, 30.0);
        float front_left = minRangeDeg(msg, 20.0, 80.0);
        float front_right = minRangeDeg(msg, 280.0, 340.0);

        // Seguretat incrementada en mode navegació directa
        if (front < WARNING_ZONE || front_left < (CRITICAL_ZONE + 0.05) || front_right < (CRITICAL_ZONE + 0.05))
        {
            publishState("go_to_goal: obstacle al cami, activant evitacio");
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
            vel.angular.z = (angle_error > 0.0) ? angular_speed : -angular_speed;
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

            publishState("go_to_goal: avancant cap al centre");
        }

        vel_pub_.publish(vel);
        return false;
    }

    // Criteri de final d'exploracio. En aquesta versio conservadora s'explora fins
    // al temps maxim configurat per obtenir un comportament previsible.
    bool shouldFinishExploration(double elapsed, std::string& reason)
    {
        // Sempre explorar fins al temps màxim configurat
        if (max_explore_time_ > 0.0 && elapsed >= max_explore_time_)
        {
            reason = "temps maxim d'exploracio assolit";
            return true;
        }

        // Encara no hem arribat al temps màxim: seguim explorant
        std::stringstream ss;
        ss << "explorant fins al temps maxim. elapsed=" << elapsed << " / " << max_explore_time_ << " s";
        reason = ss.str();
        return false;
    }

    // Callback principal del node. Cada LaserScan fa avançar la maquina d'estats:
    // explorar, tornar a l'inici, calcular zona lliure, anar-hi i avisar READY_FOR_VISION.
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

            recordFeatureIfNeeded();
            recordScanFeatures(msg);

            double stable_for = (ros::Time::now() - last_map_growth_time_).toSec();
            ROS_INFO_THROTTLE(10.0, "[SLAM] Explorant. coneguda=%.2f m2 | estable=%.1f/%.1f s", known_area_, stable_for, map_stable_time_);

            std::string reason;
            bool finish_exploration = shouldFinishExploration(elapsed, reason);

            if (!finish_exploration)
            {
                obstacleAvoidanceMove(msg);
                return;
            }

            stopRobot();
            ROS_WARN("[SLAM] Exploracio acabada. Motiu: %s", reason.c_str());
            publishState("slam: exploracio acabada, retornant al punt inicial");

            if (!start_pose_set_ && !captureStartPoseIfNeeded())
            {
                ROS_WARN("[RETURN] Error al capturar punt inicial.");
            }

            if (start_pose_set_)
            {
                goal_x_ = start_x_;
                goal_y_ = start_y_;
                goal_available_ = true;
                mode_ = RETURNING_TO_START;
                stall_counter_ = 0;
                last_progress_time_ = ros::Time(0);
                last_distance_to_goal_ = 999.0;

                publishPhase("RETURNING_TO_START");
                publishNavigationDone(false);
                return;
            }

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

            recordCurrentPoseAsFeature();
            saveFeaturesIfNeeded();
            saveMapIfNeeded();
            mode_ = COMPUTING_GOAL;
            publishPhase("COMPUTING_FREE_AREA");
        }

        if (mode_ == COMPUTING_GOAL)
        {
            bool ok = findBestFreeSpace(goal_x_, goal_y_);
            if (!ok)
            {
                publishNavigationDone(false);
                publishPhase("SLAM_EXPLORING");
                mode_ = EXPLORING;
                start_time_ = ros::Time::now();
                last_map_growth_time_ = ros::Time::now();
                best_known_area_ = 0.0; // reinici complet per no bloquejar el criteri d'estabilitat
                return;
            }

            goal_available_ = true;
            mode_ = GOING_TO_GOAL;
            stall_counter_ = 0;
            last_progress_time_ = ros::Time(0);
            last_distance_to_goal_ = 999.0;
            publishPhase("GOING_TO_GOAL");
            return;
        }

        if (mode_ == GOING_TO_GOAL)
        {
            if (!goal_available_)
            {
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
    ros::init(argc, argv, "free_space_explorer_node");
    FindFreeSpaceExplorer node;
    ros::spin();
    return 0;
}
