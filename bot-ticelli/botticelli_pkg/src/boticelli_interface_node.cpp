#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>

#include <sys/types.h>
#include <sys/wait.h>
#include <sys/select.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>

#include <cctype>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

/*
 * Node d'interficie principal del projecte Bot-ticelli.
 *
 * Aquest fitxer coordina el sistema complet: mostra el menu, llanÃ§a els launch
 * necessaris, envia ordres als altres nodes i espera els missatges que confirmen
 * que cada etapa ha acabat. No fa SLAM, visio ni seguiment del dibuix directament;
 * aquestes tasques queden separades en nodes independents.
 *
 * Flux principal:
 *  - Mode 1: gmapping construeix el mapa, l'explorador troba zona lliure i es passa a visio.
 *  - Mode 2: map_server carrega un mapa existent, AMCL localitza el robot i es dibuixa.
 *
 * Aquesta separacio per topics facilita l'explicacio del projecte i permet depurar
 * cada bloc amb rostopic, rosnode i RViz.
 */
class BoticelliInterface
{
private:
    ros::NodeHandle nh_;

    // Publishers d'ordres cap als altres nodes. La interfÃ­cie envia esdeveniments
    // simples: capturar contorn, publicar dimensions, anar al punt inicial i comenÃ§ar.
    ros::Publisher cmd_pub_;
    ros::Publisher capture_pub_;
    ros::Publisher dimensions_pub_;
    ros::Publisher go_to_drawing_pub_;
    ros::Publisher start_drawing_pub_;

    // Subscribers d'estat. Cada node informa quan la seva feina esta preparada o acabada.
    ros::Subscriber contour_info_sub_;
    ros::Subscriber drawing_path_sub_;
    ros::Subscriber drawing_ready_sub_;
    ros::Subscriber drawing_finished_sub_;
    ros::Subscriber drawing_progress_sub_;
    ros::Subscriber current_phase_sub_;

    ros::ServiceClient global_localization_client_;

    // Flags actualitzats pels callbacks. El flux interactiu espera aquests flags
    // abans de passar a la fase seguent.
    bool contour_info_received_;
    bool drawing_path_received_;
    bool drawing_ready_received_;
    bool drawing_finished_received_;
    bool drawing_progress_received_;
    bool ready_for_vision_received_;

    // Informacio del dibuix i de l'estat global que es mostra per terminal.
    double aspect_ratio_;
    std::string drawing_path_frame_;
    double drawing_progress_current_;
    double drawing_progress_total_;
    double drawing_progress_percent_;
    std::string current_phase_;

    // Identificadors dels processos roslaunch oberts per poder-los tancar ordenadament.
    pid_t slam_pid_;
    pid_t vision_pid_;
    pid_t drawing_pid_;
    pid_t draw_full_pid_;

    std::string package_path_;
    std::string log_dir_;

public:
    BoticelliInterface()
    {
        contour_info_received_ = false;
        drawing_path_received_ = false;
        drawing_ready_received_ = false;
        drawing_finished_received_ = false;
        drawing_progress_received_ = false;
        ready_for_vision_received_ = false;

        aspect_ratio_ = 1.0;
        drawing_path_frame_ = "";
        drawing_progress_current_ = 0.0;
        drawing_progress_total_ = 0.0;
        drawing_progress_percent_ = 0.0;
        current_phase_ = "";

        slam_pid_ = -1;
        vision_pid_ = -1;
        drawing_pid_ = -1;
        draw_full_pid_ = -1;

        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        capture_pub_ = nh_.advertise<std_msgs::Bool>("/capture_contour", 10);
        dimensions_pub_ = nh_.advertise<geometry_msgs::Vector3>("/drawing_dimensions", 10);
        go_to_drawing_pub_ = nh_.advertise<std_msgs::Bool>("/go_to_drawing", 10);
        start_drawing_pub_ = nh_.advertise<std_msgs::Bool>("/start_drawing", 10);

        contour_info_sub_ = nh_.subscribe("/contour_info", 10, &BoticelliInterface::contourInfoCallback, this);
        drawing_path_sub_ = nh_.subscribe("/drawing_path", 1, &BoticelliInterface::drawingPathCallback, this);
        drawing_ready_sub_ = nh_.subscribe("/drawing_ready", 10, &BoticelliInterface::drawingReadyCallback, this);
        drawing_finished_sub_ = nh_.subscribe("/drawing_finished", 10, &BoticelliInterface::drawingFinishedCallback, this);
        drawing_progress_sub_ = nh_.subscribe("/drawing_progress", 10, &BoticelliInterface::drawingProgressCallback, this);
        current_phase_sub_ = nh_.subscribe("/boticelli/current_phase", 10, &BoticelliInterface::currentPhaseCallback, this);

        global_localization_client_ = nh_.serviceClient<std_srvs::Empty>("/global_localization");

        package_path_ = getCommandOutput("rospack find botticelli_pkg");
        trim(package_path_);

        log_dir_ = std::string(getenv("HOME")) + "/.ros/boticelli_logs";
        std::system(("mkdir -p " + log_dir_).c_str());
    }

    void shutdown()
    {
        stopRobot();
        stopAllLaunches();
        stopRobot();
    }

    // Bucle principal de l'aplicacio. Es deixa en format interactiu perque, en el robot
    // real, l'usuari ha de confirmar camera, localitzacio i retolador.
    void run()
    {
        while (ros::ok()) {
            printMainMenu();

            std::string option;
            std::getline(std::cin, option);

            if (option == "1") {
                slamAndVisionMode();
            }
            else if (option == "2") {
                drawModeWithExistingMap();
            }
            else if (option == "3") {
                shutdown();
                clearScreen();
                std::cout << "Sortint de Bot-ticelli." << std::endl;
                return;
            }
            else {
                std::cout << "Opcio no valida." << std::endl;
                waitEnter("Prem ENTER per tornar al menu...");
            }
        }
    }

private:
    // La visio publica informacio del contorn. La relacio d'aspecte permet escalar
    // el dibuix en metres sense deformar la figura detectada a la camera.
    void contourInfoCallback(const geometry_msgs::Vector3::ConstPtr& msg)
    {
        aspect_ratio_ = msg->z;

        if (aspect_ratio_ <= 0.0) {
            aspect_ratio_ = 1.0;
        }

        contour_info_received_ = true;
    }

    // El planner publica la trajectoria final. Es guarda el frame per avisar si el path
    // no esta referenciat al mapa, que es el frame esperat per navegar i dibuixar.
    void drawingPathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        if (!msg->poses.empty()) {
            drawing_path_received_ = true;
            drawing_path_frame_ = msg->header.frame_id;
        }
    }

    // El navigator confirma que el robot ja ha arribat al punt inicial del dibuix.
    void drawingReadyCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (msg->data) {
            drawing_ready_received_ = true;
        }
    }

    // El follower confirma que ja ha recorregut tota la trajectoria de dibuix.
    void drawingFinishedCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (msg->data) {
            drawing_finished_received_ = true;
        }
    }

    // Progres publicat pel follower: punt actual, total de punts i percentatge.
    // La interfÃ­cie nomes ho mostra; el calcul real el fa el node que segueix el path.
    void drawingProgressCallback(const geometry_msgs::Vector3::ConstPtr& msg)
    {
        drawing_progress_current_ = msg->x;
        drawing_progress_total_ = msg->y;
        drawing_progress_percent_ = msg->z;
        drawing_progress_received_ = true;
    }

    // L'explorador publica fases textuals. READY_FOR_VISION indica que el SLAM ja ha
    // arribat al punt on es pot passar a captura i dibuix.
    void currentPhaseCallback(const std_msgs::String::ConstPtr& msg)
    {
        current_phase_ = msg->data;

        if (current_phase_.find("READY_FOR_VISION") != std::string::npos) {
            ready_for_vision_received_ = true;
        }
    }

    void printMainMenu()
    {
        clearScreen();

        std::cout << "============================================" << std::endl;
        std::cout << "                BOT-TICELLI" << std::endl;
        std::cout << "============================================" << std::endl;
        std::cout << "Sistema de mapatge, localitzacio, visio i dibuix" << std::endl;
        std::cout << "amb TurtleBot3 Waffle." << std::endl;
        std::cout << std::endl;

        std::cout << "Selecciona el mode d'us:" << std::endl;
        std::cout << std::endl;
        std::cout << "  1 - Crear mapa amb SLAM i continuar a visio" << std::endl;
        std::cout << "  2 - Dibuixar amb mapa existent" << std::endl;
        std::cout << "  3 - Sortir" << std::endl;
        std::cout << std::endl;
        std::cout << "Opcio: ";
    }

    // Mode complet amb mapa nou. Gmapping continua obert quan es passa a visio, de manera
    // que no cal relocalitzar amb AMCL: el robot ja esta localitzat dins el mapa que crea.
    void slamAndVisionMode()
    {
        clearScreen();

        std::cout << "============================================" << std::endl;
        std::cout << "          MODE 1 - SLAM + VISIO" << std::endl;
        std::cout << "============================================" << std::endl;
        std::cout << std::endl;

        std::cout << "Aquest mode fa el flux complet:" << std::endl;
        std::cout << std::endl;
        std::cout << "  1. Crea el mapa amb gmapping." << std::endl;
        std::cout << "  2. Mou el robot amb free_space_explorer_node." << std::endl;
        std::cout << "  3. Guarda el mapa amb el nom que indiquis." << std::endl;
        std::cout << "  4. Quan arriba a READY_FOR_VISION, passa directament a visio." << std::endl;
        std::cout << std::endl;

        std::cout << "En aquest mode NO es torna a fer AMCL, perquÃ¨ gmapping continua actiu." << std::endl;
        stopAllLaunches();

        std::string map_name = askNewMapName();
        std::string map_base = package_path_ + "/maps/" + map_name;
        std::string slam_args = "map_output_file:=" + map_base +
                                " features_output_file:=" + map_base + "_features.txt";

        ready_for_vision_received_ = false;
        current_phase_ = "";

        slam_pid_ = launchProcess("boticelli_slam.launch",
                                  "slam_" + map_name + ".log",
                                  slam_args);

        std::cout << std::endl;
        std::cout << "SLAM automatic en marxa." << std::endl;
        std::cout << "Esperant READY_FOR_VISION..." << std::endl;
        std::cout << std::endl;
        std::cout << "Per veure logs:" << std::endl;
        std::cout << "  tail -f " << log_dir_ << "/slam_" << map_name << ".log" << std::endl;
        std::cout << std::endl;

        bool ready = waitForReadyForVision(300.0);

        if (!ready) {
            std::cout << std::endl;
            std::cout << "No s'ha rebut READY_FOR_VISION dins del temps esperat." << std::endl;
            std::cout << "Vols passar igualment a visio amb el mapa actual? [s/n]: ";

            std::string option;
            std::getline(std::cin, option);

            if (!(option == "s" || option == "S")) {
                stopAllLaunches();
                std::cout << "Mode SLAM aturat." << std::endl;
                waitEnter("Prem ENTER per tornar al menu...");
                return;
            }
        }

        std::cout << std::endl;
        std::cout << "SLAM preparat. Aturant nomÃ©s l'explorador..." << std::endl;

        stopRobot();
        stopExplorerOnly();

        std::cout << "Gmapping continua actiu. Ara s'obren visio i nodes de dibuix." << std::endl;

        vision_pid_ = launchProcess("vision.launch", "vision_after_slam.log");
        drawing_pid_ = launchProcess("drawing.launch", "drawing_after_slam.log");

        std::cout << std::endl;
        std::cout << "Sistema de visio i dibuix iniciat." << std::endl;
        std::cout << "Ara es passa directament a la captura de la figura." << std::endl;

        bool ok = visionAndDrawingFlow(false);

        if (!ok) {
            std::cout << std::endl;
            std::cout << "El proces de visio/dibuix no s'ha completat." << std::endl;
            waitEnter("Prem ENTER per tornar al menu...");
            stopAllLaunches();
            return;
        }

        waitForDrawingFinished();

        stopRobot();

        std::cout << std::endl;
        std::cout << "Dibuix acabat. El robot queda aturat." << std::endl;
        std::cout << "RViz queda obert per revisar /drawing_path i /executed_path." << std::endl;
        std::cout << "Quan hagis acabat de revisar, prem ENTER per tornar al menu." << std::endl;
        waitEnter();
    }

    // Mode amb mapa existent. En aquest cas si cal map_server + AMCL, perque el robot
    // ha d'estimar la seva posicio dins un mapa guardat anteriorment.
    void drawModeWithExistingMap()
    {
        clearScreen();

        std::cout << "============================================" << std::endl;
        std::cout << "      MODE 2 - DIBUIX AMB MAPA EXISTENT" << std::endl;
        std::cout << "============================================" << std::endl;
        std::cout << std::endl;

        std::cout << "Aquest mode carrega el mapa que seleccionis, utilitza AMCL," << std::endl;
        std::cout << "obre la visio i executa el dibuix." << std::endl;
        std::cout << std::endl;

        stopAllLaunches();
        resetDrawFlags();

        std::string map_name = askExistingMapName();
        std::string map_file = package_path_ + "/maps/" + map_name + ".yaml";
        std::string draw_args = "map_file:=" + map_file;

        draw_full_pid_ = launchProcess("boticelli_draw.launch",
                                       "draw_" + map_name + ".log",
                                       draw_args);

        std::cout << "Sistema de dibuix iniciat." << std::endl;
        std::cout << "Esperant que s'obrin RViz i la finestra de camera..." << std::endl;

        ros::Duration(4.0).sleep();

        localizationStep();

        bool ok = visionAndDrawingFlow(true);

        if (!ok) {
            std::cout << std::endl;
            std::cout << "El proces de dibuix no s'ha completat." << std::endl;
            waitEnter("Prem ENTER per tornar al menu...");
            stopAllLaunches();
            return;
        }

        waitForDrawingFinished();

        stopRobot();

        std::cout << std::endl;
        std::cout << "Dibuix acabat. El robot queda aturat." << std::endl;
        std::cout << "RViz queda obert per revisar /drawing_path i /executed_path." << std::endl;
        std::cout << "Quan hagis acabat de revisar, prem ENTER per tornar al menu." << std::endl;
        waitEnter();
    }

    // Flux comu de dibuix: captura, planificacio, navegacio al punt inicial i execucio.
    // Cada pas retorna bool per poder aturar el flux si algun node no respon.
    bool visionAndDrawingFlow(bool show_title)
    {
        if (show_title) {
            std::cout << std::endl;
            std::cout << "Entrant a la part de visio i dibuix." << std::endl;
        }

        resetDrawFlags();

        if (!captureStep()) {
            return false;
        }

        if (!pathStep()) {
            return false;
        }

        if (!navigationStep()) {
            return false;
        }

        if (!startDrawingStep()) {
            return false;
        }

        return true;
    }

    void resetDrawFlags()
    {
        contour_info_received_ = false;
        drawing_path_received_ = false;
        drawing_ready_received_ = false;
        drawing_finished_received_ = false;
        drawing_progress_received_ = false;
        drawing_path_frame_ = "";
        drawing_progress_current_ = 0.0;
        drawing_progress_total_ = 0.0;
        drawing_progress_percent_ = 0.0;
    }

    // Localitzacio assistida amb AMCL. /global_localization reparteix particules pel mapa
    // i el gir del robot dona mesures laser de diferents direccions per convergir millor.
    void localizationStep()
    {
        std::cout << std::endl;
        std::cout << "PAS 1 - Localitzacio amb AMCL" << std::endl;
        std::cout << std::endl;

        std::cout << "Vols fer una relocalitzacio global i girar una mica el robot? [s/n]: ";

        std::string option;
        std::getline(std::cin, option);

        if (option == "s" || option == "S") {
            callGlobalLocalization();

            std::cout << std::endl;
            std::cout << "El robot comenÃ§ara a girar per ajudar AMCL." << std::endl;
            std::cout << "Prem ENTER quan vegis el robot ben situat al mapa." << std::endl;
            std::cout << std::endl;

            rotateUntilEnter(0.35, 120.0);
        }

        std::cout << std::endl;
        std::cout << "Comprova a RViz que el robot esta ben situat al mapa." << std::endl;
        std::cout << "Quan estigui correcte, prem ENTER." << std::endl;
        waitEnter();
    }

    // La captura es fa sota demanda: l'usuari confirma quan la figura es veu be i llavors
    // s'envia /capture_contour al node de visio.
    bool captureStep()
    {
        while (ros::ok()) {
            std::cout << std::endl;
            std::cout << "PAS 2 - Captura de la figura" << std::endl;
            std::cout << std::endl;
            std::cout << "ColÂ·loca el full davant la camera." << std::endl;
            std::cout << "Quan el contorn es vegi correcte a la finestra de visio, prem ENTER." << std::endl;
            waitEnter();

            contour_info_received_ = false;

            waitForSubscribers(capture_pub_, "/capture_contour", 8.0);
            publishBoolRepeated(capture_pub_, true, 10);

            std::cout << "Esperant /contour_info..." << std::endl;

            if (waitUntilContourInfo(25.0)) {
                std::cout << "Contorn rebut correctament." << std::endl;
                return true;
            }

            std::cout << std::endl;
            std::cout << "No s'ha rebut /contour_info." << std::endl;
            std::cout << "Vols reintentar la captura? [s/n]: ";

            std::string option;
            std::getline(std::cin, option);

            if (!(option == "s" || option == "S")) {
                return false;
            }
        }

        return false;
    }

    // Converteix la mida escollida per l'usuari en dimensions metriques. El planner
    // mantÃ© la relacio d'aspecte de la figura utilitzant l'aspect_ratio rebut de visio.
    bool pathStep()
    {
        std::cout << std::endl;
        std::cout << "PAS 3 - Mida i trajectoria" << std::endl;
        std::cout << std::endl;

        double width = askWidth();
        double height = width / aspect_ratio_;

        geometry_msgs::Vector3 msg;
        msg.x = width;
        msg.y = height;
        msg.z = 0.0;

        drawing_path_received_ = false;

        waitForSubscribers(dimensions_pub_, "/drawing_dimensions", 5.0);

        for (int i = 0; i < 8; ++i) {
            dimensions_pub_.publish(msg);
            ros::Duration(0.15).sleep();
        }

        std::cout << "Mida publicada:" << std::endl;
        std::cout << "  amplada: " << width << " m" << std::endl;
        std::cout << "  alcada:  " << height << " m" << std::endl;
        std::cout << "Esperant /drawing_path..." << std::endl;

        if (!waitUntilDrawingPath(20.0)) {
            std::cout << "No s'ha rebut /drawing_path. Revisa drawing_planner_node." << std::endl;
            return false;
        }

        std::cout << "Trajectoria generada." << std::endl;
        std::cout << "Frame: " << drawing_path_frame_ << std::endl;

        if (drawing_path_frame_ != "map") {
            std::cout << "AVIS: la trajectoria no esta en frame map." << std::endl;
        }

        return true;
    }

    // Mou el robot fins al primer punt del dibuix abans de baixar el retolador.
    // AixÃ­ s'eviten marques no desitjades durant el posicionament inicial.
    bool navigationStep()
    {
        std::cout << std::endl;
        std::cout << "PAS 4 - Anar al punt inicial" << std::endl;
        std::cout << std::endl;
        std::cout << "Revisa la trajectoria a RViz." << std::endl;
        std::cout << "Quan estigui be, prem ENTER per moure el robot al punt inicial." << std::endl;
        waitEnter();

        drawing_ready_received_ = false;

        waitForSubscribers(go_to_drawing_pub_, "/go_to_drawing", 5.0);
        publishBoolRepeated(go_to_drawing_pub_, true, 8);

        std::cout << "Robot anant al punt inicial..." << std::endl;

        if (!waitUntilDrawingReady(120.0)) {
            std::cout << "No s'ha rebut /drawing_ready." << std::endl;
            return false;
        }

        std::cout << "Robot situat al punt inicial." << std::endl;
        return true;
    }

    // Dona l'ordre d'inici al follower. La pausa manual permet baixar el retolador
    // exactament abans que el robot comenci a seguir la trajectoria.
    bool startDrawingStep()
    {
        std::cout << std::endl;
        std::cout << "PAS 5 - Dibuix" << std::endl;
        std::cout << std::endl;
        std::cout << "Baixa el retolador." << std::endl;
        std::cout << "Quan estigui preparat, prem ENTER per comenÃ§ar." << std::endl;
        waitEnter();

        drawing_finished_received_ = false;
        drawing_progress_received_ = false;
        drawing_progress_current_ = 0.0;
        drawing_progress_total_ = 0.0;
        drawing_progress_percent_ = 0.0;

        waitForSubscribers(start_drawing_pub_, "/start_drawing", 5.0);
        publishBoolRepeated(start_drawing_pub_, true, 8);

        std::cout << "Dibuix iniciat." << std::endl;
        std::cout << "Observa el traÃ§ real i /executed_path a RViz." << std::endl;

        return true;
    }

    // Espera el final del dibuix i mostra una barra de progres. Aixo dona feedback
    // immediat encara que l'usuari no estigui mirant tots els topics de RViz.
    bool waitForDrawingFinished()
    {
        std::cout << std::endl;
        std::cout << "Dibuix en execucio." << std::endl;
        std::cout << "Progres:" << std::endl;

        ros::Rate rate(5.0);
        ros::Time last_print = ros::Time(0);

        while (ros::ok() && !drawing_finished_received_) {
            ros::spinOnce();

            if ((ros::Time::now() - last_print).toSec() >= 0.5) {
                printDrawingProgress();
                last_print = ros::Time::now();
            }

            rate.sleep();
        }

        ros::spinOnce();
        printDrawingProgress();
        std::cout << std::endl;

        return drawing_finished_received_;
    }

    // Dibuixa la barra de progres en una sola linia de terminal.
    void printDrawingProgress()
    {
        int percent = static_cast<int>(drawing_progress_percent_ + 0.5);

        if (percent < 0) {
            percent = 0;
        }

        if (percent > 100) {
            percent = 100;
        }

        int width = 30;
        int filled = percent * width / 100;

        std::string bar_done(filled, '#');
        std::string bar_left(width - filled, '-');

        std::cout << "\r  ["
                  << bar_done
                  << bar_left
                  << "] "
                  << percent
                  << "%";

        if (drawing_progress_received_ && drawing_progress_total_ > 0.0) {
            std::cout << "  punt "
                      << static_cast<int>(drawing_progress_current_) + 1
                      << "/"
                      << static_cast<int>(drawing_progress_total_);
        }
        else {
            std::cout << "  esperant dades...";
        }

        std::cout << std::flush;
    }

    // Espera que l'explorador publiqui READY_FOR_VISION. El timeout evita quedar bloquejat
    // indefinidament si el robot no troba zona lliure o si gmapping no publica mapa.
    bool waitForReadyForVision(double timeout)
    {
        ros::Rate rate(10);
        ros::Time start_time = ros::Time::now();
        ros::Time last_print = ros::Time::now();

        while (ros::ok() && !ready_for_vision_received_) {
            double elapsed = (ros::Time::now() - start_time).toSec();

            if ((ros::Time::now() - last_print).toSec() > 3.0) {
                if (current_phase_.empty()) {
                    std::cout << "[SLAM] Esperant fase del node..." << std::endl;
                }
                else {
                    std::cout << "[SLAM] Fase actual: " << current_phase_ << std::endl;
                }

                last_print = ros::Time::now();
            }

            if (elapsed > timeout) {
                return false;
            }

            rate.sleep();
        }

        return ready_for_vision_received_;
    }

    // Crida el servei estandard d'AMCL per reinicialitzar les particules.
    void callGlobalLocalization()
    {
        std::cout << "Cridant /global_localization..." << std::endl;

        if (!global_localization_client_.waitForExistence(ros::Duration(5.0))) {
            std::cout << "No s'ha trobat /global_localization. Pots localitzar manualment a RViz." << std::endl;
            return;
        }

        std_srvs::Empty srv;

        if (global_localization_client_.call(srv)) {
            std::cout << "Global localization executat." << std::endl;
        }
        else {
            std::cout << "No s'ha pogut cridar /global_localization." << std::endl;
        }
    }

    // LlanÃ§a un roslaunch com a proces independent i guarda la sortida en un log.
    // Aixo permet que la interfÃ­cie continuÃ¯ activa mentre els nodes ROS treballen.
    pid_t launchProcess(const std::string& launch_file,
                        const std::string& log_name,
                        const std::string& extra_args = "")
    {
        std::string log_path = log_dir_ + "/" + log_name;
        std::string command = "roslaunch botticelli_pkg " + launch_file;

        if (!extra_args.empty()) {
            command += " " + extra_args;
        }

        std::cout << "LlanÃ§ant: " << command << std::endl;
        std::cout << "Logs detallats: " << log_path << std::endl;
        std::cout << std::endl;

        pid_t pid = fork();

        if (pid == 0) {
            setpgid(0, 0);

            int fd = open(log_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);

            if (fd >= 0) {
                dup2(fd, STDOUT_FILENO);
                dup2(fd, STDERR_FILENO);
                close(fd);
            }

            execlp("bash", "bash", "-lc", command.c_str(), (char*) NULL);
            std::exit(1);
        }

        ros::Duration(5.0).sleep();
        return pid;
    }

    // En mode SLAM nomes es mata l'explorador: gmapping queda viu per mantenir
    // el mapa i la transformacio de localitzacio generada durant el mapatge.
    void stopExplorerOnly()
    {
        stopRobot();
        std::system("rosnode kill /free_space_explorer_node >/dev/null 2>&1");
        ros::Duration(1.0).sleep();
        stopRobot();
    }

    // Tanca un roslaunch de forma progressiva. Primer SIGINT per tancament net,
    // despres senyals mes fortes si algun proces queda penjat.
    void stopOneLaunch(pid_t& pid)
    {
        if (pid > 0) {
            kill(-pid, SIGINT);
            ros::Duration(2.0).sleep();

            kill(-pid, SIGTERM);
            ros::Duration(1.0).sleep();

            kill(-pid, SIGKILL);

            int status;
            waitpid(pid, &status, WNOHANG);

            pid = -1;
        }
    }

    // Atura tots els processos i nodes del projecte per evitar conflictes de topics
    // quan es canvia de mode o es torna al menu.
    void stopAllLaunches()
    {
        stopRobot();

        stopOneLaunch(drawing_pid_);
        stopOneLaunch(vision_pid_);
        stopOneLaunch(draw_full_pid_);
        stopOneLaunch(slam_pid_);

        std::system("rosnode kill /slam_gmapping /free_space_explorer_node /map_server /amcl /vision_node /drawing_planner_node /drawing_navigator_node /drawing_follower_node /rviz >/dev/null 2>&1");
        std::system("pkill -f 'roslaunch botticelli_pkg boticelli_slam.launch' >/dev/null 2>&1");
        std::system("pkill -f 'roslaunch botticelli_pkg boticelli_draw.launch' >/dev/null 2>&1");
        std::system("pkill -f 'roslaunch botticelli_pkg vision.launch' >/dev/null 2>&1");
        std::system("pkill -f 'roslaunch botticelli_pkg drawing.launch' >/dev/null 2>&1");

        stopRobot();
    }

    // Executa una comanda curta i en captura la sortida. S'utilitza per trobar
    // la ruta real del paquet amb rospack.
    std::string getCommandOutput(const std::string& command)
    {
        std::string result;
        char buffer[128];

        FILE* pipe = popen(command.c_str(), "r");
        if (!pipe) {
            return "";
        }

        while (fgets(buffer, sizeof(buffer), pipe) != NULL) {
            result += buffer;
        }

        pclose(pipe);
        return result;
    }

    // Elimina espais i salts de linia sobrants d'una cadena.
    void trim(std::string& text)
    {
        while (!text.empty() &&
               (text.back() == '\n' ||
                text.back() == '\r' ||
                text.back() == ' ')) {
            text.pop_back();
        }
    }

    // Filtra el nom del mapa per evitar caracters problematics en rutes o shell.
    std::string sanitizeMapName(const std::string& raw)
    {
        std::string clean;

        for (char c : raw) {
            if (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-') {
                clean.push_back(c);
            }
            else if (c == ' ') {
                clean.push_back('_');
            }
        }

        if (clean.size() > 5 && clean.substr(clean.size() - 5) == ".yaml") {
            clean = clean.substr(0, clean.size() - 5);
        }

        if (clean.size() > 4 && clean.substr(clean.size() - 4) == ".pgm") {
            clean = clean.substr(0, clean.size() - 4);
        }

        return clean;
    }

    // Comprova si ja existeix un mapa per no sobreescriure proves reals.
    bool mapYamlExists(const std::string& map_name)
    {
        std::string file = package_path_ + "/maps/" + map_name + ".yaml";
        std::ifstream in(file.c_str());
        return in.good();
    }

    // Mostra els mapes YAML disponibles a la carpeta maps.
    void printAvailableMaps()
    {
        std::cout << std::endl;
        std::cout << "Mapes disponibles:" << std::endl;
        std::string cmd = "cd " + package_path_ + "/maps && ls -1 *.yaml 2>/dev/null";
        std::system(cmd.c_str());
        std::cout << std::endl;
    }

    // Demana un nom nou i obliga que no existeixi encara.
    std::string askNewMapName()
    {
        while (ros::ok()) {
            std::cout << std::endl;
            std::cout << "Nom del mapa nou, sense extensio." << std::endl;
            std::cout << "Exemple: mapa_aula2_prova1" << std::endl;
            std::cout << "Nom: ";

            std::string raw;
            std::getline(std::cin, raw);

            std::string name = sanitizeMapName(raw);

            if (name.empty()) {
                std::cout << "Nom buit. Escriu un nom per no sobreescriure mapes." << std::endl;
                continue;
            }

            if (mapYamlExists(name)) {
                std::cout << "Aquest mapa ja existeix: " << name << ".yaml" << std::endl;
                std::cout << "Tria un altre nom per no sobreescriure'l." << std::endl;
                continue;
            }

            return name;
        }

        return "mapa_nou";
    }

    // Selecciona un mapa existent per carregar-lo amb map_server i AMCL.
    std::string askExistingMapName()
    {
        while (ros::ok()) {
            printAvailableMaps();

            std::cout << "Quin mapa vols utilitzar? Escriu el nom sense .yaml" << std::endl;
            std::cout << "Mapa [mapa_aula2]: ";

            std::string raw;
            std::getline(std::cin, raw);

            if (raw.empty()) {
                raw = "mapa_aula2";
            }

            std::string name = sanitizeMapName(raw);

            if (mapYamlExists(name)) {
                return name;
            }

            std::cout << "No trobo el mapa: " << name << ".yaml" << std::endl;
        }

        return "mapa_aula2";
    }

    // Mides fixes del dibuix. Ajuden a fer proves repetibles i eviten errors manuals.
    double askWidth()
    {
        std::cout << "Selecciona mida:" << std::endl;
        std::cout << "  1 - Petita  (0.50 m)" << std::endl;
        std::cout << "  2 - Mitjana (0.80 m)" << std::endl;
        std::cout << "  3 - Gran    (1.00 m)" << std::endl;
        std::cout << "Opcio [2]: ";

        std::string option;
        std::getline(std::cin, option);

        if (option == "1") {
            return 0.50;
        }

        if (option == "3") {
            return 1.00;
        }

        return 0.80;
    }

    bool waitUntilContourInfo(double timeout)
    {
        return waitUntilFlag(&contour_info_received_, timeout);
    }

    bool waitUntilDrawingPath(double timeout)
    {
        return waitUntilFlag(&drawing_path_received_, timeout);
    }

    bool waitUntilDrawingReady(double timeout)
    {
        return waitUntilFlag(&drawing_ready_received_, timeout);
    }

    // Espera generica d'un flag actualitzat per callbacks ROS.
    bool waitUntilFlag(bool* flag, double timeout)
    {
        ros::Time start = ros::Time::now();
        ros::Rate rate(10);

        while (ros::ok() && (ros::Time::now() - start).toSec() < timeout) {
            if (*flag) {
                return true;
            }

            rate.sleep();
        }

        return false;
    }

    // Dona temps als subscribers a connectar-se abans d'enviar una ordre puntual.
    void waitForSubscribers(ros::Publisher& pub,
                            const std::string& topic_name,
                            double timeout)
    {
        ros::Time start = ros::Time::now();
        ros::Rate rate(10);

        while (ros::ok() &&
               pub.getNumSubscribers() == 0 &&
               (ros::Time::now() - start).toSec() < timeout) {
            rate.sleep();
        }

        if (pub.getNumSubscribers() == 0) {
            std::cout << "AVIS: cap subscriptor detectat a " << topic_name << "." << std::endl;
        }
    }

    // Publica una ordre diverses vegades per fer-la robusta a retards de connexio.
    void publishBoolRepeated(ros::Publisher& pub, bool value, int repetitions)
    {
        std_msgs::Bool msg;
        msg.data = value;

        for (int i = 0; i < repetitions; ++i) {
            pub.publish(msg);
            ros::Duration(0.15).sleep();
        }
    }

    // Gira el robot mentre l'usuari comprova la localitzacio a RViz.
    void rotateUntilEnter(double angular_speed, double max_duration)
    {
        geometry_msgs::Twist cmd;
        cmd.angular.z = angular_speed;

        ros::Time start = ros::Time::now();
        ros::Rate rate(10);

        while (ros::ok() && (ros::Time::now() - start).toSec() < max_duration) {
            cmd_pub_.publish(cmd);

            if (enterPressed()) {
                std::string line;
                std::getline(std::cin, line);
                break;
            }

            rate.sleep();
        }

        stopRobot();
    }

    // Comprovacio no bloquejant d'ENTER per no aturar el bucle de gir.
    bool enterPressed()
    {
        fd_set set;
        struct timeval timeout;

        FD_ZERO(&set);
        FD_SET(STDIN_FILENO, &set);

        timeout.tv_sec = 0;
        timeout.tv_usec = 0;

        int rv = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);

        return rv > 0;
    }

    // Publica velocitat zero diverses vegades per assegurar l'aturada.
    void stopRobot()
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;

        for (int i = 0; i < 5; ++i) {
            cmd_pub_.publish(cmd);
            ros::Duration(0.1).sleep();
        }
    }

    // Pausa interactiva estandard.
    void waitEnter(const std::string& message = "")
    {
        if (!message.empty()) {
            std::cout << message;
        }

        std::string line;
        std::getline(std::cin, line);
    }

    // Neteja la terminal per mantenir el menu llegible.
    void clearScreen()
    {
        std::system("clear");
    }
};

BoticelliInterface* global_interface = NULL;

// Gestor de senyals per aturar el robot encara que l'usuari faci Ctrl+C.
void sigintHandler(int)
{
    std::cout << std::endl;
    std::cout << "Aturant Bot-ticelli de forma segura..." << std::endl;

    if (global_interface != NULL) {
        global_interface->shutdown();
    }

    ros::shutdown();
    _exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "boticelli_interface_node", ros::init_options::NoSigintHandler);

    BoticelliInterface interface_node;
    global_interface = &interface_node;

    signal(SIGINT, sigintHandler);
    signal(SIGTERM, sigintHandler);

    // Spinner separat: els callbacks ROS poden actualitzar flags mentre el fil principal
    // esta esperant una entrada de teclat.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    interface_node.run();

    interface_node.shutdown();

    return 0;
}
