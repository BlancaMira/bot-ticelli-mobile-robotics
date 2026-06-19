#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>

#include <iostream>
#include <string>
#include <limits>

class UserInterfaceNode
{
private:
    ros::NodeHandle nh_;

    ros::Subscriber contour_info_sub_;
    ros::Subscriber drawing_ready_sub_;

    ros::Publisher capture_pub_;
    ros::Publisher dimensions_pub_;
    ros::Publisher go_to_drawing_pub_;
    ros::Publisher start_drawing_pub_;

    bool contour_info_received_;
    bool drawing_ready_received_;
    bool drawing_started_;

    double bbox_width_px_;
    double bbox_height_px_;
    double aspect_ratio_;

    double safety_margin_;

public:
    UserInterfaceNode()
    {
        ros::NodeHandle private_nh("~");

        private_nh.param<double>("safety_margin", safety_margin_, 0.20);

        contour_info_received_ = false;
        drawing_ready_received_ = false;
        drawing_started_ = false;

        bbox_width_px_ = 0.0;
        bbox_height_px_ = 0.0;
        aspect_ratio_ = 1.0;

        contour_info_sub_ = nh_.subscribe(
            "/contour_info",
            10,
            &UserInterfaceNode::contourInfoCallback,
            this
        );

        drawing_ready_sub_ = nh_.subscribe(
            "/drawing_ready",
            10,
            &UserInterfaceNode::drawingReadyCallback,
            this
        );

        capture_pub_ = nh_.advertise<std_msgs::Bool>("/capture_contour", 10);

        dimensions_pub_ = nh_.advertise<geometry_msgs::Vector3>(
            "/drawing_dimensions",
            10,
            true
        );

        go_to_drawing_pub_ = nh_.advertise<std_msgs::Bool>(
            "/go_to_drawing",
            10,
            true
        );

        start_drawing_pub_ = nh_.advertise<std_msgs::Bool>(
            "/start_drawing",
            10,
            true
        );
    }

    void contourInfoCallback(const geometry_msgs::Vector3::ConstPtr& msg)
    {
        bbox_width_px_ = msg->x;
        bbox_height_px_ = msg->y;
        aspect_ratio_ = msg->z;

        if (aspect_ratio_ <= 0.0) {
            aspect_ratio_ = 1.0;
        }

        contour_info_received_ = true;
    }

    void drawingReadyCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (msg->data && !drawing_started_) {
            drawing_ready_received_ = true;
        }
    }

    void run()
    {
        ros::Rate rate(10);

        printHeader();

        std::cout << "PAS 1 - Captura de la figura" << std::endl;
        std::cout << "Col·loca el full davant la camera." << std::endl;
        std::cout << "Quan el contorn sigui correcte, prem ENTER." << std::endl;
        std::cout << std::endl;

        waitForEnter();

        publishCaptureCommand();

        std::cout << "Captura enviada. Esperant dades del contorn..." << std::endl;

        waitForContourInfo(rate);

        if (!contour_info_received_) {
            std::cout << "ERROR: no s'ha rebut /contour_info." << std::endl;
            return;
        }

        double target_width = askTargetWidth();
        double target_height = target_width / aspect_ratio_;

        publishDimensions(target_width, target_height);

        std::cout << std::endl;
        std::cout << "Trajectoria generada amb mida aproximada:" << std::endl;
        std::cout << " - Amplada: " << target_width << " m" << std::endl;
        std::cout << " - Alcada:  " << target_height << " m" << std::endl;
        std::cout << std::endl;

        std::cout << "PAS 2 - Anar al punt inicial del dibuix" << std::endl;
        std::cout << "Revisa a RViz que la trajectoria esta ben situada." << std::endl;
        std::cout << "Prem ENTER per enviar el robot a un punt de la trajectoria." << std::endl;
        std::cout << std::endl;

        waitForEnter();

        publishGoToDrawing();

        std::cout << "Ordre enviada. El robot anira fins a un punt inicial del dibuix." << std::endl;
        std::cout << "Esperant que el navegador confirmi que ja ha arribat..." << std::endl;
        std::cout << std::endl;

        waitForDrawingReady(rate);

        if (!drawing_ready_received_) {
            std::cout << "ERROR: no s'ha rebut /drawing_ready." << std::endl;
            return;
        }

        std::cout << std::endl;
        std::cout << "============================================" << std::endl;
        std::cout << "El robot ja esta al punt inicial del dibuix." << std::endl;
        std::cout << "Baixa el retolador per començar a dibuixar!" << std::endl;
        std::cout << "Quan estigui preparat, prem ENTER." << std::endl;
        std::cout << "============================================" << std::endl;
        std::cout << std::endl;

        waitForEnter();

        publishStartDrawing();

        drawing_started_ = true;

        std::cout << "Ordre d'inici enviada. El robot comença a dibuixar." << std::endl;
        std::cout << std::endl;
    }

    void printHeader()
    {
        std::cout << std::endl;
        std::cout << "=========================================" << std::endl;
        std::cout << " SafeSketchBot - Interficie d'usuari" << std::endl;
        std::cout << "=========================================" << std::endl;
        std::cout << std::endl;
    }

    void waitForEnter()
    {
        std::string line;
        std::getline(std::cin, line);
    }

    void publishCaptureCommand()
    {
        std_msgs::Bool msg;
        msg.data = true;

        capture_pub_.publish(msg);
        ros::Duration(0.3).sleep();
    }

    void waitForContourInfo(ros::Rate& rate)
    {
        int max_iter = 80;

        for (int i = 0; i < max_iter && ros::ok(); ++i) {
            ros::spinOnce();

            if (contour_info_received_) {
                return;
            }

            rate.sleep();
        }
    }

    double askTargetWidth()
    {
        std::cout << std::endl;
        std::cout << "PAS 1.2 - Mida del dibuix" << std::endl;
        std::cout << "Selecciona la mida final:" << std::endl;
        std::cout << "  1 - Petita  (0.50 m d'amplada)" << std::endl;
        std::cout << "  2 - Mitjana (0.80 m d'amplada)" << std::endl;
        std::cout << "  3 - Gran    (1.00 m d'amplada)" << std::endl;
        std::cout << "  4 - Manual" << std::endl;
        std::cout << std::endl;
        std::cout << "Opcio: ";

        int option = 0;
        std::cin >> option;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        if (option == 1) {
            return 0.50;
        }

        if (option == 2) {
            return 0.80;
        }

        if (option == 3) {
            return 1.00;
        }

        if (option == 4) {
            double width = 0.80;

            std::cout << "Amplada final en metres: ";
            std::cin >> width;
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

            if (width <= 0.0) {
                std::cout << "Valor invalid. S'utilitza 0.80 m." << std::endl;
                return 0.80;
            }

            return width;
        }

        std::cout << "Opcio invalida. S'utilitza mida mitjana." << std::endl;
        return 0.80;
    }

    void publishDimensions(double target_width, double target_height)
    {
        geometry_msgs::Vector3 dim_msg;

        dim_msg.x = target_width;
        dim_msg.y = target_height;
        dim_msg.z = safety_margin_;

        dimensions_pub_.publish(dim_msg);
        ros::Duration(0.3).sleep();
    }

    void publishGoToDrawing()
    {
        std_msgs::Bool msg;
        msg.data = true;

        go_to_drawing_pub_.publish(msg);
        ros::Duration(0.3).sleep();
    }

    void waitForDrawingReady(ros::Rate& rate)
    {
        while (ros::ok() && !drawing_ready_received_) {
            ros::spinOnce();
            rate.sleep();
        }
    }

    void publishStartDrawing()
    {
        std_msgs::Bool msg;
        msg.data = true;

        start_drawing_pub_.publish(msg);
        ros::Duration(0.3).sleep();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "user_interface_node");

    UserInterfaceNode node;
    node.run();

    return 0;
}
