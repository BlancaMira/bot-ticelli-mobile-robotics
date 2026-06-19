#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "Ús: vision_offline ruta_imatge" << std::endl;
        return 1;
    }

    cv::Mat img = cv::imread(argv[1], cv::IMREAD_COLOR);

    if (img.empty()) {
        std::cout << "No s'ha pogut carregar la imatge." << std::endl;
        return 1;
    }

    // 1. Convertir a grisos
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    // 2. Suavitzat
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

    // 3. Binarització
    cv::Mat mask;
    cv::threshold(gray, mask, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

    // 4. Neteja morfològica
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

    // 5. Buscar contorns
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        std::cout << "No s'ha detectat cap contorn." << std::endl;
        return 1;
    }

    // 6. Escollir el contorn més gran
    int best_idx = 0;
    double best_area = 0.0;

    for (int i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > best_area) {
            best_area = area;
            best_idx = i;
        }
    }

    std::cout << "Àrea del millor contorn: " << best_area << std::endl;

    // 7. Simplificar contorn
    std::vector<cv::Point> approx;
    double epsilon = 0.01 * cv::arcLength(contours[best_idx], true);
    cv::approxPolyDP(contours[best_idx], approx, epsilon, true);

    std::cout << "Contorn original amb " << contours[best_idx].size() << " punts." << std::endl;
    std::cout << "Contorn simplificat amb " << approx.size() << " punts." << std::endl;

    // 8. Dibuixar contorns
    cv::Mat contour_img = img.clone();

    // contorn original en verd
    cv::drawContours(contour_img,
                     contours,
                     best_idx,
                     cv::Scalar(0, 255, 0),
                     2);

    // contorn simplificat en vermell
    cv::drawContours(contour_img,
                     std::vector<std::vector<cv::Point>>{approx},
                     -1,
                     cv::Scalar(0, 0, 255),
                     2);

    // 9. Convertir màscara a BGR per poder concatenar
    cv::Mat mask_bgr;
    cv::cvtColor(mask, mask_bgr, cv::COLOR_GRAY2BGR);

    // 10. Redimensionar totes les imatges a la mateixa mida
    int display_width = 400;
    int display_height = 400;

    cv::Mat img_resized, mask_resized, contour_resized;

    cv::resize(img, img_resized, cv::Size(display_width, display_height));
    cv::resize(mask_bgr, mask_resized, cv::Size(display_width, display_height));
    cv::resize(contour_img, contour_resized, cv::Size(display_width, display_height));

    // 11. Afegir títols a cada imatge
    cv::putText(img_resized, "Original", cv::Point(20, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

    cv::putText(mask_resized, "Mascara", cv::Point(20, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

    cv::putText(contour_resized, "Contorn", cv::Point(20, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);

    // 12. Ajuntar les tres imatges en una sola finestra
    cv::Mat combined;
    cv::hconcat(std::vector<cv::Mat>{img_resized, mask_resized, contour_resized}, combined);

    // 13. Mostrar resultat
    cv::imshow("Resultat visio", combined);
    cv::waitKey(0);

    return 0;
}

