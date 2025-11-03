
#include "camera.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <unistd.h>

using namespace cv;
using namespace std;

Camera::Camera()
{
    m_fps = 30;
}

bool Camera::open(std::string filename)
{
    m_fileName = filename;

    istringstream iss(filename);
    int devid;
    bool isOpen;

    if (!(iss >> devid))
    {
        isOpen = m_cap.open(filename);
    }
    else
    {
        isOpen = m_cap.open(devid);
    }

    if (!isOpen)
    {
        cerr << "Unable to open video file." << endl;
        return false;
    }

    m_fps = m_cap.get(cv::CAP_PROP_FPS);
    if (m_fps == 0)
        m_fps = 30;

    return true;
}

struct Car {
    int id;
    cv::Rect bbox;
    cv::Point center;
    int framesSinceSeen;
    bool countedGauche;
    bool countedDroite;
};


void Camera::play(double speed)
{
    bool isReading = true;

    int timeToWait;
    if(speed <= 0.0){
        timeToWait = 1;
        std::cout << "Mode vitesse Max" << std::endl;
    }
    else {
        timeToWait = 1000 / (m_fps * speed);
        if (timeToWait < 1) timeToWait = 1;
    }

    int threshold1 = 34;
    int threshold2 = 350;
    cv::Mat background, bg_gris, l_frame, frame2, b_frame, After_canny;
    std::vector<cv::Vec4i> lines;
    int nbGauche = 0, nbDroite = 0, nbTotal = 0;

    isReading = m_cap.read(m_frame);
    isReading = m_cap.read(background);

    static std::vector<Car> trackedCars; // persistent between frames
    static int nextCarID = 1;

    while (isReading)
    {
        isReading = m_cap.read(m_frame);
        if (!isReading) break;

        cv::Size dim = m_frame.size();
        int h = dim.height;
        int w = dim.width;

        m_frame.copyTo(l_frame);

        // Initialize Hough lines from first frame
        if (lines.empty())
        {
            cv::cvtColor(background, bg_gris, cv::COLOR_BGR2GRAY);
            cv::Canny(bg_gris, After_canny, threshold1, threshold2, 3);
            cv::HoughLinesP(After_canny, lines, 1, CV_PI / 180, 40, 123, 10);
        }
        imshow("Canny", After_canny);
        // Draw extended lines
        for (size_t i = 0; i < lines.size(); i++)
        {
            cv::Vec4i l = lines[i];
            double a = 0.0, b = 0.0;
            int x1, x2;

            if (l[0] - l[2] != 0)
            {
                a = (double)(l[1] - l[3]) / (double)(l[0] - l[2]);
                if (a != 0.0) {
                    b = l[1] - a * l[0];
                    x1 = static_cast<int>((-b) / a);
                    x2 = static_cast<int>((h - b) / a);
                }
                else {
                    b = (double)l[1];
                    x1 = 0;
                    x2 = w;
                }
            }
            else {
                x1 = l[0];
                x2 = l[0];
            }
            cv::line(l_frame, cv::Point(x1, 0), cv::Point(x2, h), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        }

        // Background subtraction and preprocessing
        cv::absdiff(m_frame, background, frame2);
        imshow("Juste les voitures",frame2);
        cv::cvtColor(frame2, frame2, cv::COLOR_BGR2GRAY);
        cv::threshold(frame2, b_frame, 50, 255, cv::THRESH_BINARY);
        imshow("Seuil", b_frame);
        // Morphological operations
        cv::dilate(b_frame, b_frame, cv::Mat(), cv::Point(-1, -1), 4);
        cv::erode(b_frame, b_frame, cv::Mat(), cv::Point(-1, -1), 4);
        imshow("Fermeture", b_frame);
        cv::erode(b_frame, b_frame, cv::Mat(), cv::Point(-1, -1), 3);
        cv::dilate(b_frame, b_frame, cv::Mat(), cv::Point(-1, -1), 3);
        imshow("Ouverture", b_frame);

        // Find contours
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(b_frame, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<std::vector<cv::Point> > contours_poly(contours.size());

        // Define detection zones
    
        cv::Rect rectangleG(0, 100, w / 2, 8);
        cv::Rect rectangleD(w / 2, 140, w / 2, 8);


        cv::rectangle(l_frame, rectangleG.tl(), rectangleG.br(), cv::Scalar(255, 0, 0), 2);
        cv::rectangle(l_frame, rectangleD.tl(), rectangleD.br(), cv::Scalar(255, 0, 0), 2);


        // --- Car detection and tracking ---
        for (size_t i = 0; i < contours.size(); i++) {
            cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
            cv::Rect bbox = cv::boundingRect(cv::Mat(contours_poly[i]));
            cv::Point center(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);

            bool isNewCar = true;

            for (size_t j = 0; j < trackedCars.size(); j++) {
                Car &car = trackedCars[j];

                // Compute distance between centers
                double dx = center.x - car.center.x;
                double dy = center.y - car.center.y;
                double dist = sqrt(dx * dx + dy * dy);

                // Compute IoU (intersection over union)
                cv::Rect intersection = car.bbox & bbox;
                double iou = 0.0;
                if (intersection.area() > 0) {
                    iou = double(intersection.area()) / double(car.bbox.area() + bbox.area() - intersection.area());
                }

                // If distance is small OR IoU is large, it's the same car
                double distThreshold = std::max(30.0, std::max(car.bbox.width, car.bbox.height) / 2.0);
                if (dist < distThreshold || iou > 0.005) {
                    car.bbox = bbox;
                    car.center = center;
                    car.framesSinceSeen = 0;
                    isNewCar = false;
                    break;
                }
            }

            if (isNewCar) {
                Car newCar;
                newCar.id = nextCarID++;
                newCar.bbox = bbox;
                newCar.center = center;
                newCar.framesSinceSeen = 0;
                newCar.countedGauche = false;
                newCar.countedDroite = false;
                trackedCars.push_back(newCar);
            }

            cv::rectangle(l_frame, bbox, cv::Scalar(0, 255, 0), 2);
        }


        // --- Count cars when entering zones ---
        for (size_t i = 0; i < trackedCars.size(); i++) {
            Car &car = trackedCars[i];

            // Count left lane if bounding box intersects rectangle
            if (!car.countedGauche && (car.bbox & rectangleG).area() > 0) {
                nbGauche++;
                car.countedGauche = true;
            }

            // Count right lane if bounding box intersects rectangle
            if (!car.countedDroite && (car.bbox & rectangleD).area() > 0) {
                nbDroite++;
                car.countedDroite = true;
            }

            // Increment frames since last seen
            car.framesSinceSeen++;
        }


        // Remove old cars
        for (size_t i = 0; i < trackedCars.size();) {
            if (trackedCars[i].framesSinceSeen > 15) {
                trackedCars.erase(trackedCars.begin() + i);
            } else i++;
        }

        // Display car IDs
        for (size_t i = 0; i < trackedCars.size(); i++) {
            cv::putText(l_frame, "ID:" + std::to_string(trackedCars[i].id),
                        trackedCars[i].center, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(255, 255, 0), 1);
        }

        // Display counts
        cv::putText(l_frame, "Nombre de voitures voie de gauche : " + std::to_string(nbGauche),
                    cv::Point(230, 190), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1.5);
        cv::putText(l_frame, "Nombre de voitures voie de droite : " + std::to_string(nbDroite),
                    cv::Point(230, 210), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1.5);

        nbTotal = nbGauche + nbDroite;
        cv::putText(l_frame, "Total : " + std::to_string(nbTotal),
                    cv::Point(350, 280), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1.5);

        cv::imshow("Final video", l_frame);

        if (cv::waitKey(timeToWait) % 256 == 27) {
            std::cerr << "Stopped by user" << std::endl;
            break;
        }
    }

    printf("Nombre total : %d \n", nbTotal);
}

bool Camera::close()
{
    m_cap.release();
    destroyAllWindows();
    usleep(100000);
    return true;
}
