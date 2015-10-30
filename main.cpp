/* Copyright
 * Copyright (c) 2012-2014 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

/* Coautores
   Marco Ramirez
   Juan Carlos Aguilera Perez
   Aurelio Puebla
   Fernando Lopez
*/

/* Aspectos de revision
 * Confirmar se mantega dentro del carril
 * Revisar Caidas
 * Agregar que se detenga a los 3 mins
 * Crear rutina de apagado y encendido
*/

/* Reglas
 * Distancia total 5m
 * Franja inicio 18 cm
 * Franja final 30 cm y de color rojo
 * Max tiempo fuera de carril = 3 segs
 * Se considera fuera de carril con 1 pie fuera de la linea
 * Si algo sucede el robot tiene que poder responder autonomamente (levantar)
*/

#include <iostream>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>

using namespace std;
using namespace AL;
using namespace cv;

AL::ALValue walk();
double getOrientation(const vector<Point> &pts, Mat &img);
double linearVelocity(double theta);
double angularVelocity(double theta);
void drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale);

int main(int argc, char *argv[]) {
    string ip;

    ip = argv[1]; // IP del robot
    AL::ALRobotPostureProxy posture(ip, 9559); // Posture Proxy
    AL::ALMotionProxy motion(ip, 9559); // Motion Proxy
    cout << "IP: " << ip << endl;

    posture.goToPosture("Crouch", 0.5);
    posture.goToPosture("StandInit", 0.5);
    cout << "Stand" << endl;

    // ************************************* RUN *************************************
    Mat src;
    Mat src_gray;
    Point2f punto;
    Point2f puntoMax;
    vector<vector<Point> > contoursClean;
    RNG rng(12345);
    AL::ALVideoDeviceProxy cameraProxy(ip, 9559);

    bool DEBUG = true;  // Bandera para mostrar mensajes
    bool LOCAL = false; // Bandera para el tipo de ejecucion (local o remota)
    bool NAO = true;
    int area;
    int length;
    int thresh = 110;
    int contFrames = 0; // Frames leidos al momento
    int umbral = 60;    // Parte del frame que no se tomará en cuenta
    double orientation;    // Angulo de la línea detectada
    char key = 'x';

    // Load source image and convert it to gray
    VideoCapture cap(1);    // Class for video capturing from video files or cameras
    const string clientName = cameraProxy.subscribe("test", AL::kQVGA, AL::kBGRColorSpace, 30); // Subscribe to ALVideoDevice
    // ************************************* --- *************************************

    while(key != 27) {
        // ************************************* RUN *************************************
        if (NAO) {
            // Get image from NAO
            cameraProxy.setActiveCamera(AL::kBottomCamera); // Conect to bottom camera
            cameraProxy.setResolution("test", 1);   // Image of 320*240px

            // Create an cv::Mat header to wrap into an opencv image.
            Mat imgHeader = cv::Mat(cv::Size(320, 240), CV_8UC3);

            // Retrieves the latest image from the video resource
            ALValue img = cameraProxy.getImageRemote(clientName);

            // Access the image buffer (6th field) and assign it to the opencv image container
            imgHeader.data = (uchar*) img[6].GetBinary();

            // Tells to ALVideoDevice that it can give back the image buffer to the driver.
            // Optional after a getImageRemote but MANDATORY after a getImageLocal.
            cameraProxy.releaseImage(clientName);

            // Display the iplImage on screen.
            src = imgHeader.clone();

            if(!LOCAL)
                imshow("src", src);
        }
        else {
            cap >> src;
        }

        if (contFrames == 1) {
            contFrames = 0;

            // Convert image to gray and blur it
            cvtColor(src, src_gray, CV_BGR2GRAY);
            blur(src_gray, src_gray, Size(3,3));

            if(!LOCAL)
                imshow("src", src);

            Mat canny_output;
            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;

            // Detect edges using canny
            Canny(src_gray, canny_output, thresh, thresh*2, 3);

            // Find contours
            findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

            // Get the moments
            vector<Moments> mu(contours.size());

            for( int i = 0; i < contours.size(); i++ )
                mu[i] = moments(contours[i], false );

            // Get the mass centers
            vector<Point2f> mc( contours.size());

            for( int i = 0; i < contours.size(); i++ )
                mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );

            // Eliminate contours without area
            contoursClean.clear();
            int indMax = 0;
            int lengthMax = 0;

            for(int i = 0; i < contours.size(); i++){
                area = mu[i].m00;
                length = arcLength(contours[i], true);
                punto = mc[i];

                if(area != 0 && length > 200 && punto.x > 0 && punto.y > 0)
                    contoursClean.push_back(contours.at(i));
            }

            if(contoursClean.size() != 0) {
                // Get moments and mass for new vector
                vector<Moments> muClean(contoursClean.size() );
                for(int i = 0; i < contoursClean.size(); i++)
                    muClean[i] = moments(contoursClean[i], false);

                // Get the mass centers
                vector<Point2f> mcClean( contoursClean.size());

                for( int i = 0; i < contoursClean.size(); i++ )
                    mcClean[i] = Point2f( muClean[i].m10/muClean[i].m00 , muClean[i].m01/muClean[i].m00 );

                for(int i = 0; i < contoursClean.size(); i++){
                    punto = mcClean[i];
                    length = arcLength(contoursClean[i], true);
                }

                // Find the longest
                for(int i = 0; i < contoursClean.size(); i++){
                    length = arcLength(contoursClean[i], true);
                    lengthMax = arcLength(contoursClean[indMax], true);

                    if(i > 0){
                        if(length  > lengthMax)
                            indMax = i;
                    }else
                        indMax = 0;
                }

                // Draw contours
                Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3);

                Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                drawContours( drawing, contoursClean, indMax, color, 2, 8, hierarchy, 0, Point() );
                circle( drawing, mcClean[indMax], 4, color, 5, 8, 0 );

                // Calculate the angle of the line
                orientation = getOrientation(contoursClean[indMax], drawing);

                // Show in a window
                if(!LOCAL) {
                    namedWindow("Contours", CV_WINDOW_AUTOSIZE);
                    imshow("Contours", drawing);
                }

                if(contoursClean.size() != 0) {
                    puntoMax = mcClean[indMax];
                    lengthMax = arcLength(contoursClean[indMax], true);

                    if(!LOCAL) {
                        // Dibujar cuadrícula
                        line(drawing, Point(260,0), Point(260, drawing.rows), Scalar(255,255,255));
                        line(drawing, Point(umbral,0), Point(umbral, drawing.rows), Scalar(255,255,255));
                        line(drawing, Point((drawing.cols/2),0), Point((drawing.cols/2), drawing.rows), Scalar(255,255,255));
                        line(drawing, Point(0,120), Point(320,120), Scalar(255,255,255));
                        imshow("Contours", drawing);
                    }
                }
                else { // Sigue derecho
                    orientation = 90.0;
                }
            }
            else { // Sigue derecho
                orientation = 90.0;
            }

            if(DEBUG){
                cout << "VelLin: " << linearVelocity(orientation) << endl;
                cout << "VelAng: " << angularVelocity(orientation) << endl;
                cout << "Punto Max: " << puntoMax.x << " Y: " << puntoMax.y << endl;
                cout << "Umbral: " << umbral << endl;
                cout << "Theta: " << orientation << endl;
                cout << "--------------------------------" << endl;
            }
        }

        contFrames++;
        key = waitKey(10);

        motion.move(linearVelocity(orientation), 0, angularVelocity(orientation),walk());
        for(int i = 0; i < 250000; i++);
    }

    cameraProxy.unsubscribe(clientName);

    motion.stopMove();
    cout << "Stop" << endl;
    posture.goToPosture("Crouch", 0.5);
    motion.setStiffnesses("Body", 0);

    return 0;
}

double getOrientation(const vector<Point> &pts, Mat &img){
    //Construct a buffer used by the pca analysis
    int sz = static_cast<int>(pts.size());
    Mat data_pts = Mat(sz, 2, CV_64FC1);
    for(int i = 0; i < data_pts.rows; ++i){
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }
    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
    //Store the center of the object
    Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                      static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
    //Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);
    for(int i = 0; i < 2; ++i){
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }
    // Draw the principal components
    circle(img, cntr, 3, Scalar(255, 0, 255), 2);
    Point p1 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
    Point p2 = cntr - 0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
    drawAxis(img, cntr, p1, Scalar(0, 255, 0), 1);
    drawAxis(img, cntr, p2, Scalar(255, 255, 0), 5);
    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
    double degrees = angle * 180 / CV_PI; // convert radians to degrees (0-180 range)
    degrees = degrees < 0 ? degrees + 180 : degrees;
    return degrees;

}

void drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale = 0.2){
    double angle;
    double hypotenuse;
    angle = atan2( (double) p.y - q.y, (double) p.x - q.x ); // angle in radians
    hypotenuse = sqrt( (double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
    // Here we lengthen the arrow by a factor of scale
    q.x = (int) (p.x - scale * hypotenuse * cos(angle));
    q.y = (int) (p.y - scale * hypotenuse * sin(angle));
    line(img, p, q, colour, 1, CV_AA);
    // create the arrow hooks
    p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
    line(img, p, q, colour, 1, CV_AA);
    p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
    line(img, p, q, colour, 1, CV_AA);
}

//v = vmax * e^(-k*abs(theta - 90))
double linearVelocity(double theta){
    const double vMax = 0.85; //0.85
    const double k1 = 1.0 / 40; // 1/40
    const double k2 = 1.0 / 15; // 1/ 15
    return vMax * exp(-(theta > 90 ? k2 : k1) * abs(theta - 90));
    //return vMax * (1 - (abs(theta - 90) / 90));
}
//w = wmax * ( 1 - e^(-k*abs(theta - 90)))*N if (theta > 90) (N = -1) else (N = 1)
double angularVelocity(double theta){
    const double wMax = 0.25;
     //K1 right to left correction
    const double k1 = 1.0 / 50; // 1 / 50
     //K2 left to right correction
    const double k2 = 1.0 / 20;
    return pow(-1, theta > 90) * (wMax * (1 - exp(-(theta > 90 ? k2 : k1) * abs(theta - 90))));
    //return wMax * -((theta - 90) / 90);
}

AL::ALValue walk(){
   return  AL::ALValue::array(AL::ALValue::array("MaxStepX",0.08),AL::ALValue::array("MaxStepY",0.14),
                             AL::ALValue::array("MaxStepTheta",0.4),AL::ALValue::array("MaxStepFrequency",0.5), //Frec 0.5
                             AL::ALValue::array("StepHeight",0.04),AL::ALValue::array("TorsoWx",0.0),
                             AL::ALValue::array("TorsoWy",0));
}


