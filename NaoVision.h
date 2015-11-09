#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <alvision/alimage.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alvisiondefinitions.h>

using namespace std;
using namespace AL;
using namespace cv;

class NaoVision {
public:
    NaoVision(const string ip, const int port, bool local);
    Mat getImage();
    Mat getImageTop();
    Mat getImageBot();
    double calculateAngleToBlackLine();
    void unsubscribe();
    void calibracionColorCamara();
    bool filtroColor(Mat imgOriginal);
    void setSourceMat(Mat source);
    Mat getSourceMat();

private:
    RNG rng;
    Mat src;
    Mat src_gray;
    Point2f punto;
    Point2f puntoMax;
    vector<vector<Point> > contoursClean;
    AL::ALVideoDeviceProxy cameraProxy;

    bool local;             // Flag for the execution type (local or remote).
    int area;
    int port;
    int length;
    int thresh;
    int umbral;             // Part of the frame that will not be taken into account.
    double angleToALine;    // Detected angle line.
    string ip;
    string clientName;
    string parameterClientName;

    // Variables that allow us to detect different colors.
    int iLowH;
    int iHighH;
    int iLowS;
    int iHighS;
    int iLowV;
    int iHighV;

    double getAngleDegrees(const vector<Point> &pts, Mat &img);
    void drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale);
};
