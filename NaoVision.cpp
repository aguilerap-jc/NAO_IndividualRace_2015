#include "NaoVision.h"

NaoVision::NaoVision(const string ip, const int port, bool localFlag): cameraProxy(ip, port), rng(12345) {
    lineFlag = 0;   // For default, the Nao is on the middle.
    thresh = 110;
    umbral = 60;
    this->ip = ip;
    this->port = port;
    this->local = local;
    this->parameterClientName = "test";
    this->clientName = cameraProxy.subscribe(parameterClientName, AL::kQVGA, AL::kBGRColorSpace, 30); // Subscribe to ALVideoDevice
}

// Get image from NAO.
Mat NaoVision::getImage() {
    // Connect to bottom camera.
    cameraProxy.setActiveCamera(AL::kBottomCamera);

    // Image of 320*240 px.
    cameraProxy.setResolution(parameterClientName, 1);

    // Create an cv::Mat header to wrap into an opencv image.
    Mat imgHeader = Mat(cv::Size(320, 240), CV_8UC3);

    // Retrieves the latest image from the video resource.
    ALValue img = cameraProxy.getImageRemote(clientName);

    // Access the image buffer (6th field) and assign it to the opencv image container.
    imgHeader.data = (uchar*)img[6].GetBinary();

    // Tells to ALVideoDevice that it can give back the image buffer to the driver.
    // Optional after a getImageRemote but MANDATORY after a getImageLocal.
    cameraProxy.releaseImage(clientName);

    // Display the iplImage on screen.
    src = imgHeader.clone();

    return src;
}

// Process an image containing a line and return the angle with respect to NAO.
double NaoVision::calculateAngleToBlackLine() {
    // Convert image to gray and blur it.
    cvtColor(src, src_gray, CV_BGR2GRAY);
    blur(src_gray, src_gray, Size(3,3));

    if(!local)
        imshow("src", src);

    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    // Detect edges using canny.
    Canny(src_gray, canny_output, thresh, thresh * 2, 3);

    // Find contours.
    findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    // Get the moments.
    vector<Moments> mu(contours.size());

    for(int i = 0; i < contours.size(); i++)
        mu[i] = moments(contours[i], false);

    // Get the mass centers.
    vector<Point2f> mc( contours.size());

    for(int i = 0; i < contours.size(); i++)
        mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);

    // Eliminate contours without area.
    contoursClean.clear();
    int indMax = 0;
    int lengthMax = 0;

    for(int i = 0; i < contours.size(); i++) {
        area = mu[i].m00;
        length = arcLength(contours[i], true);
        punto = mc[i];

        if(area != 0 && length > 200 && punto.x > 0 && punto.y > 0)
            contoursClean.push_back(contours.at(i));
    }

    if(contoursClean.size() != 0) {
        // Get moments and mass for new vector.
        vector<Moments> muClean(contoursClean.size());
        for(int i = 0; i < contoursClean.size(); i++)
            muClean[i] = moments(contoursClean[i], false);

        // Get the mass centers.
        vector<Point2f> mcClean( contoursClean.size());

        for(int i = 0; i < contoursClean.size(); i++)
            mcClean[i] = Point2f( muClean[i].m10/muClean[i].m00 , muClean[i].m01/muClean[i].m00 );

        for(int i = 0; i < contoursClean.size(); i++) {
            punto = mcClean[i];
            length = arcLength(contoursClean[i], true);
        }

        // Find the longest.
        for(int i = 0; i < contoursClean.size(); i++) {
            length = arcLength(contoursClean[i], true);
            lengthMax = arcLength(contoursClean[indMax], true);

            if(i > 0) {
                if(length  > lengthMax)
                    indMax = i;
            } else
                indMax = 0;
        }

        // Draw contours.
        Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3);

        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contoursClean, indMax, color, 2, 8, hierarchy, 0, Point());
        circle( drawing, mcClean[indMax], 4, color, 5, 8, 0 );

        // Calculate the angle of the line.
        angleToALine = getAngleDegrees(contoursClean[indMax], drawing);

        if(angleToALine >= 80 && angleToALine <= 100 && lineFlag == 1)
            angleToALine = 180;  // Nao is on the left black line, so we have to go to the right.
        else if (angleToALine >= 80 && angleToALine <= 100 && lineFlag == 2)
            angleToALine = 0;    // Nao is on the right black line, so we have to go to the left.
        else if (angleToALine < 90)
            lineFlag = 2;       // Nao is on the right side.
        else if (angleToALine > 90)
            lineFlag = 1;       // Nao is on the left side.

        // Show in a window.
        if(!local) {
            namedWindow("Contours", CV_WINDOW_AUTOSIZE);
            imshow("Contours", drawing);
        }

        if(contoursClean.size() != 0) {
            puntoMax = mcClean[indMax];
            lengthMax = arcLength(contoursClean[indMax], true);

            if(!local) {
                // Draw grid.
                line(drawing, Point(260,0), Point(260, drawing.rows), Scalar(255,255,255));
                line(drawing, Point(umbral,0), Point(umbral, drawing.rows), Scalar(255,255,255));
                line(drawing, Point((drawing.cols/2),0), Point((drawing.cols/2), drawing.rows), Scalar(255,255,255));
                line(drawing, Point(0,120), Point(320,120), Scalar(255,255,255));
                imshow("Contours", drawing);
            }
        }
        else { // Go straight.
            lineFlag = 0;
            angleToALine = 90.0;
        }
    }
    else { // Go straight.
        angleToALine = 90.0;
    }

    return angleToALine;
}

void NaoVision::unsubscribe() {
   cameraProxy.unsubscribe(clientName);
}

// Calculate the angle in degrees of a certain line.
double NaoVision::getAngleDegrees(const vector<Point> &pts, Mat &img) {
    // Construct a buffer used by the pca analysis.
    int sz = static_cast<int>(pts.size());
    Mat data_pts = Mat(sz, 2, CV_64FC1);

    for(int i = 0; i < data_pts.rows; ++i) {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }

    // Perform PCA analysis.
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);

    // Store the center of the object.
    Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                      static_cast<int>(pca_analysis.mean.at<double>(0, 1)));

    // Store the eigenvalues and eigenvectors.
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);

    for(int i = 0; i < 2; ++i) {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }

    // Draw the principal components.
    circle(img, cntr, 3, Scalar(255, 0, 255), 2);
    Point p1 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
    Point p2 = cntr - 0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
    drawAxis(img, cntr, p1, Scalar(0, 255, 0), 1);
    drawAxis(img, cntr, p2, Scalar(255, 255, 0), 5);

    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x);     // Angle in radians.
    double degrees = angle * 180 / CV_PI;                       // Convert radians to degrees (0-180 range).
    degrees = degrees < 0 ? degrees + 180 : degrees;

    return degrees;
}

// The principal components are drawn in lines.
void NaoVision::drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale = 0.2) {
     double angle = atan2( (double) p.y - q.y, (double) p.x - q.x );     // Angle in radians.
     double hypotenuse = sqrt( (double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));

    // Here we lengthen the arrow by a factor of scale.
    q.x = (int) (p.x - scale * hypotenuse * cos(angle));
    q.y = (int) (p.y - scale * hypotenuse * sin(angle));
    line(img, p, q, colour, 1, CV_AA);

    // Create the arrow hooks.
    p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
    line(img, p, q, colour, 1, CV_AA);
    p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
    line(img, p, q, colour, 1, CV_AA);
}

// Getters and setters
void NaoVision::setSourceMat(Mat source) {
    src = source;
}

Mat NaoVision::getSourceMat() {
    return src;
}
