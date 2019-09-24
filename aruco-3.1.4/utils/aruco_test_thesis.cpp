#include "aruco.h"
#include "cvdrawingutils.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "aruco_test_thesis.h"
#include <assert.h>

struct   TimerAvrg {
    std::vector<double> times;
    size_t curr = 0, n; std::chrono::high_resolution_clock::time_point begin, end;   TimerAvrg(int _n = 30)
    {
        n = _n; times.reserve(n);
    }inline void start()
    {
        begin = std::chrono::high_resolution_clock::now();
    }inline void stop()
    {
        end = std::chrono::high_resolution_clock::now(); double duration = double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count())*1e-6; if (times.size() < n) times.push_back(duration); else { times[curr] = duration; curr++; if (curr >= times.size()) curr = 0; }
    }double getAvrg()
    {
        double sum = 0; for (auto t : times) sum += t; return sum / double(times.size());
    }
};

cv::Mat rot2euler(cv::Mat dest);
cv::Mat convert4x4to3x3(cv::Mat T);
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat cam0_id113x3);
//void rotationMatrixToEulerAngles(cv::Mat dest);

int main(int argc, char** argv)
{
    TimerAvrg fps_;
    std::string dictionaryStr_0 = "ALL_DICTS";
    std::string dictionaryStr_1 = "ALL_DICTS";
    float markerSize_0 = 0.2f;
    float markerSize_1 = 0.2f;
    aruco::CameraParameters camparams_0;
    aruco::CameraParameters camparams_1;
    aruco::MarkerDetector detector_0;
    aruco::MarkerDetector detector_1;
    camparams_0.readFromXMLFile("cam_0.yml");
    camparams_1.readFromXMLFile("cam_1.yml");
    detector_0.setDictionary(dictionaryStr_0, 0.0f);
    detector_1.setDictionary(dictionaryStr_1, 0.0f);
    cv::Mat image_0;
    cv::Mat image_1;
    image_0 = cv::imread("image_0.png", 1);
    image_1 = cv::imread("image_1.png", 1);
    std::vector<aruco::Marker> markers_0;
    std::vector<aruco::Marker> markers_1;

    fps_.start();
    markers_0 = detector_0.detect(image_0, camparams_0, markerSize_0);
    markers_1 = detector_1.detect(image_1, camparams_1, markerSize_1);
    fps_.stop();

    /* check the speed by calculating the mean speed of all iterations */
    std::cout << "\rTime detection=" << fps_.getAvrg() * 1000 << " milliseconds " << std::endl;
    std::cout << "\r nmarkers = " << markers_0.size() << " images resolution = " << image_0.size() << std::endl;
    std::cout << "\r nmarkers = " << markers_1.size() << " images resolution = " << image_1.size() << std::endl;

    /* The first camera */
    auto candidates_0 = detector_0.getCandidates();
    for (auto cand : candidates_0)
        aruco::Marker(cand, -1).draw(image_0, cv::Scalar(255, 0, 255));

    for (unsigned int i = 0; i < markers_0.size(); i++) {
        std::cout << markers_0[i] << std::endl;
        markers_0[i].draw(image_0, cv::Scalar(0, 0, 255), 2, true);
    }

    /* draw a 3d cube in each marker if there is 3d info */
    if (camparams_0.isValid() && markerSize_0 > 0) {
        for (unsigned int i = 0; i < markers_0.size(); i++) {
            aruco::CvDrawingUtils::draw3dCube(image_0, markers_0[i], camparams_0);
            aruco::CvDrawingUtils::draw3dAxis(image_0, markers_0[i], camparams_0);
        }
    }

    /* The second camera */
    auto candidates_1 = detector_1.getCandidates();
    for (auto cand : candidates_1)
        aruco::Marker(cand, -1).draw(image_1, cv::Scalar(255, 0, 255));

    for (unsigned int i = 0; i < markers_1.size(); i++) {
        std::cout << markers_1[i] << std::endl;
        markers_1[i].draw(image_1, cv::Scalar(0, 0, 255), 2, true);
    }



    /* draw a 3d cube in each marker if there is 3d info */
    if (camparams_1.isValid() && markerSize_1 > 0) {
        for (unsigned int i = 0; i < markers_1.size(); i++) {
            aruco::CvDrawingUtils::draw3dCube(image_1, markers_1[i], camparams_1);
            aruco::CvDrawingUtils::draw3dAxis(image_1, markers_1[i], camparams_1);
        }
    }

    /* TODO: check if detected */
    cv::Mat T_cam0_id11 = markers_0[0].getTransformMatrix();

    //conver cam0_id11 to 3*3
    cv::Mat cam0_id113x3 = convert4x4to3x3(T_cam0_id11);
    cv::Vec3f xyz = rotationMatrixToEulerAngles(cam0_id113x3.t());
    std::cout << xyz;
    cv::Mat T_cam0_id17 = markers_0[1].getTransformMatrix();
    cv::Vec3f xyz17 = rotationMatrixToEulerAngles(T_cam0_id17);
    std::cout << xyz17;
    cv::Mat T_cam1_id17 = markers_1[0].getTransformMatrix();

    /* GT: dist btw cam and marker: 1710.77 mm , detection: marker wrt. camera */
    cv::Mat RmarkerOnTheCar = cv::Mat::eye(3, 3, CV_32FC1);
    cv::Mat TmarkerOnTheCar = cv::Mat::eye(4, 4, CV_32FC1);
    // TODO: put 0,90,0 from blender to obtain R, later T with the
    TmarkerOnTheCar.at<float>(0, 3) = 0.168206;
    TmarkerOnTheCar.at<float>(1, 3) = 1.42624;
    TmarkerOnTheCar.at<float>(2, 3) = 1.38777;
    cv::Mat T = TmarkerOnTheCar * T_cam0_id11.inv() * T_cam0_id17 * T_cam1_id17.inv();
    //camera on car location ground truth [m]
    float gx = -0.30904;
    float gy = 3.0766;
    float gz = 1.8331;
    float ex = std::abs(T.at<float>(0, 3) - gx);
    float ey = std::abs(T.at<float>(1, 3) - gy);
    float ez = std::abs(T.at<float>(2, 3) - gz);

    float dist = sqrt(T.at<float>(0, 3) * T.
        at<float>(0, 3) +
        T.at<float>(1, 3) * T.at<float>(1, 3) +
        T.at<float>(2, 3) * T.at<float>(2, 3));

    /*
    std::cout << "dest" << dest;
    cv::Mat Eulerangles_id17_cam_0 = cv::Mat(3, 3, CV_32FC1);
    cv::Mat dest = convert4x4to3x3(T_cam0_id17);
    Eulerangles_id17_cam_0 = rot2euler(dest);
    rotationMatrixToEulerAngles(dest);

    std::cout << Eulerangles_id17_cam_0;
    cv::Mat Degree_euler = Eulerangles_id17_cam_0 * 180 / CV_PI;
    std::cout << Degree_euler;
    cv::Mat Eulerangles_id11 = cv::Mat(3, 3, CV_32FC1);


     dest = convert4x4to3x3(T);
    Eulerangles_id11 = rot2euler(dest);
    std::cout << Eulerangles_id11; */
    return 0;

}

cv::Mat convert4x4to3x3(cv::Mat mat4x4)
{
    int width = mat4x4.cols;
    int height = mat4x4.rows;
    cv::Mat rota = cv::Mat(3, 3, CV_32FC1);
    for (int x = 0; x < height - 1; x++)
    {
        for (int y = 0; y < width - 1; y++)
        {

            rota.at<float>(x, y) = mat4x4.at<float>(x, y);

        }
    }
    return rota;
}

cv::Vec3f rotationMatrixToEulerAngles(cv::Mat R)
{

    //assert(isRotationMatrix(R));

    float sy = sqrt(R.at<float>(0, 0) * R.at<float>(0, 0) + R.at<float>(1, 0) * R.at<float>(1, 0));

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<float>(2, 1), R.at<float>(2, 2));
        y = atan2(-R.at<float>(2, 0), sy);
        z = atan2(R.at<float>(1, 0), R.at<float>(0, 0));
    }
    else
    {
        x = atan2(-R.at<float>(1, 2), R.at<float>(1, 1));
        y = atan2(-R.at<float>(2, 0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);



}
/*
 cv::Mat rot2euler(cv::Mat dest)
{
     //std::cout << dest ;

    cv::Mat euler(3, 1, CV_32FC1);
    double m00 = dest.at<float>(0, 0);
    double m02 = dest.at<float>(0, 2);
    double m10 = dest.at<float>(1, 0);
    double m11 = dest.at<float>(1, 1);
    double m12 = dest.at<float>(1, 2);
    double m20 = dest.at<float>(2, 0);
    double m22 = dest.at<float>(2, 2);
    double x, y, z;

    // Assuming the angles are in radians.
    if (m10 > 0.998) { // singularity at north pole
        x = 0;
        y = CV_PI / 2;
        z = atan2(m02, m22);
    }
    else
    {
        x = atan2(-m12, m11);
        y = asin(m10);
        z = atan2(-m20, m00);
    }

    euler.at<float>(0) = x;
    euler.at<float>(1) = y;
    euler.at<float>(2) = z;

    return euler;
}

 void rotationMatrixToEulerAngles(cv::Mat R)
 {
     std::cout << R;
     float sy = sqrt(R.at<float>(0, 0) * R.at<float>(0, 0) + R.at<float>(1, 0) * R.at<float>(1, 0));

     bool singular = sy < 1e-6; // If

     float x, y, z;
     if (!singular)
     {
         x = atan2(R.at<float>(2, 1), R.at<float>(2, 2));
         y = atan2(-R.at<float>(2, 0), sy);
         z = atan2(R.at<float>(1, 0), R.at<float>(0, 0));
     }
     else
     {
         x = atan2(-R.at<float>(1, 2), R.at<float>(1, 1));
         y = atan2(-R.at<float>(2, 0), sy);
         z = 0;
     }
     std::cout << "x" << x << "y" << y << "z" << z;




 }
 */