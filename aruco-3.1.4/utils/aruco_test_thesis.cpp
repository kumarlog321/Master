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
//#include "aruco_test_thesis.h"
#include <assert.h>

cv::Mat Rx(float angle)
{
    /* Calculate rotation about x axis */
    cv::Mat R_x = (cv::Mat_<float>(3, 3) <<
        1, 0, 0,
        0, cos(angle), -sin(angle),
        0, sin(angle), cos(angle)
        );
    return R_x;
}

cv::Mat Ry(float angle)
{
    /* Calculate rotation about y axis */
    cv::Mat R_y = (cv::Mat_<float>(3, 3) <<
        cos(angle), 0, sin(angle),
        0, 1, 0,
        -sin(angle), 0, cos(angle)
        );
    return R_y;
}

cv::Mat Rz(float angle)
{
    /* Calculate rotation about z axis */
    cv::Mat R_z = (cv::Mat_<float>(3, 3) <<
        cos(angle), -sin(angle), 0,
        sin(angle), cos(angle), 0,
        0, 0, 1);
    return R_z;
}


float deg2rad(float theta)
{
    return (theta * 0.01745329251994329576923690768489);
}

cv::Mat setR(float angle_x, float angle_y, float angle_z)
{
    return Rx(deg2rad(angle_x)) * Ry(deg2rad(angle_y)) * Rz(deg2rad(angle_z));
}

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

/* create 4x4 matrix float */
cv::Mat rotationMatrixToEulerAngles(float x, float y, float z, float rx, float ry, float rz);
float e_dist(cv::Mat T4x4, float gx, float gy, float gz);
float e_dist_T(cv::Mat T4x4, cv::Mat gtT4x4);
void QuatsToMat(const float *q, float *m);
cv::Mat T_from_q(float x, float y, float z, float qw, float qx, float qy, float qz);

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
    image_0 = cv::imread("image00_00.png", 1);
    image_1 = cv::imread("image11_11.png", 1);
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
    cv::Mat T_id11_cam0 = markers_0[0].getTransformMatrix();
    cv::Mat T_id17_cam0 = markers_0[1].getTransformMatrix();
    cv::Mat T_id17_cam1 = markers_1[0].getTransformMatrix();


    /* g states ground truth */
    //camera on car location ground truth [m]

    /* cam0 External */
    float gx_cam0 = -0.081863;
    float gy_cam0 = -0.56468;
    float gz_cam0 = 3.1863;
    float g_roll_cam0 = 180.0f;
    float g_pitch_cam0 = 180.0f;
    float g_yaw_cam0 = 269.0f;
    //cv::Mat Tcam0 = rotationMatrixToEulerAngles(gx_cam0, gy_cam0, gz_cam0, g_roll_cam0, g_pitch_cam0, g_yaw_cam0);
    cv::Mat Tcam0 = T_from_q(gx_cam0, gy_cam0, gz_cam0, 0.713375f, 0.000012f, 0.0, 0.700783f);

    /* cam1*/
    float gx_cam1 = -0.432727;
    float gy_cam1 = 1.09911;
    float gz_cam1 = -0.449301;
    float g_roll_cam1 = -9.96f;
    float g_pitch_cam1 = 91.2f;
    float g_yaw_cam1 = 87.2f;
    //cv::Mat Tcam1 = rotationMatrixToEulerAngles(gx_cam1, gy_cam1, gz_cam1, g_roll_cam1, g_pitch_cam1, g_yaw_cam1);
    cv::Mat Tcam1 = T_from_q(gx_cam1, gy_cam1, gz_cam1, 0.461698f, -0.535092f, 0.473444f, 0.525702f);

    /* Id17 - the goal and GT*/
    float gx_id17 = -0.22672f;
    float gy_id17 = -1.1264f;
    float gz_id17 = -0.15329f;
    float g_roll_id17 = 135.0f;
    float g_pitch_id17 = 0.0f;
    float g_yaw_id17 = 0.0f;
    //cv::Mat Tid17 = rotationMatrixToEulerAngles(gx_id17, gy_id17, gz_id17, g_roll_id17, g_pitch_id17, g_yaw_id17);
    cv::Mat Tid17 = T_from_q(gx_id17, gy_id17, gz_id17, 0.382683f, 0.92388f, 0.0f, 0.0f);

    /* Id11 */
    float gx_id11 = 0;
    float gy_id11 = 0;
    float gz_id11 = 0;
    float g_roll_id11 = 0.0f;
    float g_pitch_id11 = 0.0f;
    float g_yaw_id11 = 0.0f;
    //cv::Mat Tid11 = rotationMatrixToEulerAngles(gx_id11, gy_id11, gz_id11, g_roll_id11, g_pitch_id11, g_yaw_id11);
    cv::Mat Tid11 = T_from_q(gx_id11, gy_id11, gz_id11, 1.0f, 0.0f, 0.0f, 0.0f);

    cv::Mat gt_T_id17_to_cam0 = Tid17.inv() * Tcam0;
    cv::Mat gt_T_id11_to_cam0 = Tid11.inv() * Tcam0;
    cv::Mat gt_T_id17_to_cam1 = Tid17.inv() * Tcam1;

    cv::Mat gt_T_cam0_to_id17 = Tcam0.inv() * Tid17;
    cv::Mat gt_T_cam0_to_id11 = Tcam0.inv() * Tid11;
    cv::Mat gt_T_cam1_to_id17 = Tcam1.inv() * Tid17;

    /* This is the concept */
    cv::Mat TmarkerOnTheCar = Tid11;
    cv::Mat Tcam12vcs = TmarkerOnTheCar * T_id11_cam0.inv() * T_id17_cam0 * T_id17_cam1.inv(); //the goal
    cv::Mat Tcam02vcs = TmarkerOnTheCar * T_id11_cam0.inv();
    cv::Mat Tid172vcs = TmarkerOnTheCar * T_id11_cam0.inv() * T_id17_cam0;

    float e_cam1 = e_dist(Tcam12vcs, gx_cam1, gy_cam1, gz_cam1);
    float e_cam0 = e_dist(Tcam02vcs, gx_cam0, gy_cam0, gz_cam0);
    float e_id17 = e_dist(Tid172vcs, gx_id17, gy_id17, gz_id17);

    float e_0 = e_dist_T(T_id17_cam0, gt_T_cam0_to_id17);
    float e_1 = e_dist_T(T_id11_cam0, gt_T_cam0_to_id11);
    float e_2 = e_dist_T(T_id17_cam1, gt_T_cam1_to_id17);

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

/* rename it */
cv::Mat rotationMatrixToEulerAngles(float x, float y, float z, float rx, float ry, float rz)
{
    cv::Mat T4x4 = cv::Mat::eye(4, 4, CV_32FC1);
    cv::Mat R3x3_ = cv::Mat::eye(3, 3, CV_32FC1);
    T4x4.at<float>(0, 3) = x;
    T4x4.at<float>(1, 3) = y;
    T4x4.at<float>(2, 3) = z;
    R3x3_ = setR(rx, ry, rz);
    cv::Mat R3x3 = R3x3_.t();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            T4x4.at<float>(i, j) = R3x3.at<float>(i, j);
        }
    }
    return T4x4;
}

cv::Mat T_from_q(float x, float y, float z, float qw, float qx, float qy, float qz)
{
    cv::Mat T4x4 = cv::Mat::eye(4, 4, CV_32FC1);
    T4x4.at<float>(0, 3) = x;
    T4x4.at<float>(1, 3) = y;
    T4x4.at<float>(2, 3) = z;

    /* clean and optmize */
    float q[4];
    float M[9];
    q[0] = qx;
    q[1] = qy;
    q[2] = qz;
    q[3] = qw;
    QuatsToMat(q, M);
    cv::Mat R3x3_ = cv::Mat(3, 3, CV_32FC1, M);
    cv::Mat R3x3 = R3x3_.t();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            T4x4.at<float>(i, j) = R3x3.at<float>(i, j);
        }
    }
    return T4x4;
}

float e_dist(cv::Mat T4x4, float gx, float gy, float gz)
{
    float ex = std::abs(T4x4.at<float>(0, 3) - gx);
    float ey = std::abs(T4x4.at<float>(1, 3) - gy);
    float ez = std::abs(T4x4.at<float>(2, 3) - gz);
    float e = sqrt(ex * ex + ey * ey + ez * ez);
    return e;
}

float e_dist_T(cv::Mat T4x4, cv::Mat gtT4x4)
{
    float ex = std::abs(T4x4.at<float>(0, 3) - gtT4x4.at<float>(0, 3));
    float ey = std::abs(T4x4.at<float>(1, 3) - gtT4x4.at<float>(1, 3));
    float ez = std::abs(T4x4.at<float>(2, 3) - gtT4x4.at<float>(2, 3));
    float e = sqrt(ex * ex + ey * ey + ez * ez);
    return e;
}

void QuatsToMat(const float *q, float *m) {

    float x2 = q[0] + q[0];
    float y2 = q[1] + q[1];
    float z2 = q[2] + q[2];
    {
        float xx2 = q[0] * x2;
        float yy2 = q[1] * y2;
        float zz2 = q[2] * z2;
        m[0 * 3 + 0] = 1.0f - yy2 - zz2;
        m[1 * 3 + 1] = 1.0f - xx2 - zz2;
        m[2 * 3 + 2] = 1.0f - xx2 - yy2;
    }
    {
        float yz2 = q[1] * z2;
        float wx2 = q[3] * x2;
        m[2 * 3 + 1] = yz2 - wx2;
        m[1 * 3 + 2] = yz2 + wx2;
    }
    {
        float xy2 = q[0] * y2;
        float wz2 = q[3] * z2;
        m[1 * 3 + 0] = xy2 - wz2;
        m[0 * 3 + 1] = xy2 + wz2;
    }
    {
        float xz2 = q[0] * z2;
        float wy2 = q[3] * y2;
        m[0 * 3 + 2] = xz2 - wy2;
        m[2 * 3 + 0] = xz2 + wy2;
    }
}