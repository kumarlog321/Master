 #include "aruco.h"
#include "cvdrawingutils.h"
#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <string>
#include <stdexcept>

struct   TimerAvrg { std::vector<double> times;
size_t curr = 0, n; std::chrono::high_resolution_clock::time_point begin, end;   TimerAvrg(int _n = 30) 
{ n = _n; times.reserve(n); }inline void start()
{ begin = std::chrono::high_resolution_clock::now(); }inline void stop()
{ end = std::chrono::high_resolution_clock::now(); double duration = double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count())*1e-6; if (times.size() < n) times.push_back(duration); else { times[curr] = duration; curr++; if (curr >= times.size()) curr = 0; } }double getAvrg()
{ double sum = 0; for (auto t : times) sum += t; return sum / double(times.size()); } };


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
    image_0 = cv::imread("image_0.jpg", 1);
    image_1 = cv::imread("image_1.png", 1);
    std::vector<aruco::Marker> markers_0;
    std::vector<aruco::Marker> markers_1;

    fps_.start();
    markers_0 = detector_0.detect(image_0, camparams_0, markerSize_0);
    markers_1 = detector_1.detect(image_1, camparams_1, markerSize_1);
    fps_.stop();

    // chekc the speed by calculating the mean speed of all iterations
    std::cout << "\rTime detection=" << fps_.getAvrg() * 1000 << " milliseconds nmarkers=" << markers_0.size() << " images resolution=" << image_0.size() << std::endl;
    std::cout << "\rTime detection=" << fps_.getAvrg() * 1000 << " milliseconds nmarkers=" << markers_1.size() << " images resolution=" << image_1.size() << std::endl;

    /* The first camera */
    auto candidates_0 = detector_0.getCandidates();
    for (auto cand : candidates_0)
        aruco::Marker(cand, -1).draw(image_0, cv::Scalar(255, 0, 255));

    for (unsigned int i = 0; i < markers_0.size(); i++) {
        std::cout << markers_0[i] << std::endl;
        markers_0[i].draw(image_0, cv::Scalar(0, 0, 255), 2, true);
    }

    // draw a 3d cube in each marker if there is 3d info
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

    // draw a 3d cube in each marker if there is 3d info
    if (camparams_1.isValid() && markerSize_1 > 0) {
        for (unsigned int i = 0; i < markers_1.size(); i++) {
            aruco::CvDrawingUtils::draw3dCube(image_1, markers_1[i], camparams_1);
            aruco::CvDrawingUtils::draw3dAxis(image_1, markers_1[i], camparams_1);
        }
    }

    return 0;
}