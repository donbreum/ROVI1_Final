#ifndef VISION_H
#define VISION_H


#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/shape.hpp>
#include "opencv2/imgcodecs.hpp"

#include <iostream>
#include <string>

using namespace std;
using namespace cv;



class Vision{

public:

    cv::Point getCenterPoint(cv::Mat inputImg);
    cv::Point findMidPoint(std::vector<cv::Point> pts);
    void writeImage(Mat &image, String name);

private:
};


#endif
