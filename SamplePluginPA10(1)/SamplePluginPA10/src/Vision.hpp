#ifndef VISION_H
#define VISION_H


#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/shape.hpp>
#include "opencv2/imgcodecs.hpp"

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>

// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>

#include <rws/RobWorkStudioPlugin.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/RPY.hpp>
#include <rw/kinematics/MovableFrame.hpp>


#include <iostream>
#include <string>
using namespace rw::common;
using namespace std;
using namespace cv;


using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;
using namespace rw::math;

using namespace rws;


class Vision{

public:

    cv::Point getCenterPoint(cv::Mat inputImg);
    cv::Point findMidPoint(std::vector<cv::Point> red,std::vector<cv::Point> blue);
    void writeImage(Mat &image, String name);

    cv::Mat getImage();

    cv::Mat imgDraw;

private:
};


#endif
