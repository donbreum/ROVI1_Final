#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#define DEF_VISION 1 // set to 1 if you are using the vision tracking, other set to 0 for using the given marker coordinates
#define DEF_MARKER 1 // set to the correct marker. Only marker 1 and 3 is implemented
#define POINTS 3 // number of points to be tracked - need to be either 1 or 3
#define SEQUENCE "Slow"
#define d_t 0.15
#define TAU 0.1 // this is for marker1

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>

// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>


// OpenCV 3
#include <opencv2/opencv.hpp>


// Qt
#include <QTimer>

#include "ui_SamplePlugin.h"

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

#include <rw/math/LinearAlgebra.hpp>

#include <string>

#include <algorithm>    // std::min
#include <fstream>
#include <stdlib.h>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;
using namespace rw::math;

using namespace rws;

using namespace cv;

using namespace std;

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
public:
    SamplePlugin();
    virtual ~SamplePlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

    void writeLog(int i);
    cv::Point lastPoint;
    void setCamera(cv::Mat img);


private slots:

    void btnPressed();
    void timer();
  
    void stateChangedListener(const rw::kinematics::State& state);

private:
    static cv::Mat toOpenCVImage(const rw::sensor::Image& img);
    std::vector<Transform3D<double> > read_file(std::string f);
    rw::models::Device::Ptr device;
    State state;
    int counter = 0;
   std::vector<Transform3D<double> > marker_motions;
    Frame* cameraFrame;
    QTimer* _timer;
    rw::math::Q q_init;

    cv::Mat getCameraImage();
    std::vector<Jacobian> getImageJacobian(std::vector<Vector2D<> > uv_pts);
    vector<Jacobian> calcZimage(const vector<Jacobian> Jimage, rw::models::Device::Ptr device, const Q q, rw::kinematics::State state);
    vector<Vector2D<double> > goal;
    Jacobian stack_jacobian(vector<Jacobian> j);
    Jacobian stack_dudv(vector<Vector2D<double> >);
    std::vector<Vector2D<double> > get_marker_pts(Frame *marker_frame, Frame *camera_frame, int num_tracked_points);
    void print_all_data(vector<Vector2D<double> > track_error);
    vector<Point2f> getPoints(Mat src);
    vector<Point2f> getCornyPoints(Mat img_object, Mat img_scene);
    std::vector<Vector2D<double> > get_tracking_pts(Mat cam_img, MovableFrame* marker);
    vector<Vector2D<double> > find_tracking_error(std::vector<Vector2D<double> >uv_points);
    void print_bound_scaled(Q qq);
    Q calc_delta_q(vector<Jacobian> z_img,std::vector<Vector2D<double> >uv_points);

    String usernamestr = "per";
    double DT = d_t;
    ofstream error_data;
    ofstream pose_config_data;
    ofstream velocity_scaled;
    ofstream pos_scaled;
    double max_error = 0;

    cv::Mat img_corny;
    int MARKER = DEF_MARKER;
    int VISION = DEF_VISION;
    double f = 823.0; // focal length
    double z = 0.5; // distance from camera to point being tracked. 0.5 meters


    rw::models::WorkCell::Ptr _wc;
    rw::kinematics::State _state;
    rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
    rwlibs::simulation::GLFrameGrabber* _framegrabber;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
