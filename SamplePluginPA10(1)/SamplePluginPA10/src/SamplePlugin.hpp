#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

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

#include <string>

#include "Vision.hpp"

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

    rw::math::VelocityScrew6D<double> calculateDeltaU(const rw::math::Transform3D<double>& baseTtool, const rw::math::Transform3D<double>& baseTtool_desired);
    rw::math::Q algorithm1(const rw::models::Device::Ptr device, rw::kinematics::State state, const rw::kinematics::Frame* tool,
                           const rw::math::Transform3D<double> baseTtool_desired, rw::math::Q q);

    Vision *vis;
    void writeLog(int i);
    cv::Point lastPoint;
    void setCamera(cv::Mat img);
    cv::Mat getCameraImage();
private slots:

    void btnPressed();
    void timer();
  
    void stateChangedListener(const rw::kinematics::State& state);

private:
    static cv::Mat toOpenCVImage(const rw::sensor::Image& img);
    std::vector<Transform3D<double> > readMotionFile(std::string fileName);
    rw::models::Device::Ptr device;
    State state;
    int counter = 0;
   std::vector<Transform3D<double> > marker_motions;
    Frame* cameraFrame;
    QTimer* _timer;
    rw::math::Q q_init;


    rw::models::WorkCell::Ptr _wc;
    rw::kinematics::State _state;
    rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
    rwlibs::simulation::GLFrameGrabber* _framegrabber;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
