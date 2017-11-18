#include "SamplePlugin.hpp"

#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <functional>

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace rws;

using namespace cv;

using namespace std::placeholders;

using namespace std;

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	_timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

	Image textureImage(300,300,Image::GRAY,Image::Depth8U);
	_textureRender = new RenderImage(textureImage);
	Image bgImage(0,0,Image::GRAY,Image::Depth8U);
	_bgRender = new RenderImage(bgImage,2.5/1000.0);
	_framegrabber = NULL;
}

SamplePlugin::~SamplePlugin()
{
    delete _textureRender;
    delete _bgRender;
}

void SamplePlugin::initialize() {
	log().info() << "INITALIZE" << "\n";

    getRobWorkStudio()->stateChangedEvent().add(std::bind(&SamplePlugin::stateChangedListener, this, _1), this);

    // Auto load workcell
    WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/stu"

                                                     "dent/Dropbox/tek_studie/1_kandidat/ROVI/Final_Project/PA10WorkCell/PA10WorkCell/ScenePA10RoVi1.wc.xml");
    getRobWorkStudio()->setWorkCell(wc);

    // Load Lena image
    Mat im, image;
    im = imread("/home/student/Dropbox/tek_studie/1_kandidat/ROVI/Final_Project/SamplePluginPA10(1)/SamplePluginPA10/src/lena.bmp", CV_LOAD_IMAGE_COLOR); // Read the file
    cvtColor(im, image, CV_BGR2RGB); // Switch the red and blue color channels
    if(! image.data ) {
        RW_THROW("Could not open or find the image: please modify the file path in the source code!");
    }
    QImage img(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888); // Create QImage from the OpenCV image
    _label->setPixmap(QPixmap::fromImage(img)); // Show the image at the label in the plugin
}


void SamplePlugin::open(WorkCell* workcell)
{
    log().info() << "OPEN" << "\n";
    _wc = workcell;
    _state = _wc->getDefaultState();

    log().info() << workcell->getFilename() << "\n";

    if (_wc != NULL) {
    // Add the texture render to this workcell if there is a frame for texture
    Frame* textureFrame = _wc->findFrame("MarkerTexture");
    if (textureFrame != NULL) {
        getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
    }
    // Add the background render to this workcell if there is a frame for texture
    Frame* bgFrame = _wc->findFrame("Background");
    if (bgFrame != NULL) {
        getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
    }

    // Create a GLFrameGrabber if there is a camera frame with a Camera property set
    Frame* cameraFrame = _wc->findFrame("CameraSim");
    if (cameraFrame != NULL) {
        if (cameraFrame->getPropertyMap().has("Camera")) {
            // Read the dimensions and field of view
            double fovy;
            int width,height;
            std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
            std::istringstream iss (camParam, std::istringstream::in);
            iss >> fovy >> width >> height;
            // Create a frame grabber
            _framegrabber = new GLFrameGrabber(width,height,fovy);
            SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
            _framegrabber->init(gldrawer);
        }
    }
    // Find and create a pointer to the device
    device = _wc->findDevice("PA10");

    // Get default state
    state = _wc->getDefaultState();
    }

        vis = new Vision;

}


void SamplePlugin::close() {
    log().info() << "CLOSE" << "\n";

    // Stop the timer
    _timer->stop();
    // Remove the texture render
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}
	// Remove the background render
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	}
	// Delete the old framegrabber
	if (_framegrabber != NULL) {
		delete _framegrabber;
	}
	_framegrabber = NULL;
	_wc = NULL;
}

Mat SamplePlugin::toOpenCVImage(const Image& img) {
    Mat res(img.getHeight(),img.getWidth(), CV_8UC3);
	res.data = (uchar*)img.getImageData();
	return res;
}


void SamplePlugin::btnPressed() {
    marker_motions = readMotionFile("/home/student/Dropbox/tek_studie/1_kandidat/ROVI/Final_Project/SamplePluginPA10(1)/SamplePluginPA10/motions/MarkerMotionSlow.txt");
    QObject *obj = sender();
	if(obj==_btn0){
        log().info() << "Button 00000\n";
		// Set a new texture (one pixel = 1 mm)
		Image::Ptr image;
        image = ImageLoader::Factory::load("/home/student/Dropbox/tek_studie/1_kandidat/ROVI/Final_Project/SamplePluginPA10(1)/SamplePluginPA10/markers/Marker1.ppm");
		_textureRender->setImage(*image);
        image = ImageLoader::Factory::load("/home/student/Dropbox/tek_studie/1_kandidat/ROVI/Final_Project/SamplePluginPA10(1)/SamplePluginPA10/backgrounds/white.ppm");
		_bgRender->setImage(*image);
		getRobWorkStudio()->updateAndRepaint();
	} else if(obj==_btn1){

        //state = getRobWorkStudio()->getState();
        MovableFrame* marker = (MovableFrame*) _wc->findFrame("Marker");
        // Find the marker frame origin coordinate relative to the camera
        Frame* cameraFrame = _wc->findFrame("Camera");

        Q q(7, 0, -0.65, 0, 1.80, 0, 0.42, 0);
        q_init = q;;
        device->setQ(q, state);
        log().info() << q << endl;
        //counter = 0;
        marker->moveTo(marker_motions[0], state);
        getRobWorkStudio()->setState(state);

        cv::Mat im = getCameraImage();
        // now the image is drawed on while the center points are aquired, and then you can get the image with getImage().
        //The point is the center point, which can be used for tracking center of picture
        cv::Point point = vis->getCenterPoint(im);
        lastPoint = point;
        log().info() << "First center: " << point << endl ;
        log().info() << "Button 1111\n";
		// Toggle the timer on and off
        if (!_timer->isActive()
                )
		    _timer->start(100); // run 10 Hz
		else
			_timer->stop();
	} else if(obj==_spinBox){
		log().info() << "spin value:" << _spinBox->value() << "\n";
	}
}

void SamplePlugin::timer() {

    state = getRobWorkStudio()->getState();

    // Find the marker frame and cast it to a moveable frame
    MovableFrame* marker = (MovableFrame*) _wc->findFrame("Marker");
    // Find the marker frame origin coordinate relative to the camera
    cameraFrame = _wc->findFrame("Camera");

    cv::Mat im = getCameraImage();
    // now the image is drawed on while the center points are aquired, and then you can get the image with getImage().
    //The point is the center point, which can be used for tracking center of picture
    cv::Point point = vis->getCenterPoint(im);
    cv::Mat imflip =vis->getImage();

    int cols = imflip.cols;
    int rows = imflip.rows;
    cv::Point viewCenterPoint(cols/2,rows/2);
    cv::circle(imflip,viewCenterPoint, 4, cv::Scalar(0, 0, 0), 5);

log().info() << "counter val: " << counter << "\n";
//marker->moveTo(marker_motions[counter++], state);
marker->setTransform(marker_motions[counter++], state); // both moveTo and setTransform is functions from the Movableframe class. They seem to do the same thing

getRobWorkStudio()->setState(state);

const rw::math::Transform3D<> baseTtool = device->baseTframe(cameraFrame, state);

// Choose a small positional change, deltaP (ca. 10^-4)
double deltaX = point.x - lastPoint.x ;//0.0035;
double deltaY = (point.y - lastPoint.y);//0.0035;
double diffC = (point.x) - viewCenterPoint.x;
const double delta = 0.0;
log().info() << "diff : " << diffC << endl;
log().info() << "point.y: " << (point.y) << endl;
const rw::math::Vector3D<double> deltaP(0,delta , 0);



// Choose baseTtool_desired by adding the positional change deltaP to the position part of baseTtool
const rw::math::Vector3D<> deltaPdesired = baseTtool.P() + deltaP;


const rw::math::Transform3D<double> baseTtool_desired(deltaPdesired, baseTtool.R());

//// Apply algorithm 1

rw::math::Q q_desired = algorithm1(device, state, cameraFrame, baseTtool_desired, q_init);

     device->setQ(q_desired, state);
getRobWorkStudio()->setState(state);
 log().info() <<"alg1 Q; " << q_desired << endl;


    setCamera(imflip); // set the image that we see in the workcell camera simulator

    lastPoint = point; // the current point will be used next time for calculating diff for translation
}

void SamplePlugin::stateChangedListener(const State& state) {
  _state = state;
}



std::vector<Transform3D<double> > SamplePlugin::readMotionFile(std::string file_name)
{
    // here we can read the different motions files. Slow, normal fast files. Which one are loaded are hardcoded atm. in btn1 press
    RPY<double> rpy;
    Vector3D<double> xyz;
    std::vector<Transform3D<double> > motions; // Vector to be returned
    string line;
    double x, y, z, R, P, Y;

    ifstream file(file_name);

    if(file.is_open())
    {
        while(getline(file,line))
        {
            stringstream input_stream(line); // Create a stream lines
            // Read from motions file
            input_stream >> x >> y >> z >> R >> P >> Y;

            rpy = RPY<double>(R,P,Y);   // RPY matrix
            xyz = Vector3D<double>(x,y,z);   //  vector for translation

            // Create matrix with rpy and translation. Vector is used for motions
            motions.push_back(Transform3D<double>(xyz,rpy.toRotation3D()));
        }

        file.close();
    }

    return motions;
}

// This function calculates delta U as in Equation 4.13. The output class is a velocity screw as that is a 6D vector with a positional and rotational part
// What a velocity screw really is is not important to this class. For our purposes it is only a container.
rw::math::VelocityScrew6D<double> SamplePlugin::calculateDeltaU(const rw::math::Transform3D<double>& baseTtool, const rw::math::Transform3D<double>& baseTtool_desired) {
    // Calculate the positional difference, dp
    rw::math::Vector3D<double> dp = baseTtool_desired.P() - baseTtool.P();

    // Calculate the rotational difference, dw
    rw::math::EAA<double> dw(baseTtool_desired.R() * rw::math::inverse(baseTtool.R()));

    return rw::math::VelocityScrew6D<double>(dp, dw);
}

// The inverse kinematics algorithm needs to know about the device, the tool frame and the desired pose. These parameters are const since they are not changed by inverse kinematics
// We pass the state and the configuration, q, as value so we have copies that we can change as we want during the inverse kinematics.
rw::math::Q SamplePlugin::algorithm1(const rw::models::Device::Ptr device, rw::kinematics::State state, const rw::kinematics::Frame* tool,
                       const rw::math::Transform3D<double> baseTtool_desired, rw::math::Q q) {
    // We need an initial base to tool transform and the positional error at the start (deltaU)
    rw::math::Transform3D<> baseTtool = device->baseTframe(tool, state);
    rw::math::VelocityScrew6D<double> deltaU = calculateDeltaU(baseTtool, baseTtool_desired);

    // This epsilon is the desired tolerance on the final position.
    const double epsilon = 0.0001;
    while(deltaU.norm2() > epsilon) {
        rw::math::Jacobian J = device->baseJframe(tool, state); // This line does the same as the function from Programming Exercise 4.1
    // We need the inverse of the jacobian. To do that, we need to access the Eigen representation of the matrix.
    // For information on Eigen, see http://eigen.tuxfamily.org/.
    rw::math::Jacobian Jinv(J.e().inverse());

    // In RobWork there is an overload of operator* for Jacobian and VelocityScrew that gives Q
    // This can also manually be done via Eigen as J.e().inverse() * deltaU.e()
    // Note that this approach only works for 6DOF robots. If you use a different robot, you need to use a pseudo inverse to solve the equation J * deltaQ = deltaU
    rw::math::Q deltaQ = Jinv*deltaU;

    // Here we add the change in configuration to the current configuration and move the robot to that position.
        q += deltaQ;
        log().info() << "dd: " << q_init << endl;
        device->setQ(q, state);

    // We need to calculate the forward dynamics again since the robot has been moved
        baseTtool = device->baseTframe(tool, state); // This line performs the forward kinematics (Programming Exercise 3.4)

    // Update the cartesian position error
        deltaU = calculateDeltaU(baseTtool, baseTtool_desired);
    }
    return q;
}

void SamplePlugin::writeLog(int i){
    log().info() << "size: " << i << endl;
}

void SamplePlugin::setCamera(cv::Mat imflip){
    // Show in QLabel
    QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
    QPixmap p = QPixmap::fromImage(img);
    unsigned int maxW = 400;
    unsigned int maxH = 800;
    _label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
}

cv::Mat SamplePlugin::getCameraImage(){
    cv::Mat dummy;

    if (_framegrabber != NULL) {
        // Get the image as a RW image
        Frame* cameraFrame = _wc->findFrame("CameraSim");
        _framegrabber->grab(cameraFrame, _state);
        const Image& image = _framegrabber->getImage();

        // Convert to OpenCV image
        Mat im = toOpenCVImage(image);
        Mat imflip;
        cv::flip(im, imflip, 0);

        return imflip;
    }
    return dummy;
}
