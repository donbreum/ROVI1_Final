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

using namespace rw::math;

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
    String wc_str = ("/home/" +usernamestr +"/Dropbox/tek_studie/1_kandidat/ROVI/Final_Project/PA10WorkCell/PA10WorkCell/ScenePA10RoVi1.wc.xml");
    WorkCell::Ptr wc = WorkCellLoader::Factory::load(wc_str);
    getRobWorkStudio()->setWorkCell(wc);

    // Load Lena image
    Mat im, image;
    String im_str = ("/home/" +usernamestr +"/Dropbox/tek_studie/1_kandidat/ROVI/Final_Project/SamplePluginPA10(1)/SamplePluginPA10/src/lena.bmp");
    im = imread(im_str, CV_LOAD_IMAGE_COLOR); // Read the file
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
    String mot_str = "/home/" +usernamestr +"/Dropbox/tek_studie/1_kandidat/ROVI/Final_Project/SamplePluginPA10(1)/SamplePluginPA10/motions/MarkerMotionSlow.txt";
    marker_motions = readMotionFile(mot_str);
    QObject *obj = sender();
	if(obj==_btn0){
        log().info() << "Button 00000\n";
		// Set a new texture (one pixel = 1 mm)
		Image::Ptr image;
        String ima_str = ("/home/" +usernamestr +"/Dropbox/tek_studie/1_kandidat/ROVI/Final_Project/SamplePluginPA10(1)/SamplePluginPA10/markers/Marker1.ppm");
        image = ImageLoader::Factory::load(ima_str);
		_textureRender->setImage(*image);
        String imag_str = ("/home/" +usernamestr +"/Dropbox/tek_studie/1_kandidat/ROVI/Final_Project/SamplePluginPA10(1)/SamplePluginPA10/backgrounds/white.ppm");
        image = ImageLoader::Factory::load(imag_str);
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

        double f = 823.0; // focal length
        double z = 0.5; // distance from camera to point being tracked. 0.5 meters
        std::vector<Vector2D<double> >uv_points = get_marker_pts(marker, cameraFrame, 1, z, f);
        goal = uv_points[0];
        log().info() << "goal " << goal << endl;
        // now the image is drawed on while the center points are aquired, and then you can get the image with getImage().
        //The point is the center point, which can be used for tracking center of picture
        cv::Point point1 = vis->getCenterPoint(im);

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

//    int cols = imflip.cols;//
 //   int rows = imflip.rows;
//    cv::Point viewCenterPoint(cols/2,rows/2);
//    cv::circle(imflip,viewCenterPoint, 4, cv::Scalar(0, 0, 0), 5);

log().info() << "counter val: " << counter << "\n";
//marker->moveTo(marker_motions[counter++], state);
marker->setTransform(marker_motions[counter++], state); // both moveTo and setTransform is functions from the Movableframe class. They seem to do the same thing

getRobWorkStudio()->setState(state);

//const rw::math::Transform3D<> baseTtool = device->baseTframe(cameraFrame, state);

/// new approach
///
double f = 823.0; // focal length
double z = 0.5; // distance from camera to point being tracked. 0.5 meters

    std::vector<Vector2D<double> >uv_points = get_marker_pts(marker, cameraFrame, 1, z, f);
    cv::Point ptt(uv_points[0][0]+(imflip.cols/2),uv_points[0][1]+(imflip.rows/2));
//    log().info() << "point::::: " << ptt << endl;
    cv::circle(imflip,ptt, 4, cv::Scalar(255, 0, 0), 5);


    Jacobian j_image = getImageJacobian(uv_points[0],f,z);

    Jacobian j__z = calcZimage(j_image, device, q_init, state); // return  Jimage*S*J;

    // https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html
    // use eigen library for handling the matrix calculations
    // matrices sizes does not have to be known at compile time with Eigen::MatrixXd (last letter is specifing datatype, e.g. double)

    // WHAT WE NEEED (taken from the robotics notes):
    // Z_image(q) = J_image * S(q) * J(q)  --- calculated above, need new representation
    // Need to solve:  (Z_Image(q) * Z_image(q)^Transpose) * y = [du]  --- we have [du] from above


    // get the eigen representation of the j_z Jacobian
    Eigen::MatrixXd z_image = j__z.e();

    // get the inverse of z_image matrix
    Eigen::MatrixXd z_inv = z_image.inverse();

    // get the transpose of z_image: Z_image(q)^Tranpose
    Eigen::MatrixXd z_image_transpose = z_image.transpose();

    // to use for calculating Moore-Penrose inverse (we can use this if there is more than 2 DOF)
    // formula: Z* = inverse(Z^T * Z) * Z^T
    Eigen::MatrixXd z_moore_penrose = z_image_transpose*LinearAlgebra::pseudoInverse(z_image*z_image_transpose);

//    log().info() << "zzz\n" << zz << endl;
//    log().info() << "goal " << goal << endl;
//    log().info() << "curr point  " << uv_points[0] << endl;

    // subtract the current point from the desired point
    Vector2D<double> error_tracking  = goal - uv_points[0];
//    log().info() << "error " << error_tracking << endl;

    // cast the tracked point(s) to a Jacobian and push it to our vector of jacobian - it will hold the stacked Jacobians, if we have more than 1 tracking point
    std::vector<Jacobian> jac;
    jac.push_back(Jacobian(error_tracking.e()));

 //    log().info() << "jacs  " << jacbs[0] << endl;

    // Now we have all to solve for y (see Robotics notes p. 52 top)
    Jacobian y(z_moore_penrose * jac[0].e());

    // create Q vector for the chosen y
    Q dq_q(y.e());

    //  log().info() << "dq_q\n  " << dq_q << endl;

    // log().info() << "curr Q\n  " << temp << endl;

     log().info() << "vel limits: " << device->getVelocityLimits() << endl;

    // get current configuration
    Q temp = device->getQ(state);
    // set the deltaQ to current configuration + the calculated adjustment
    Q dq = temp;
    dq += dq_q;
    // log().info() << "NEW  Q\n  " << dq << "\n\n"<<endl;

    // remember to set the new configuration and set the state in robworkstudio
    device->setQ(dq, state);
    getRobWorkStudio()->setState(state);

//    zzz = LinearAlgebra::pseudoInverse(z_imageE*z_transpose) *z_transpose ; // 2x2
//    log().info() << "zzz1\n" << zzz << endl;


//    zzz = LinearAlgebra::pseudoInverse(z_transpose * z_imageE) *z_transpose ; // 7x2 - same as the first
//    log().info() << "zzz2\n" << zzz << endl;


    //z_inverse.transpose()*points[0];
//    log().info() << "z_image\n " << z_image << endl;
//    log().info() << "z_image transpose\n " << z_transpose << endl;

//    log().info() << "z_inverse\n " << z_inv << endl;
//    log().info() << "z_z_tranpose : " << z_z_transpose << endl;

///
///
///


    setCamera(imflip); // set the image that we see in the workcell camera simulator
}

void SamplePlugin::stateChangedListener(const State& state) {
  _state = state;
}

std::vector<Vector2D<double> > SamplePlugin::get_marker_pts(Frame *marker_frame, Frame *camera_frame, int num_tracked_points,double z, double f){

    std::vector<Vector2D<double>> uv_points; // used to hold all points tracked

    Transform3D<> marker_transform = inverse(marker_frame->fTf(camera_frame, state)); // frame-to-frame transformation, used to find midpoint of marker in the workspace

    std::vector<Vector3D<> > pts_marker; // Vector3D used since we need to use 3D coordinates for tests (though the last is 0)

    pts_marker.push_back(marker_transform * Vector3D<>(0,0,0));
    //log().info() << "marker points\n " << pts_marker[0] << endl;
    for(auto pt_marker : pts_marker)
    {
        Vector2D<double> pt;
        pt[0] = f * pt_marker[0] / z; // using pinholde model to obtain image coordinates u,v
        pt[1] = f * pt_marker[1] / z;
        uv_points.push_back(pt);
     }
    return uv_points;
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


Jacobian SamplePlugin::getImageJacobian(Vector2D<> uv_pts, double f, double z) //
{
    //double x = xyz.x;//[0];
    //double y = xyz.y;//[1];
    //int z = xyz[2];
    double u = uv_pts[0];
    double v = uv_pts[1];
    Jacobian Jimage(2,6);
    Jimage(0,0) = -f/z;
    Jimage(0,1) = 0;
    Jimage(0,2) = u/z;
    Jimage(0,3) = u*v/f;
    Jimage(0,4) = -((pow(f,2)+pow(u,2))/f);
    Jimage(0,5) = v;
    Jimage(1,0) = 0;
    Jimage(1,1) = -f/z;
    Jimage(1,2) = v/z;
    Jimage(1,3) = ((pow(f,2)+pow(v,2))/f);
    Jimage(1,4) = -u*v/f;
    Jimage(1,5) = -u;
    return Jimage;
}

Jacobian SamplePlugin::calcZimage(const Jacobian Jimage, rw::models::Device::Ptr device, const Q q, rw::kinematics::State state)
{
    //device->setQ(q,state);

    Rotation3D<> baseRtool = device->baseTend(state).R().inverse();
    Jacobian S(6,6);
    S.e().setZero();
    S.e().block<3,3>(0,0) = S.e().block<3,3>(3,3) = baseRtool.e();
    Jacobian J = device->baseJend(state);
    return Jimage*S*J;
}

void SamplePlugin::drawLine( Mat img)
{
    int im_cols = img.cols/2;
    int im_rows = img.rows/2;
    cv::Point start(im_rows,im_cols);
    cv::Point end1(im_rows,im_cols+50);
    cv::Point end2(im_rows+50,im_cols);

  int thickness = 5;
  int lineType = 8;
  arrowedLine( img,
        start,
        end1,
        Scalar( 0, 0, 0 ),
        thickness,
        lineType/*,
        shift,
        tiplength*/);

  arrowedLine( img,
        start,
        end2,
        Scalar( 0, 0, 0 ),
        thickness,
        lineType/*,
        shift,
        tiplength*/);
}
