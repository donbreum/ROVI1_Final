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
}

void SamplePlugin::close() {
    log().info() << "CLOSE" << "\n";
     pose_config_data.close();
     velocity_scaled.close();
     pos_scaled.close();
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
    String mot_str = "/home/" +usernamestr +"/Dropbox/tek_studie/1_kandidat/ROVI/Final_Project/SamplePluginPA10(1)/SamplePluginPA10/motions/MarkerMotion"+SEQUENCE+".txt";
    marker_motions = read_file(mot_str);
    QObject *obj = sender();
	if(obj==_btn0){
        log().info() << "Button 00000\n";
		// Set a new texture (one pixel = 1 mm)
		Image::Ptr image;
        String marker_num = std::to_string(MARKER);
        String ima_str = ("/home/" +usernamestr +"/Dropbox/tek_studie/1_kandidat/ROVI/Final_Project/SamplePluginPA10(1)/SamplePluginPA10/markers/Marker"+marker_num+".ppm");
        image = ImageLoader::Factory::load(ima_str);
		_textureRender->setImage(*image);
        String imag_str = ("/home/" +usernamestr +"/Dropbox/tek_studie/1_kandidat/ROVI/Final_Project/SamplePluginPA10(1)/SamplePluginPA10/backgrounds/white.ppm");
        image = ImageLoader::Factory::load(imag_str);
		_bgRender->setImage(*image);
		getRobWorkStudio()->updateAndRepaint();
        counter = 0;
        String seq = std::to_string(POINTS);
        //String seq = POINTSSTR;
        pose_config_data.open ("DATA_"+seq+"_"+SEQUENCE+".txt");


        std::string filename_object = "/home/" +usernamestr +"/Dropbox/tek_studie/1_kandidat/ROVI/Final_Project/SamplePluginPA10(1)/SamplePluginPA10/markers/Marker3.ppm";
        img_corny = cv::imread(filename_object);

        velocity_scaled.open("velocity_scaled.txt");
        pos_scaled.open("pos_scaled.txt");

	} else if(obj==_btn1){

        //state = getRobWorkStudio()->getState();
        MovableFrame* marker = (MovableFrame*) _wc->findFrame("Marker");
        // Find the marker frame origin coordinate relative to the camera
        //Frame* cameraFrame = _wc->findFrame("Camera");

        Q q(7, 0, -0.65, 0, 1.80, 0, 0.42, 0);
        q_init = q;;
        device->setQ(q, state);
        log().info() << q << endl;
        //counter = 0;
        marker->moveTo(marker_motions[0], state);
        getRobWorkStudio()->setState(state);

        cv::Mat im = getCameraImage();

        std::vector<Vector2D<double> >uv_points = get_tracking_pts(im, marker);

        goal = uv_points;

		// Toggle the timer on and off
        if (!_timer->isActive()
                )
            _timer->start(100); // run 10 Hz (value 100)
		else
			_timer->stop();
	} else if(obj==_spinBox){
		log().info() << "spin value:" << _spinBox->value() << "\n";
	}
}

void SamplePlugin::timer() {

    clock_t t_start = clock(); // used to see time spent on image processing
    state = getRobWorkStudio()->getState();

    // Find the marker frame and cast it to a moveable frame
    MovableFrame* marker = (MovableFrame*) _wc->findFrame("Marker");
    // Find the marker frame origin coordinate relative to the camera
    cameraFrame = _wc->findFrame("Camera");

    cv::Mat im = getCameraImage();

    if(counter < marker_motions.size()){
        //marker->moveTo(marker_motions[counter++], state);
        marker->setTransform(marker_motions[counter++], state); // both moveTo and setTransform is functions from the Movableframe class. They seem to do the same thing
        //log().info() << "counter val: " << counter << "\n";
        getRobWorkStudio()->setState(state);
    }

    std::vector<Vector2D<double> >uv_points = get_tracking_pts(im, marker);

    log().info() << "Time used for image processing: " <<double(clock()-t_start)/CLOCKS_PER_SEC << endl;
    clock_t t_start1 = clock();

    vector<Jacobian> j_image = getImageJacobian(uv_points);

    vector<Jacobian> j__z = calcZimage(j_image, device, q_init, state); // return  Jimage*S*J;

    Q dq = calc_delta_q(j__z,uv_points);

    print_bound_scaled(dq); // should have been in the other print function with the rest :)

    // remember to set the new configuration and set the state in robworkstudio
    if(counter < marker_motions.size()){
        device->setQ(dq, state);
        getRobWorkStudio()->setState(state);
    }

    //log().info() << "Time for inverse calculation: " <<double(clock()-t_start1)/CLOCKS_PER_SEC << endl;
    setCamera(im); // set the image that we see in the workcell camera simulator
}

void SamplePlugin::print_bound_scaled(Q qq){

    // BOUNDS WITH RESPECT TO LIMITS
     auto limit = device->getBounds();
     for(int i = 0; i < limit.first.size(); i++){
         //log().info() << "q: " << qq << endl;
         pos_scaled << qq[i]/limit.second[i] << ";";
         //log().info() << "lim pos: " << qq[i]/limit.second[i] << endl;
     }
    pos_scaled << endl;

}

vector<Vector2D<double> > SamplePlugin::find_tracking_error(std::vector<Vector2D<double> >uv_points){

    double err =0;
    vector<Vector2D<double> > error_tracking;
    error_tracking.resize(POINTS);
    for(int i = 0; i < POINTS; i++){
     // subtract the current point from the desired point
     error_tracking[i]  = goal[i] - uv_points[i];

     // used to calculate max_error in a single run
     err = abs(error_tracking[i][0]) + abs(error_tracking[i][1]);
     if(max_error < err)
        max_error = err;

    }

    // When we are finisheded with a sequence print all data
    if(counter < marker_motions.size()){
        print_all_data(error_tracking);
        String dt_seq = std::to_string(d_t);
        String seq = std::to_string(POINTS);
        error_data.open("max_error"+dt_seq+"_"+seq+"_"+SEQUENCE+".txt");
        error_data << max_error << endl;
        error_data.close();
    }

    return error_tracking;
}

Q SamplePlugin::calc_delta_q(vector<Jacobian> j__z,std::vector<Vector2D<double> >uv_points){

    Jacobian stacked_jac = stack_jacobian(j__z);

    // https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html
    // use eigen library for handling the matrix calculations
    // matrices sizes does not have to be known at compile time with Eigen::MatrixXd (last letter is specifing datatype, e.g. double)

    // WHAT WE NEEED (taken from the robotics notes):
    // Z_image(q) = J_image * S(q) * J(q)  --- calculated above, need new representation
    // Need to solve:  (Z_Image(q) * Z_image(q)^Transpose) * y = [du]  --- we have [du] from above


    // get the eigen representation of the j_z Jacobian
    Eigen::MatrixXd z_image;
    if(POINTS == 3)
        z_image = stacked_jac.e();
    else if(POINTS == 1)
        z_image = stacked_jac.e();

    // get the inverse of z_image matrix
    Eigen::MatrixXd z_inv = z_image.inverse();
    //Eigen::MatrixXd z_inv = z_image.inverse();

    // get the transpose of z_image: Z_image(q)^Tranpose
    Eigen::MatrixXd z_image_transpose = z_image.transpose();

//    // to use for calculating Moore-Penrose inverse (we can use this if there is more than 2 DOF)
//    // formula: Z* = inverse(Z^T * Z) * Z^T
    Eigen::MatrixXd z_moore_penrose = z_image_transpose*LinearAlgebra::pseudoInverse(z_image*z_image_transpose);

    vector<Vector2D<double> > error_tracking = find_tracking_error(uv_points);

    Jacobian stacked_dudv = stack_dudv(error_tracking);

    Q dq;
    if(POINTS == 3){
        Jacobian y(z_moore_penrose * stacked_dudv.e());
        Q dq_calculated(y.e());
        Q velocityLimits =  device->getVelocityLimits() ;

        vector<double> scaled_velocity;
        for(int i = 0; i < dq_calculated.size(); i++){
        //scaled_velocity.push_back( = dq_calculated[i]/velocityLimits[i];
        log().info() << "vel limit: " << velocityLimits << endl;
        log().info() << "vel scaled: "<<dq_calculated[i]/velocityLimits[i] << endl;
        }

        dq_calculated = dq_calculated*(DT-TAU);

        for(int i = 0; i < dq_calculated.size();i++){
            if(dq_calculated[i] > 0)
                dq_calculated[i] = std::min(dq_calculated[i],velocityLimits[i]);
            else
                dq_calculated[i] = -(std::min(-dq_calculated[i],velocityLimits[i]));
        }

        for(int i = 0; i < dq_calculated.size(); i++){
            velocity_scaled << (dq_calculated[i])/(DT-TAU) << ";";
        }
        velocity_scaled << endl;

        Q temp = device->getQ(state);
        // set the deltaQ to current configuration + the calculated adjustment
        dq = temp;
        dq += dq_calculated;
    }else if(POINTS == 1){
        Jacobian y(z_moore_penrose * stacked_dudv.e());
        Q dq_calculated(y.e());
        Q velocityLimits =  device->getVelocityLimits() ;

        dq_calculated = dq_calculated*(DT-TAU);
        for(int i = 0; i < dq_calculated.size();i++){
            dq_calculated[i] = std::min(dq_calculated[i],velocityLimits[i]);

    }
        Q temp = device->getQ(state);
        // set the deltaQ to current configuration + the calculated adjustment
        dq = temp;
        dq += dq_calculated;

    }

    return dq;
}
Jacobian SamplePlugin::stack_dudv(vector<Vector2D<double> > error_tracking){

    std::vector<Jacobian> jac;

    for(auto err : error_tracking){
        jac.push_back(Jacobian(err.e()));
    }
    // cast the tracked point(s) to a Jacobian and push it to our vector of jacobian - it will hold the stacked Jacobians, if we have more than 1 tracking point

    Jacobian stacked_du_single(2,1);
    Jacobian stacked_du_three(6,1);

    if(POINTS == 3){
        Jacobian stacked_jac(6,1);

       for(int i = 0, ii = 0; i < jac.size(); i++, ii+=2){
           for(int j = 0; j < 1; j++){
               stacked_jac.elem(ii,j) = jac[i].elem(0,j);
               stacked_jac.elem((ii+1),j) = jac[i].elem(1,j);
           }
       }
       stacked_du_three = stacked_jac;
       return stacked_du_three;

   }else if(POINTS == 1){
        Jacobian stacked_jac = jac[0];
        stacked_du_single = stacked_jac;
        return stacked_du_single;

    }

}

void SamplePlugin::print_all_data(vector<Vector2D<double> > track_error ){

    // plotting format: tracking error, joint variables, position, RPY

    // get joint variables and tool pose
    Q q_vec = device->getQ(state);
    //log().info() << q_vec << endl;

    Transform3D<> t_P(device->baseTframe(cameraFrame, state).P());

    RPY<> t_R(device->baseTframe(cameraFrame, state).R());
    //log().info() << t_P << endl;
    //log().info() << t_R << endl;


    // tracking error
    double err;
    for(int i = 0; i < track_error.size(); i++){
        err += abs(track_error[i][0]) + abs(track_error[i][1]);
    }
    err = err/POINTS;
    pose_config_data << err <<";";
    log().info() << "err: " << err << endl;


    // joint variables
    for(int i = 0; i < q_vec.size(); i++){
        pose_config_data << q_vec[i] << ";";
    }
    // position xyz
    for(int i = 0; i < t_P.P().size(); i++){
        pose_config_data << t_P.P()[i] <<";";
        //log().info() << t_P.P()[i] << endl;
    }
    // RPY
    for(int i = 0; i < t_R.size(); i++){
        pose_config_data << t_R[i] << ";";
        log().info() << "R: " << t_R[i] << endl;
    }

        pose_config_data << endl;

}

void SamplePlugin::stateChangedListener(const State& state) {
  _state = state;
}

std::vector<Vector2D<double> > SamplePlugin::get_marker_pts(Frame *marker_frame, Frame *camera_frame, int num_tracked_points){

    std::vector<Vector2D<double>> uv_points; // used to hold all points tracked

    Transform3D<> marker_transform = inverse(marker_frame->fTf(camera_frame, state)); // frame-to-frame transformation, used to find midpoint of marker in the workspace

    std::vector<Vector3D<> > pts_marker; // Vector3D used since we need to use 3D coordinates for tests (though the last is 0)

    if(num_tracked_points == 1){
        pts_marker.push_back(marker_transform * Vector3D<>(0,0,0));
    }else if (num_tracked_points == 3){
        pts_marker.push_back(marker_transform * Vector3D<>(0.15, 0.15, 0));
        pts_marker.push_back(marker_transform * Vector3D<>(-0.15,0.15, 0));
        pts_marker.push_back(marker_transform * Vector3D<>(0.15,-0.15, 0));
    }

    //log().info() << "marker points\n " << pts_marker[0] << endl;
    for(auto pt_marker : pts_marker){
        Vector2D<double> pt;
        pt[0] = f * pt_marker[0] / z; // using pinholde model to obtain image coordinates u,v
        pt[1] = f * pt_marker[1] / z;
        uv_points.push_back(pt);
     }
    //log().info() << "uv point\n " << uv_points[0] << endl;
    return uv_points;

}

// inspiration found at
// https://stackoverflow.com/questions/20756968/reading-multiple-lines-from-a-file-using-getline
std::vector<Transform3D<double> > SamplePlugin::read_file(std::string file_name){
    // here we can read the different motions files. Slow, normal fast files. Which one are loaded are hardcoded atm. in btn1 press
    RPY<double> rpy;
    Vector3D<double> xyz;
    std::vector<Transform3D<double> > motions; // Vector to be returned
    string row;
    double x, y, z, R, P, Y;
    ifstream ifstr(file_name);
    while(getline(ifstr,row))
    {
        stringstream input_stream(row); // Create a stream lines
        // Read from motions file
        input_stream >> x >> y >> z >> R >> P >> Y;

        rpy = RPY<double>(R,P,Y);   // RPY matrix
        xyz = Vector3D<double>(x,y,z);   //  vector for translation

        // Create matrix with rpy and translation. Vector is used for motions
        motions.push_back(Transform3D<double>(xyz,rpy.toRotation3D()));
    }
    ifstr.close();
    return motions;
}

void SamplePlugin::writeLog(int i){
    //log().info() << "size: " << i << endl;
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

Jacobian SamplePlugin::stack_jacobian(vector<Jacobian> jacb){

    if(POINTS == 3){
        Jacobian stacked_jack_three_point(6,7);
        Jacobian stacked_jac(jacb.size()*2,7);

       for(int i = 0, ii = 0; i < jacb.size(); i++, ii+=2){
           for(int j = 0; j < 7; j++){
               stacked_jac.elem(ii,j) = jacb[i].elem(0,j);
               stacked_jac.elem((ii+1),j) = jacb[i].elem(1,j);
           }
       }
       stacked_jack_three_point = stacked_jac;
       return stacked_jack_three_point;

   }else if(POINTS == 1){
        Jacobian stacked_jack_single_point(2,7);
        Jacobian stacked_jac = jacb[0];
        stacked_jack_single_point = stacked_jac;
        return stacked_jack_single_point;
    }
}

vector<Jacobian> SamplePlugin::getImageJacobian(vector<Vector2D<> > uv_pts) //
{
    //double x = xyz.x;//[0];
    //double y = xyz.y;//[1];
    //int z = xyz[2];
    vector<Jacobian> stacked_jacobians;

    for(auto jac : uv_pts){
        double u = jac[0];
        double v = jac[1];
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
        stacked_jacobians.push_back(Jimage);
    }
    return stacked_jacobians;
}

vector<Jacobian> SamplePlugin::calcZimage(const vector<Jacobian> Jimage, rw::models::Device::Ptr device, const Q q, rw::kinematics::State state)
{
    //device->setQ(q,state);
    vector<Jacobian> z_img;
    Rotation3D<> baseRtool = device->baseTend(state).R().inverse();
    Jacobian S(6,6);
    S.e().setZero();
    S.e().block<3,3>(0,0) = S.e().block<3,3>(3,3) = baseRtool.e();
    Jacobian J = device->baseJend(state);
    for(auto j_img : Jimage){
        z_img.push_back(j_img*S*J);
    }
    return z_img;
}

std::vector<Vector2D<double> > SamplePlugin::get_tracking_pts(Mat im, MovableFrame* marker){
    std::vector<Vector2D<double> >uv_points;
    vector<Point2f> vision_points;
    double x_diff = im.cols/2;
    double y_diff = im.rows/2;
    if(VISION){
        if(MARKER == 1){
            vision_points = getPoints(im);
        }
        else if(MARKER == 3){
            vision_points = getCornyPoints(img_corny,im);
        }
        for(auto vispt : vision_points){
            Vector2D<double> tmp;
            tmp[0] = vispt.x-x_diff;
            tmp[1] = vispt.y-y_diff;
            uv_points.push_back(tmp);
            log().info() << "start point: " << tmp << endl;
        }
    }else{
        uv_points = get_marker_pts(marker, cameraFrame, POINTS);
    }
    return uv_points;
}

vector<Point2f> SamplePlugin::getCornyPoints(Mat img_object, Mat img_scene)
{
    vector<Point2f> points;
    // Load images
    Mat descriptors_object;
    std::vector<cv::KeyPoint> keypoints_object;


    cv::Ptr<cv::Feature2D> detector;
    detector = cv::xfeatures2d::SURF::create();
    detector->detectAndCompute(img_object, cv::noArray(), keypoints_object, descriptors_object);



    // Construct detector
    //cv::Ptr<cv::Feature2D> detector;

    // 1. / 2. Detect keypoints and compute descriptors
    std::vector<cv::KeyPoint> keypoints_scene;
    Mat descriptors_scene;
    detector->detectAndCompute(img_scene, cv::noArray(), keypoints_scene, descriptors_scene);

    // Draw and show keypoints
    //cv::Mat img_keypoints_object;
   // cv::Mat img_keypoints_scene;
    //cv::drawKeypoints(img_object, keypoints_object, img_keypoints_object);
    //cv::drawKeypoints(img_scene, keypoints_scene, img_keypoints_scene);
    //cv::imshow("Keypoints 1", img_keypoints_object);
    //cv::imshow("Keypoints 2", img_keypoints_scene);

    // Construct matcher
    cv::Ptr<cv::DescriptorMatcher> matcher;

    matcher = cv::FlannBasedMatcher::create();

    // 3. / 4. Match descriptor vectors
    std::vector<cv::DMatch> matches;
    matcher->match(descriptors_object, descriptors_scene, matches);

    // Draw and show matches


    double max_dist = 0; double min_dist = 100;

      //-- Quick calculation of max and min distances between keypoints
      for( int i = 0; i < descriptors_object.rows; i++ )
      { double dist = matches[i].distance;
        if( dist < min_dist )
            min_dist = dist;
        if( dist > max_dist )
            max_dist = dist;
      }

       vector<DMatch> good_matches;
       for( int i = 0; i < descriptors_object.rows; i++ )
         { if( matches[i].distance < 3*min_dist )
            { good_matches.push_back( matches[i]); }
         }
       cv::Mat img_matches;
       cv::drawMatches(img_object, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches);
       //cv::imshow("Matches", img_matches);
       //imwrite("../../Final project report/filtered_matches_hard_sequence.png",img_matches);

       //-- Localize the object
         std::vector<Point2f> obj;
         std::vector<Point2f> scene;

         for( int i = 0; i < good_matches.size(); i++ )
         {
           //-- Get the keypoints from the good matches
           obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
           scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
         }

         Mat H = findHomography( obj, scene, CV_RANSAC );

         //-- Get the corners from the image_1 ( the object to be "detected" )
         std::vector<Point2f> obj_corners(4);
         obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
         obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
         std::vector<Point2f> scene_corners(4);

         perspectiveTransform( obj_corners, scene_corners, H);
         Scalar color1 = Scalar(255,0,0);
         Scalar color2 = Scalar(0,255,0);
         Scalar color3 = Scalar(0,0,255);
         Scalar color4 = Scalar(0,0,0);
         //-- Draw lines between the corners (the mapped object in the scene - image_2 )
         line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
         line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
         line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
         line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );

         //-- Show detected matches
         Point2f first;// = scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
        first = scene_corners[0];
         circle(img_scene,scene_corners[0],4,color1,-1,8,0);
         circle(img_scene,scene_corners[1],4,color2,-1,8,0);
         circle(img_scene,scene_corners[2],4,color3,-1,8,0);
         circle(img_scene,scene_corners[3],4,color4,-1,8,0);
         //imshow( "Good Matches & Object detection", img_matches );
         points.push_back(scene_corners[0]);
         points.push_back(scene_corners[1]);
         points.push_back(scene_corners[2]);
         imshow( "img", img_scene );
         log().info() << "size of corny points " <<points.size() << endl;
    return points;
}

vector<Point2f> SamplePlugin::getPoints(Mat src)
{
    Mat hsv;
    Mat dummy;
    cvtColor(src, src,CV_BGR2RGB);
    cvtColor( src, hsv, CV_BGR2HSV);
    Scalar blue_l(110,100,40);
    Scalar blue_h(140,255,255);
    //Scalar blue_h(114+10,157+100,60+100);
    Scalar red_l(0,100,100);
    Scalar red_h(20,255,255);
    //Rect r(630,200,30,30);
    //src(r) = 255;
    Mat bw,rw;
    vector<vector<Point> > contours_blue;
    vector<vector<Point> > contours_red;
    vector<Vec4i> hierarchy;
    inRange(hsv,blue_l,blue_h,bw);
    inRange(hsv,red_l,red_h,rw);
    findContours( bw, contours_blue, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    findContours( rw, contours_red, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    vector<Moments> mu;
    for(int i = 0; i < contours_blue.size();)
    {
        double density = (4*3.14* fabs(contourArea(cv::Mat(contours_blue[i]))))/pow(arcLength(contours_blue[i],false),2);
        if(  density < 0.7 || fabs(contourArea(cv::Mat(contours_blue[i]))) < 2500)
            contours_blue.erase(contours_blue.begin()+i);
        else
        {
             mu.push_back(moments(contours_blue[i],false));
            i++;
        }
    }
    for(int i = 0; i < contours_red.size();)
    {
        double density = (4*3.14* fabs(contourArea(cv::Mat(contours_red[i]))))/pow(arcLength(contours_red[i],false),2);
        if(  density < 0.63 || fabs(contourArea(cv::Mat(contours_red[i]))) < 2500)
            contours_red.erase(contours_red.begin()+i);
        else
        {
             mu.push_back(moments(contours_red[i],false));
             i++;
        }
    }


    //get centers
    vector<Point2f> mc(contours_blue.size()+contours_red.size());
    for(int i = 0;i < mu.size();i++)
        mc[i] = Point2f(mu[i].m10/mu[i].m00,mu[i].m01/mu[i].m00);

     Scalar color = Scalar( 255, 0, 0 );
     Scalar color2 = Scalar( 0, 0, 255 );

     Mat drawing = Mat::zeros( bw.size()+rw.size(), CV_8UC3 );

     for( int i = 0; i< contours_blue.size(); i++ )
     {
        drawContours( src, contours_blue, i, color2, 2, 8, hierarchy, 0, Point() );
        circle(src,mc[i],4,color2,-1,8,0);
     }
     for( int i = 0; i< contours_red.size(); i++ )
     {
        drawContours( src, contours_red, i, color, 2, 8, hierarchy, 0, Point() );
        circle(src,mc[mc.size()-1],4,color,-1,8,0);
     }
     for(int i =0;i < mc.size()-1;i++)
     {
         cout << norm(mc[mc.size()-1]- mc[i]) << " << " <<  norm(mc[mc.size()-1]- mc[i+1]) << endl;
         if(norm(mc[mc.size()-1]- mc[i]) < norm(mc[mc.size()-1]- mc[i+1]) || norm(mc[mc.size()-1]- mc[i+1]) == 0)
         {
             cout << "erase" << endl;
             mc.erase(mc.begin()+i);
             i--;
         }
         else
             mc.erase(mc.begin()+i+1);

     }
    // cout << mc.size() << endl;

     //circle(src,mc[0],4,0,-1,8,0);
     circle(src,mc[1],4,0,-1,8,0);
     Point2f mid((mc[0].x+mc[1].x)/2,(mc[0].y+mc[1].y)/2);
     mc.push_back(mid);

     circle(src,mc[0],4,0,-1,8,0);
     circle(src,mc[1],4,0,-1,8,0);
     circle(src,mc[2],4,0,-1,8,0);

      imshow("src",src);
     //imshow("blue"+str, drawing);
    return mc;

}

