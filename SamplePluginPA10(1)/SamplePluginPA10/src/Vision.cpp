#include "Vision.hpp"
#include "SamplePlugin.hpp"

struct sortclass {
    bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.x < pt2.x);}
} myobj;

void Vision::writeImage(Mat &image, String name){
    imwrite("images/" +name+".png", image);
    cout << "writeimage  " << endl;
}

cv::Mat Vision::getImage(){
    return imgDraw;
}

cv::Point Vision::getCenterPoint(cv::Mat inputImg){

            Mat fix_c;
            cvtColor(inputImg, fix_c,CV_BGR2RGB);
    // https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/
            Mat red_hue_image,blue_hue_image,imgHSV;
            Mat img = inputImg.clone();
            Mat display = img.clone();
            //cv::medianBlur(img, img, 3);
            cvtColor(inputImg,imgHSV,CV_RGB2HSV);//CV_BGR2HSV); // color need to be RGB on robwork??

            //Scalar blueThresholdLOW(110,60,40);
            Scalar blueThresholdLOW(110,100,40);

            //Scalar blueThresholdHIGH(130,190,150);
            Scalar blueThresholdHIGH(140,255,255);

            //Scalar redThresholdLOW(0,150,120);
            Scalar redThresholdLOW(0,100,100);


            //Scalar redThresholdHIGH(45,230,220);
            Scalar redThresholdHIGH(10,255,255);

            inRange(imgHSV,blueThresholdLOW,blueThresholdHIGH,blue_hue_image);
            inRange(imgHSV,redThresholdLOW,redThresholdHIGH,red_hue_image);

            cv::Mat hue_image;
            //cv::addWeighted(blue_hue_image, 1.0, red_hue_image, 1.0, 0.0, hue_image);


            //cv::GaussianBlur(hue_image, hue_image, cv::Size(9, 9), 2, 2);
            cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);
            cv::GaussianBlur(blue_hue_image, blue_hue_image, cv::Size(9, 9), 2, 2);
            // Use the Hough transform to detect circles in the combined threshold image
            std::vector<cv::Vec3f> circles;
            cv::HoughCircles(blue_hue_image, circles, CV_HOUGH_GRADIENT, 1, blue_hue_image.rows/8, 100, 20, 0, 0);
            std::vector<cv::Point> ptsBLUE(circles.size());
            cout << "circle size; " << circles.size() << endl;
            // Loop over all detected circles and outline them on the original image
            if(circles.size() == 0) std::exit(-1);
            for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
                cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
                int radius = std::round(circles[current_circle][2]);

                cv::circle(display, center, radius, cv::Scalar(0, 0, 0), 5);
                cv::circle(display, center, 3, cv::Scalar(0, 0, 0), 5);
                cout << "[" << current_circle << "]" <<" circles center: " << center << endl; // display center point for circles detected

                ptsBLUE[current_circle] = Point(circles[current_circle][0],circles[current_circle][1]);

                }

            std::vector<cv::Vec3f> circles1;
            cv::HoughCircles(red_hue_image, circles1, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 20, 0, 0);
            std::vector<cv::Point> ptsRED(circles1.size());
            cout << "circle size; " << circles1.size() << endl;
            // Loop over all detected circles and outline them on the original image
            if(circles1.size() == 0) std::exit(-1);
            for(size_t current_circle = 0; current_circle < circles1.size(); ++current_circle) {
                cv::Point center(std::round(circles1[current_circle][0]), std::round(circles1[current_circle][1]));
                int radius = std::round(circles1[current_circle][2]);

                cv::circle(display, center, radius, cv::Scalar(0, 0, 0), 5);
                cv::circle(display, center, 3, cv::Scalar(0, 100,100), 5);
                cout << "[" << current_circle << "]" <<" circles center: " << center << endl; // display center point for circles detected

                ptsRED[current_circle] = Point(circles1[current_circle][0],circles1[current_circle][1]);

                }

            cout << "pts size: " << ptsBLUE.size() << endl;
            writeImage(display,"disp");

            SamplePlugin sp;

            imgDraw = display.clone();

            cv::Point center = findMidPoint(ptsRED,ptsBLUE);
            cv::circle(imgDraw, center, 4, cv::Scalar(0, 0, 0), 5);

            //cv::line(imgDraw,ptsBLUE[0],ptsBLUE[2],Scalar(0,0,0),5);
            //sp.writeLog(center);
            return center;
}

cv::Point Vision::findMidPoint(std::vector<cv::Point> red,std::vector<cv::Point> blue){

    // sort vector using myobject as comparator
//    std::sort(pts.begin(), pts.end(), myobj);

//    cv::Point center = (pts[2]+pts[0])/2;

//    cout << "center: " << center << endl;
    int index =0;
    double biggest =0.0;
    SamplePlugin sp;
    sp.writeLog(blue.size());
    for(int i  = 0; i < blue.size();i++){
          if(cv::norm(red[0]-blue[i]) > biggest){
              index = i;
              biggest = cv::norm(red[0]-blue[i]);
          }

    }

    cv::Point center = (red[0]+blue[index])/2;

    return center;

}
