#include "Vision.hpp"

struct sortclass {
    bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.x < pt2.x);}
} myobj;

void Vision::writeImage(Mat &image, String name){
    imwrite("images/" +name+".png", image);
    cout << "writeimage  " << endl;
}

cv::Point Vision::getCenterPoint(cv::Mat inputImg){


    // https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/
            Mat red_hue_image,blue_hue_image,imgHSV;
            Mat img = inputImg.clone();
            Mat display = img.clone();
            cv::medianBlur(img, img, 3);
            cvtColor(img,imgHSV,CV_BGR2HSV);
            //Scalar blueThresholdLOW(110,50,40);
            Scalar blueThresholdLOW(110,100,40);
            //Scalar blueThresholdHIGH(130,190,150);
            Scalar blueThresholdHIGH(140,255,255);
            //Scalar redThresholdLOW(0,130,100)
            Scalar redThresholdLOW(0,100,100);
            Scalar redThresholdHIGH(10,255,255);
            //Scalar redThresholdHIGH(45,210,210);//(10,255,255);
            inRange(imgHSV,blueThresholdLOW,blueThresholdHIGH,blue_hue_image);
            inRange(imgHSV,redThresholdLOW,redThresholdHIGH,red_hue_image);

            cv::Mat hue_image;
            cv::addWeighted(blue_hue_image, 1.0, red_hue_image, 1.0, 0.0, hue_image);

            cv::GaussianBlur(hue_image, hue_image, cv::Size(9, 9), 2, 2);
            //cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);
            // Use the Hough transform to detect circles in the combined threshold image
            std::vector<cv::Vec3f> circles;
            cv::HoughCircles(hue_image, circles, CV_HOUGH_GRADIENT, 1, hue_image.rows/8, 100, 20, 0, 0);
            std::vector<cv::Point> pts(circles.size());
            cout << "circle size; " << circles.size() << endl;
            // Loop over all detected circles and outline them on the original image
            if(circles.size() == 0) std::exit(-1);
            for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
                cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
                int radius = std::round(circles[current_circle][2]);

                cv::circle(display, center, radius, cv::Scalar(0, 0, 0), 5);
                cout << "[" << current_circle << "]" <<" circles center: " << center << endl; // display center point for circles detected

                pts[current_circle] = Point(circles[current_circle][0],circles[current_circle][1]);

                }

            cout << "pts size: " << pts.size() << endl;
            writeImage(display,"disp");

            cv::Point center = findMidPoint(pts);
            return center;
}

cv::Point Vision::findMidPoint(std::vector<cv::Point> pts){

    // sort vector using myobject as comparator
    std::sort(pts.begin(), pts.end(), myobj);

    cv::Point center = (pts[3]+pts[0])/2;

    cout << "center: " << center << endl;

        cout << "dis: " << cv::norm(pts[0]-pts[1]) << endl;
        cout << "dis: " << cv::norm(pts[0]-pts[2]) << endl;
        cout << "dis: " << cv::norm(pts[0]-pts[3]) << endl;
        cout << "dis: " << cv::norm(pts[1]-pts[2]) << endl;
       cout << "dis: " << cv::norm(pts[2]-pts[3]) << endl;



    return center;

}
