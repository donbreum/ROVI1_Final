#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/shape.hpp>
#include "opencv2/imgcodecs.hpp"

#include "Vision.hpp"

#include <iostream>
#include <string>
#include <numeric>

using namespace cv;
using namespace std;


namespace
{
    // windows and trackbars name canny
    const std::string windowName = "Hough Circle Detection Demo";
    const std::string cannyThresholdTrackbarName = "Canny threshold";
    const std::string accumulatorThresholdTrackbarName = "Accumulator Threshold";
    const std::string usage = "Usage : tutorial_HoughCircle_Demo <path_to_input_image>\n";

    // initial and max values of the parameters of interests.
    const int cannyThresholdInitialValue = 100;
    const int accumulatorThresholdInitialValue = 50;
    const int maxAccumulatorThreshold = 200;
    const int maxCannyThreshold = 255;

    void HoughDetection(const Mat& src_gray, const Mat& src_display, int cannyThreshold, int accumulatorThreshold)
    {
        // will hold the results of the detection
        std::vector<Vec3f> circles;
        // runs the actual detection
        HoughCircles( src_gray, circles, HOUGH_GRADIENT, 1, src_gray.rows/8, cannyThreshold, accumulatorThreshold, 0, 0 );

        // clone the colour, input image for displaying purposes
        Mat display = src_display.clone();
        for( size_t i = 0; i < circles.size(); i++ )
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle( display, center, 3, Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            circle( display, center, radius, Scalar(0,0,255), 3, 8, 0 );
        }

        // shows the results
        imshow( windowName, display);
    }
}

void writeImage(Mat &image, String name){
    imwrite("images/" +name+".png", image);
}

void useTrackbar(Mat img, Mat img_gray){
    int cannyThreshold = cannyThresholdInitialValue;
int accumulatorThreshold = accumulatorThresholdInitialValue;

// create the main window, and attach the trackbars
namedWindow( windowName, WINDOW_AUTOSIZE );
createTrackbar(cannyThresholdTrackbarName, windowName, &cannyThreshold,maxCannyThreshold);
createTrackbar(accumulatorThresholdTrackbarName, windowName, &accumulatorThreshold, maxAccumulatorThreshold);

// infinite loop to display
// and refresh the content of the output image
// until the user presses q or Q
char key = 0;
while(key != 'q' && key != 'Q')
{
    // those paramaters cannot be =0
    // so we must check here
    cannyThreshold = std::max(cannyThreshold, 1);
    accumulatorThreshold = std::max(accumulatorThreshold, 1);

    //runs the detection, and update the display
    HoughDetection(img_gray, img, cannyThreshold, accumulatorThreshold);

    // get user key
    key = (char)waitKey(10);
}
}

int main(int argc, char* argv[]){

    // Parse command line arguments
    cv::CommandLineParser parser(argc, argv,
        "{help   |            | print this message}"
        "{@image1 | ../Vision/Marker1.ppm | image path}"
    );

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    // Load image
    string filename = parser.get<string>("@image1");
    Mat img = imread(filename);

    if (img.empty()) {
        std::cout << "Input image not found at '" << filename << "'\n";
        return 1;
    }

    writeImage(img,"circles drawed");
    cout << "wrote "<< endl;
    // inspiration found at opencv dovcs: https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/hough_circle/hough_circle.html
    // Convert it to gray
    Mat img_gray;
    cvtColor( img, img_gray, CV_BGR2GRAY );

     // Blur it to remove gaussian noise, thus reducing the chance of false circle detection
     GaussianBlur( img_gray, img_gray, Size(9, 9), 2, 2 );


//        cv::circle(display, center, 2, cv::Scalar(0, 255, 0), 5);
           Vision vis;
           cv::Point point;

           point = vis.getCenterPoint(img);

           Mat display = img.clone();
            cv::circle(display, point, 2, cv::Scalar(0, 0, 0), 5);

//        cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 20, 0, 0);
//        if(circles.size() == 0) std::exit(-1);
//        for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
//            cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
//            int radius = std::round(circles[current_circle][2]);

//            cv::circle(display1, center, radius, cv::Scalar(0, 255, 0), 5);
//            }

        writeImage(display,"circles drawed");


//     writeImage(img,"orignial");
//     writeImage(img_gray,"gray");
//     writeImage(display,"hough transform");



    return 0;
}


