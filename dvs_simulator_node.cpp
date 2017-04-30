#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d.hpp"
#include <math.h>
#include <cmath>

using namespace cv;

//Parameters
//============================================
//============================================

//============================================
//============================================

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
static const char WINDOW[] = "Image 1";
 
image_transport::Publisher pub;
//cv_bridge::CvImagePtr oldImage(new cv_bridge::CvImage);
 
double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 ) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void find_squares(Mat& image, std::vector<std::vector<Point> >& squares)
{
    // blur will enhance edge detection
    Mat blurred(image);
    medianBlur(image, blurred, 9);

    Mat gray0(blurred.size(), CV_8U), gray;
    std::vector<std::vector<Point> > contours;

    // find squares in every color plane of the image
    for (int c = 0; c < 3; c++)
    {
        int ch[] = {c, 0};
        mixChannels(&blurred, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        const int threshold_level = 2;
        for (int l = 0; l < threshold_level; l++)
        {
            // Use Canny instead of zero threshold level!
            // Canny helps to catch squares with gradient shading
            if (l == 0)
            {
                Canny(gray0, gray, 10, 20, 3); // 

                // Dilate helps to remove potential holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1));
            }
            else
            {
                    gray = gray0 >= (l+1) * 255 / threshold_level;
            }

            // Find contours and store them in a list
            findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

            // Test contours
            std::vector<Point> approx;
            for (size_t i = 0; i < contours.size(); i++)
            {
                    // approximate contour with accuracy proportional
                    // to the contour perimeter
                    approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

                    // Note: absolute value of an area is used because
                    // area may be positive or negative - in accordance with the
                    // contour orientation
                    if (approx.size() == 4 &&
                            fabs(contourArea(Mat(approx))) > 1000 &&
                            isContourConvex(Mat(approx)))
                    {
                            double maxCosine = 0;

                            for (int j = 2; j < 5; j++)
                            {
                                    double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                                    maxCosine = MAX(maxCosine, cosine);
                            }

                            if (maxCosine < 0.3)
                                    squares.push_back(approx);
                    }
            }
        }
    }
}

void drawSquares( Mat& image, const std::vector<std::vector<Point> >& squares )
{
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, CV_AA);
    }

    imshow(WINDOW, image);
}

void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
	ROS_ERROR("In");
	
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    Mat gray;
    std::vector<Vec3f> circles;
    //cvSmooth( cv_ptr->image, cv_ptr->image, CV_GAUSSIAN, 3, 0, 0, 0);
    cvtColor(cv_ptr->image, gray, CV_RGB2GRAY);
    /*Canny(edges,edges,100,150,5);
    std::vector<Vec2f> lines;
    HoughLines(edges, lines, 1, CV_PI/180, 150, 0, 0 );
    for( size_t i = 0; i < lines.size(); i++ )
    {

        float rho = lines[i][0], theta = lines[i][1];

                if( theta>CV_PI/180*170 || theta<CV_PI/180*10)
        {
        Point pt1, pt2;

        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;

        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        if(abs(pt2.x-pt1.x)<1)

        	line( cv_ptr->image, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
        }

    }*/
    adaptiveThreshold(gray, gray, 255, 0, 0, 51, -25); 
	/*HoughCircles( gray, circles, CV_HOUGH_GRADIENT, 2, 10, 100, 25, 9, 16 );
	for( size_t i = 0; i < circles.size(); i++ )
  {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // circle center
      circle( cv_ptr->image, center, 3, Scalar(0,255,0), -1, 8, 0 );
      // circle outline
      circle( cv_ptr->image, center, radius, Scalar(0,0,255), 3, 8, 0 );
   }*/
    /*SimpleBlobDetector detector;

	std::vector<KeyPoint> keypoints;
	detector.detect( gray, keypoints);*/
	/*
	std::vector<KeyPoint> keypoints;
	cv::Ptr<cv::BRISK> ptrBrisk = cv::BRISK::create();
	ptrBrisk->detect(cv_ptr->image, keypoints);

	drawKeypoints( cv_ptr->image, keypoints, cv_ptr->image, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );*/
	std::vector<std::vector<Point> > squares;
	find_squares(cv_ptr->image, squares);
	drawSquares( cv_ptr->image,  squares );


    //imshow("source", cv_ptr->image);
    // detect lines
    
	imshow(WINDOW, cv_ptr->image);
	cv::waitKey(3);

	pub.publish(cv_ptr->toImageMsg());

	//oldImage->image=cv_ptr->image;
}


int main(int argc, char **argv)
{
   
    ros::init(argc, argv, "DVS");

    ros::NodeHandle nh;

    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);

    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);


    image_transport::Subscriber sub = it.subscribe("/cv_camera/image_raw", 1,   imageCallback);


    cv::destroyWindow(WINDOW);

    pub = it.advertise("DVS/image", 100);


    ros::spin();
 
}

