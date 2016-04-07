/**
 * Copyright (C) 2016 Social Robotics Lab, Yale University
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@yale.edu
 * website: www.scazlab.yale.edu
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 3 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
**/

#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define MIN_ANGLE   5
#define MIN_DIST   15

using namespace std;
 

/**
 * This function detects if two lines are close (distance less then MIN_DIST)
 * and with the same angle (angle less then MIN_ANGLE). It is called by
 * the cv::threshold method. I wasn't able to put it as a member of the class
 * and I didn't want to waste time in understanding why :)
 * @param  _l1 the first  line as a Vec2f (the output of the Hough transform)
 * @param  _l2 the second line as a Vec2F (the output of the Hough transform)
 * @return     true/false if success/failure
 */
bool isEqual(const cv::Vec2f& _l1, const cv::Vec2f& _l2)
{
    cv::Vec2f l1(_l1), l2(_l2);

    float rho1   = l1[0];
    float theta1 = l1[1]*180/CV_PI;
    float rho2   = l2[0];
    float theta2 = l2[1]*180/CV_PI;

    float angle=abs(theta2-theta1);
    float dist=abs(rho1-rho2);

    ROS_DEBUG("[BoardCalibrator] Angle is %g\tDist is %g", angle, dist);

    if (angle > MIN_ANGLE || dist > MIN_DIST)
    {
        return false;
    }

    return true;
}

class BoardCalibrator
{
private:
    std::string name;
    std::string  sub;
    std::string  pub;

    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber     imageSubscriber;
    image_transport::Publisher      imagePublisher;

    bool doShow;

    /**
     * Callback on the subscriber's topic
     * @param msgIn an RGB image
     */
    void callback(const sensor_msgs::ImageConstPtr& msgIn)
    {
        // Let's convert the ROS image to OpenCV image format
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msgIn, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Convert the image to black and white
        cv::Mat img_bw;
        cv::Mat img_res = cv_ptr->image.clone();
        cv::cvtColor(cv_ptr->image,img_bw,CV_BGR2GRAY);

        // Threshold the image to get only the black parts
        cv::threshold(img_bw,img_bw,150,200, cv::THRESH_BINARY_INV+cv::THRESH_OTSU);

        // Find the biggest blob in the image (hopefully, the board), and fill it to remove the hexagons
        img_bw=findBiggestBlob(img_bw);
        
        if (doShow)
        {
            cv::imshow("camera_thresholded",img_bw);
        }

        // Get the edge map for finding line segments with the Canny method.
        cv::Canny(img_bw, img_bw, 100, 200*3, 3);

        // Detect lines with the Hough transform method.
        // We use the probabilistic Hough transform rather than the standard one.
        // It yields less line segments but with higher accuracy.
        vector<cv::Vec2f> lines;
        HoughLines(img_bw, lines, 1, CV_PI/180, 100, 0, 0 );

        // Filter out lines that are close and parallel
        std::vector<int> labels;
        int numberOfLines = cv::partition(lines, labels, isEqual);

        int numberOfClusters=0;
        for (int i = 0; i < labels.size(); ++i)
        {
            numberOfClusters=max(numberOfClusters, labels[i]);
        }
        numberOfClusters++;

        ROS_INFO("[BoardCalibrator] Number of lines: %zu\tNumber of clusters: %i ", lines.size(), numberOfClusters);
        
        if (numberOfClusters >= 4)
        {
            // Let's keep only one line per cluster. This keeps only the first line of the 
            // cluster, it should be made better
            
            std::vector<cv::Vec2f> lines_filt;
            int cluster=0;
            for (int i = 0; i < lines.size(); ++i)
            {
                if (labels[i]==cluster)
                {
                    lines_filt.push_back(lines[i]);
                    cluster++;
                }
            }

            // Find the intersection between the lines
            std::vector<cv::Point2f> corners;
            for (int i = 0; i < lines.size(); i++)
            {
                for (int j = i+1; j < lines.size(); j++)
                {
                    cv::Point2f pt = findIntersection(lines_filt[i], lines_filt[j]);
                    if (pt.x > 0 && pt.y > 0 && pt.x < img_res.cols && pt.y < img_res.rows)
                        corners.push_back(pt);
                }
            }

            // printf("[BoardCalibrator] corners: ");
            // for (int i = 0; i < corners.size(); ++i)
            // {
            //     printf("[%g %g]\t", corners[i].x, corners[i].y );
            // }
            // printf("\n");

            if (doShow)
            {
                cv::cvtColor(img_bw,img_bw,CV_GRAY2RGB);

                // Draw the lines
                for( size_t i = 0; i < lines_filt.size(); i++ )
                {
                    float rho   = lines_filt[i][0];
                    float theta = lines_filt[i][1];
                    cv::Point pt1;
                    cv::Point pt2;
                    double a = cos(theta);
                    double b = sin(theta);
                    double x0 = a*rho;
                    double y0 = b*rho;
                    pt1.x = cvRound(x0 + 1000*(-b));
                    pt1.y = cvRound(y0 + 1000*(a));
                    pt2.x = cvRound(x0 - 1000*(-b));
                    pt2.y = cvRound(y0 - 1000*(a));
                    cv::line( img_bw, pt1, pt2, cv::Scalar(0,255,0), 2, CV_AA);
                }

                // Draw corners
                for (int i = 0; i < corners.size(); i++)
                {
                    cv::circle(img_bw, corners[i], 3, CV_RGB(255,255,255), 2);
                }
                
                cv::imshow("camera_edges",img_bw);
            }

            if (corners.size()==4)
            {
                cv::Mat quad = cv::Mat::zeros(800, 1143, CV_8UC3);  // Destination image
                
                // Determine top-left, bottom-left, top-right, and bottom-right corner
                sortCorners(corners);

                // Apply the perspective transformation

                // Corners of the destination image
                std::vector<cv::Point2f> quad_pts;
                quad_pts.push_back(cv::Point2f(0, 0));
                quad_pts.push_back(cv::Point2f(quad.cols, 0));
                quad_pts.push_back(cv::Point2f(quad.cols, quad.rows));
                quad_pts.push_back(cv::Point2f(0, quad.rows));

                // Get transformation matrix
                cv::Mat transmtx = cv::getPerspectiveTransform(corners, quad_pts);

                // Apply perspective transformation
                cv::warpPerspective(cv_ptr->image, quad, transmtx, quad.size());

                if (doShow)
                {
                    // cv::imshow("image_undistorted",quad);
                }

                cv_bridge::CvImage msgOut;
                // msgOut.header   = msgIn->header; // Same timestamp as input image
                msgOut.encoding = sensor_msgs::image_encodings::BGR8;
                msgOut.image    = quad;

                imagePublisher.publish(msgOut.toImageMsg());
            }
        }

    };

    /**
     * Finds the intersection between two lines.
     * @param  a the first  line as a Vec2f (the output of the Hough transform)
     * @param  b the second line as a Vec2f (the output of the Hough transform)
     * @return   the intersection point
     */
    cv::Point2f findIntersection(cv::Vec2f a, cv::Vec2f b)
    {
        vector<cv::Point2f> p1 = lineToPointPair(a);
        vector<cv::Point2f> p2 = lineToPointPair(b);

        float denom = (p1[0].x - p1[1].x)*(p2[0].y - p2[1].y) - (p1[0].y - p1[1].y)*(p2[0].x - p2[1].x);

        return cv::Point2f(((p1[0].x*p1[1].y - p1[0].y*p1[1].x)*(p2[0].x - p2[1].x) -
                            (p1[0].x - p1[1].x)*(p2[0].x*p2[1].y - p2[0].y*p2[1].x)) / denom,
                           ((p1[0].x*p1[1].y - p1[0].y*p1[1].x)*(p2[0].y - p2[1].y) -
                            (p1[0].y - p1[1].y)*(p2[0].x*p2[1].y - p2[0].y*p2[1].x)) / denom);

    }

    /**
     * Converts a Vec2f line into a pair of points.
     * @param   line The line as a Vec2f (the output of the Hough transform)
     * @return       The vector of points that define the line
     */
    vector<cv::Point2f> lineToPointPair(cv::Vec2f line)
    {
        vector<cv::Point2f> points;

        float r = line[0], t = line[1];
        double cos_t = cos(t), sin_t = sin(t);
        double x0 = r*cos_t, y0 = r*sin_t;
        double alpha = 1000;

        points.push_back(cv::Point2f(x0 + alpha*(-sin_t), y0 + alpha*cos_t));
        points.push_back(cv::Point2f(x0 - alpha*(-sin_t), y0 - alpha*cos_t));

        return points;
    }

    /**
     * Determine the top-left, bottom-left, top-right, and bottom-right corners.
     * It assumes to have 4 corners. It does the following:
     *  1. Computes the mass center.
     *  2. Points that have lower y-axis than the mass center are the top points.
     *  3. Given two top points, the one with lower x-axis is the top-left.
     *  4. Given two bottom points, apply the same rule as #3
     * @param corners the corners under consideration ()
     * @return        true/false if success/failure
     */
    bool sortCorners(std::vector<cv::Point2f>& corners)
    {
        if (corners.size() != 4)
        {
            ROS_ERROR("[BoardCalibrator::sortCorners] The number of corners should be 4.");
            return false;
        }

        // 1. Compute the mass center
        cv::Point2f center(0,0);
        for (int i = 0; i < corners.size(); i++)
            center += corners[i];

        center *= (1. / corners.size());
        
        // 2. Points that have lower y-axis than the mass center are the top points.
        std::vector<cv::Point2f> top, bot;

        for (int i = 0; i < corners.size(); i++)
        {
            if (corners[i].y < center.y)
                top.push_back(corners[i]);
            else
                bot.push_back(corners[i]);
        }

        // 3. Given two top points, the one with lower x-axis is the top-left.
        cv::Point2f tl = top[0].x > top[1].x ? top[1] : top[0];
        cv::Point2f tr = top[0].x > top[1].x ? top[0] : top[1];

        // 4. Given two bottom points, apply the same rule as #3
        cv::Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
        cv::Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];

        corners.clear();
        corners.push_back(tl);
        corners.push_back(tr);
        corners.push_back(br);
        corners.push_back(bl);

        return true;
    }

    /**
     * Finds the biggest blob in the image (that should be black and white)
     * @param  mat the image as a cv::Mat
     * @return     the biggest blob as a cv::Mat. It is returned as a filled blob,
     *             without any "hole" inside it.
     */
    cv::Mat findBiggestBlob(cv::Mat & mat)
    {
        int largest_area=0;
        int largest_contour_index=0;

        cv::Mat res = cv::Mat::zeros(mat.rows, mat.cols, CV_8UC3);

        vector< vector<cv::Point> > contours; // Vector for storing contour
        vector<cv::Vec4i> hierarchy;

        cv::findContours( mat, contours, hierarchy, 0, 2 ); // Find the extreme outer contours in the image

        for( int i = 0; i< contours.size(); i++ ) {     // iterate through each contour. 
            double a=contourArea( contours[i],false);   //  Find the area of contour
            if(a>largest_area){
                largest_area=a;
                largest_contour_index=i;                //Store the index of largest contour
            }
        }

        cv::drawContours( res, contours, largest_contour_index, cv::Scalar(255,255,255), CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.
        return res;
    }

public:

    /**
     * Constructor
     */
    BoardCalibrator(string _name) : name(_name), imageTransport(nodeHandle)
    {
        nodeHandle.param(("/"+name+"/show").c_str(), doShow, true);
        nodeHandle.param<std::string>(("/"+name+"/sub").c_str(), sub, "/usb_cam/image_raw");
        nodeHandle.param<std::string>(("/"+name+"/pub").c_str(), pub, "/board_calibrator/image_undistorted");

        imageSubscriber = imageTransport.subscribe(sub.c_str(),1,&BoardCalibrator::callback, this);
        imagePublisher  = imageTransport.advertise(pub,1);

        ROS_INFO("[BoardCalibrator] Show param set to %i", doShow);
        ROS_INFO("[BoardCalibrator] Subscribing    to %s", sub.c_str());
        ROS_INFO("[BoardCalibrator] Publishing     to %s", pub.c_str());

        if (doShow)
        {
            ROS_INFO("[BoardCalibrator] Creating windows..");
            // cv::namedWindow("image_undistorted");
            cv::namedWindow("camera_thresholded");
            cv::namedWindow("camera_edges");
            cv::startWindowThread();

            cv::moveWindow("camera_thresholded",3000,50);
            cv::moveWindow("camera_edges",3000,600);
            // cv::moveWindow("image_undistorted",3600,200);
        }
    };

    /**
     * Destructor
     */
    ~BoardCalibrator()
    {
        if (doShow)
        {
            ROS_INFO("[BoardCalibrator] Destroying windows..");
            // cv::destroyWindow("image_undistorted");
            cv::destroyWindow("camera_thresholded");
            cv::destroyWindow("camera_edges");
        }
    };
};

int main(int argc, char** argv)
{
    std::string name="board_calibrator";

    ros::init(argc, argv, name.c_str());
    std::string sub = "/usb_cam/image_raw";
    std::string pub = "/board_calibrator/image_undistorted";
    bool show=false;

    // Dirty way to process command line arguments. It seems that
    // there is not a straightforward standard ROS way, unfortunately.
    if (argc>1)
    {
        ros::NodeHandle nH;

        if (std::string(argv[1])=="--show")
        {
            show=std::string(argv[2])=="true"?true:false;
            nH.setParam(("/"+name+"/show").c_str(), show);
        }

        if (argc>3)
        {
            sub=std::string(argv[3]);
            nH.setParam( ("/"+name+"/sub").c_str(), sub.c_str());
        
            if (argc>4)
            {
                pub=std::string(argv[4]);
                nH.setParam( ("/"+name+"/pub").c_str(), pub.c_str());
            }
        }
    }

    BoardCalibrator board_calibrator(name);
    ros::spin();

    return 0;
}

