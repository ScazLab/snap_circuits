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

#define NANOSVG_IMPLEMENTATION      // Expands nanosvg implementation
#define NANOSVGRAST_IMPLEMENTATION  // Expands nanosvg implementation

#include "StateEstimator.h"

using namespace std;
using namespace snapCircuits;

StateEstimator::StateEstimator(string _name) : rng(ros::Time::now().toSec()), board(),
                                               name(_name), imageTransport(nodeHandle)
{
    nodeHandle.param(("/"+name+"/show").c_str(), doShow, true);
    nodeHandle.param<std::string>(("/"+name+"/sub"  ).c_str(), sub,   "/snap_circuits/image_undistorted");
    nodeHandle.param<std::string>(("/"+name+"/pubIm").c_str(), pubIm, "/snap_circuits/board_estimated");
    nodeHandle.param<std::string>(("/"+name+"/pubSt").c_str(), pubSt, "/snap_circuits/board_state");

    imageSubscriber      = imageTransport.subscribe(sub.c_str(),1,&StateEstimator::callback, this);
    imagePublisher       = imageTransport.advertise(pubIm,1);
    boardStatePublisher  = nodeHandle.advertise<snap_circuits::snap_circuits_board>(pubSt,1);

    // Build an array of pegs for later use    
    for (int i = 0; i < snapCircuits::N_ROWS; ++i)
    {
        for (int j = 0; j < snapCircuits::N_COLS; ++j)
        {
            int x = X_OFFS+i*X_STEP;
            int y = Y_OFFS+j*Y_STEP;
            pegs.push_back(cv::Point2f(x,y));
        }
    }

    ROS_INFO("[StateEstimator] Name       set   to %s", name.c_str());
    ROS_INFO("[StateEstimator] Show param set   to %i", doShow);
    ROS_INFO("[StateEstimator] Subscribing      to %s", sub.c_str());
    ROS_INFO("[StateEstimator] Publishing Image to %s", pubIm.c_str());
    ROS_INFO("[StateEstimator] Publishing State to %s", pubSt.c_str());

    if (doShow)
    {
        ROS_INFO("[StateEstimator] Creating windows..");
        // cv::namedWindow("input",cv::WINDOW_NORMAL);
        cv::namedWindow("output",cv::WINDOW_NORMAL);
        cv::startWindowThread();

        // cv::moveWindow("input",100,50);
        cv::moveWindow("output",1050,50);
    }
};

void StateEstimator::callback(const sensor_msgs::ImageConstPtr& msgIn)
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
    img_in = cv_ptr->image.clone();
    cv::Mat img_out = img_in.clone();
    cv::Mat img_bw;
    cv::cvtColor(img_in,img_bw,CV_BGR2GRAY);

    // Filter the image in order to detect the colors
    img_bw = filterByColor(img_in);

    // Find the parts and their convex hulls
    vector<vector<cv::Point> > hulls = findPartsHull(img_bw,img_out);

    // Detect some information on the parts based on their hulls
    std::vector<part> parts = detectParts(hulls,pegs);

    // Create the board to be published
    snapCircuitsBoard board;
    createBoard(board,parts);

    if (board.getNParts()>0)
    {
        board.print(1);
        boardStatePublisher.publish(board.toMsg());
    }

    if (doShow)
    {
        drawPegs (pegs, img_out);
        drawHulls(hulls,img_out);
        
        // Show the image
        cv::imshow("output", img_out);
        cv::waitKey(5);
    }

    publishImage(img_out,sensor_msgs::image_encodings::BGR8);

    return;
};

bool StateEstimator::drawHulls(const vector<vector<cv::Point> > &hulls, cv::Mat &img)
{
    for (int i = 0; i < hulls.size(); ++i)
    {
        cv::Rect rect = cv::boundingRect(hulls[i]);
        cv::rectangle(img,rect,cv::Scalar(0,0,255),2);
    }

    return true;
}

bool StateEstimator::drawPegs(const vector<cv::Point> &pegs, cv::Mat &img)
{
    for (int i = 0; i < pegs.size(); ++i)
    {
        cv::circle(img, pegs[i], 3, CV_RGB(255,255,255), 1);
    }

    return true;
}

cv::Mat StateEstimator::filterByColor(const cv::Mat &in)
{
    cv::Mat out = cv::Mat::zeros(in.rows, in.cols, CV_8UC1);  // Destination image (BW)

    for (int i = 0; i < in.cols; ++i)
    {
        for (int j = 0; j < in.rows; ++j)
        {
            cv::Vec3b pixel = in.at<cv::Vec3b>(cv::Point(i,j));

            if (abs(pixel.val[0] - pixel.val[1]) > PXL_THRES ||
                abs(pixel.val[0] - pixel.val[2]) > PXL_THRES)
            {
                out.at<uchar>(j,i) = 255;
            }
        }
    }

    return out;
}

vector<vector<cv::Point> > StateEstimator::findPartsHull(const cv::Mat &in, cv::Mat &out)
{
    int largest_area=0;
    int largest_contour_index=0;

    out.resize(in.rows, in.cols);

    vector< vector<cv::Point> > contours; // Vector for storing contour
    vector< vector<cv::Point> > big_cont; // Vector for storing contour
    vector<cv::Vec4i> hierarchy;

    // First, let's remove the small blobs from the picture
    // Find the extreme outer contours in the image
    cv::findContours( in, contours, hierarchy, 0, 2 );
    
    std::vector<int> remove_idx;

    for( int i = 0; i< contours.size(); i++ ) {     // iterate through each contour. 
        double a = contourArea(contours[i],false);

        if (a > MIN_AREA)
        {
            big_cont.push_back(contours[i]);
        }
    }

    // Then, let's find the convex hull of the remaining blobs
    vector<vector<cv::Point> > hull(big_cont.size());
    
    for( size_t i = 0; i < big_cont.size(); i++ )
    {
        cv::convexHull(cv::Mat(big_cont[i]), hull[i], false );
    }

    // And finally let's draw everything on the output image
    for( size_t i = 0; i< big_cont.size(); i++ )
    {
        cv::Scalar color( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( out, hull, (int)i, color, CV_FILLED, 8, vector<cv::Vec4i>(), 0, cv::Point() );
        color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( out, big_cont, (int)i, color, 1, 8, vector<cv::Vec4i>(), 0, cv::Point() );
    }

    return big_cont;
}

std::vector< part > StateEstimator::detectParts(const vector< vector<cv::Point> > &hull, const vector<cv::Point> &pegs)
{
    std::vector< part > res;

    for (int i = 0; i < hull.size(); ++i)
    {
        cv::Rect rect = cv::boundingRect(hull[i]);

        // Find position on the board
        // The cast to double and back to int is to avoid some issues
        // at the top and left edges (whose index should be 0)
        int x = int((double(rect.x)-X_OFFS)/X_STEP + 1);
        int y = int((double(rect.y)-Y_OFFS)/Y_STEP + 1);

        // Find size
        int h = (int) (rect.height)/X_STEP + 1;
        int w = (int) (rect.width )/X_STEP + 1;

        // Find orientation: it can be either horizontal or vertical
        int o = h>w?90:0;

        // printf("Part %i: rect %i %i %i %i\t",i, rect.x, rect.y, rect.height, rect.width);
        // printf("position(x y) %i %i \torientation %i \tsize (h w) %i %i\n", x, y, o, h, w);

        res.push_back(part(x,y,o,h,w,hull[i]));
    }

    return res;
}

bool StateEstimator::createBoard(snapCircuits::snapCircuitsBoard &_b, const std::vector<part> &_p)
{
    for (int i = 0; i < _p.size(); ++i)
    {
        std::string label="";
        if (_p[i].h!=3 && _p[i].w!=3)
        {
            if (_p[i].h==1 || _p[i].w==1)
            {
                // This is most likely to be a connector
                int l=0;

                if (_p[i].h==1)
                {
                    l=_p[i].w;
                }
                else if (_p[i].w==1)
                {
                    l=_p[i].h;
                }

                if (l>=1 && l<=6) 
                {
                    label=int_to_string(l);
                }
            }
        }
        else if (_p[i].h==3 && _p[i].w==1 ||
                 _p[i].h==1 && _p[i].w==3 ||
                 _p[i].h==3 && _p[i].w==2 ||
                 _p[i].h==2 && _p[i].w==3   )
        {
            std::string color=detectPartColor(_p[i]);

            if      (_p[i].h==3 && _p[i].w==1 ||
                     _p[i].h==1 && _p[i].w==3   )
            {
                if      (color=="red")
                {
                    label="WC"; // This can be either D1 or WC
                }
                else if (color=="green")
                {
                    label="S1"; // This can be either S1 or S2
                }
                else if (color=="blue")
                {
                    label="3";
                }
            }
            else if (_p[i].h==3 && _p[i].w==2 ||
                     _p[i].h==2 && _p[i].w==3   )
            {
                if (color=="red")
                {
                    label="U2";
                }
                if (color=="green")
                {
                    label="U3";
                }
            }
        }
        else
        {
            ROS_WARN("[StateEstimator] Unexpected part on board!");
        }

        if (label!="")
        {
            snapCircuitsPart p(label);
            p.setLocation(_p[i].x,_p[i].y,_p[i].o);
            _b.addPart(p);

        }
    }
};

std::string StateEstimator::detectPartColor(part _p)
{
    cv::Mat mask    = cv::Mat::zeros(img_in.rows, img_in.cols, CV_8U);
    cv::Mat img_lab = cv::Mat::zeros(img_in.rows, img_in.cols, CV_8UC3);
    cv::cvtColor(img_in,img_lab, CV_BGR2HSV);
    
    vector<vector<cv::Point> > hull;
    hull.push_back(_p.hull);
    cv::drawContours(mask, hull, (int)0, cv::Scalar(255,255,255), CV_FILLED);

    cv::Scalar avg = cv::mean(img_lab,mask);

    if (avg[0]>=  0 && avg[0]< 70 || 
        avg[0]>=160 && avg[0]<179   )
    {
        return "red";
    }
    else if (avg[0]>= 70 && avg[0]< 100)
    {
        return "green";
    }
    else if (avg[0]>= 100 && avg[0]< 130)
    {
        return "blue";
    }

    return "";
};

bool StateEstimator::publishImage(cv::Mat &mat, const std::string encoding)
{
    cv_bridge::CvImage msgOut;
    msgOut.encoding = encoding;
    msgOut.image    = mat;

    imagePublisher.publish(msgOut.toImageMsg());
    return true;
}

StateEstimator::~StateEstimator()
{
    if (doShow)
    {
        ROS_INFO("[StateEstimator] Destroying windows..");
        // cv::destroyWindow("input");
        cv::destroyWindow("output");
    }
};
