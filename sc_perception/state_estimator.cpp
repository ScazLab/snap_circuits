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

#define NANOSVG_IMPLEMENTATION      // Expands implementation
#define NANOSVGRAST_IMPLEMENTATION  // Expands implementation

#include <snapCircuits/utils.h>
#include <snapCircuits/snapCircuitsBoard.h>

#define PXL_THRES 20
#define MIN_AREA  300

#define X_OFFS 50
#define Y_OFFS 50
#define X_STEP 116
#define Y_STEP 116

using namespace std;
using namespace snapCircuits;

struct part
{
    int x;
    int y;
    int o;
    int h;
    int w;

    vector<cv::Point> hull;

    part(int _x, int _y, int _o, int _h, int _w, vector<cv::Point> _hull) :
            x(_x), y(_y), o(_o), h(_h), w(_w), hull(_hull) { };
}; 

class StateEstimator
{
private:
    std::string name;
    std::string  sub;
    std::string  pub;
    std::string filename;

    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber     imageSubscriber;
    image_transport::Publisher      imagePublisher;

    cv::RNG rng;

    bool doShow;

    cv::Mat filterByColor(const cv::Mat &in)
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

    vector<vector<cv::Point> > findParts(const cv::Mat &in, cv::Mat &out)
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

        for( size_t i = 0; i< big_cont.size(); i++ )
        {
            cv::Scalar color( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours( out, hull, (int)i, color, CV_FILLED, 8, vector<cv::Vec4i>(), 0, cv::Point() );
            color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours( out, big_cont, (int)i, color, 1, 8, vector<cv::Vec4i>(), 0, cv::Point() );
        }

        // cv::drawContours( out, big_cont, -1, cv::Scalar(255,255,255), CV_FILLED, 8);

        return hull;
    }

    std::vector< part > detectParts(const vector< vector<cv::Point> > &hull, const vector<cv::Point> &pegs)
    {
        std::vector< part > res;

        for (int i = 0; i < hull.size(); ++i)
        {
            cv::Rect rect = cv::boundingRect(hull[i]);

            // Find position on the board
            int x = (int) (rect.x-X_OFFS)/X_STEP + 1;
            int y = (int) (rect.y-Y_OFFS)/Y_STEP + 1;

            // Find size
            int h = (int) (rect.height)/X_STEP + 1;
            int w = (int) (rect.width )/X_STEP + 1;

            // Find orientation: it can be either horizontal or vertical
            int o = h>w?90:0;

            printf("Part %i: rect %i %i %i %i\tposition(x y) %i %i \torientation %i \tsize (h w) %i %i\n",
                                                 i, rect.x, rect.y, rect.height, rect.width, x,y,o,h,w);

            res.push_back(part(x,y,o,h,w,hull[i]));
        }

        return res;
    }

    bool createBoard(snapCircuits::snapCircuitsBoard &_b, const std::vector<part> &_p)
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
                        // label = int_to_string(l);
                    }

                }
            }
        }
    };

public:

    /**
     * Callback on the subscriber's topic. For now it opens a jpg file
     * @param msgIn an RGB image
     */
    void callback()
    {
        cv::Mat img_in, img_out, img_bw;
        img_in  = cv::imread(filename.c_str(), CV_LOAD_IMAGE_COLOR);
        img_out = img_in.clone();
        cv::cvtColor(img_in,img_bw,CV_BGR2GRAY);

        if(!img_in.data)                              // Check for invalid input
        {
            ROS_ERROR("Could not open or find the image");
            return;
        }

        // Build an array of pegs for later use
        std::vector<cv::Point> pegs;
        
        for (int i = 0; i < snapCircuits::N_ROWS; ++i)
        {
            for (int j = 0; j < snapCircuits::N_COLS; ++j)
            {
                int x = X_OFFS+i*X_STEP;
                int y = Y_OFFS+j*Y_STEP;
                pegs.push_back(cv::Point2f(x,y));
            }
        }

        // Filter the image in order to detect the colors
        img_bw = filterByColor(img_in);

        // Find the parts and their convex hulls
        vector<vector<cv::Point> > hull = findParts(img_bw,img_out);

        // Detect some information on the parts based on their hulls
        std::vector<part> parts = detectParts(hull,pegs);

        // Create the board to be published
        snapCircuitsBoard board;
        createBoard(board,parts);

        if (doShow)
        {
            // Draw the pegs
            for (int i = 0; i < pegs.size(); ++i)
            {
                cv::circle(img_out, pegs[i], 5, CV_RGB(255,255,255), 3);
            }

            for (int i = 0; i < hull.size(); ++i)
            {
                cv::Rect rect = cv::boundingRect(hull[i]);
                cv::rectangle(img_out,rect,cv::Scalar(0,0,255),3);
            }
            // Show the image
            cv::imshow("output", img_out);
        }
        cv::waitKey(0);                        // Wait for a keystroke in the window

        return;
    };

public:

    /**
     * Constructor
     */
    StateEstimator(string _name, string _filename) : rng(ros::Time::now().toSec()), name(_name), filename(_filename), imageTransport(nodeHandle)
    {
        nodeHandle.param(("/"+name+"/show").c_str(), doShow, true);
        nodeHandle.param<std::string>(("/"+name+"/sub").c_str(), sub, "/board_calibrator/image_undistorted");
        nodeHandle.param<std::string>(("/"+name+"/pub").c_str(), pub, "/state_estimator/board_state");

        // imageSubscriber = imageTransport.subscribe(sub.c_str(),1,&StateEstimator::callback, this);
        imagePublisher  = imageTransport.advertise(pub,1);

        ROS_INFO("[StateEstimator] Name       set to %s", name.c_str());
        ROS_INFO("[StateEstimator] Filename   set to %s", filename.c_str());
        ROS_INFO("[StateEstimator] Show param set to %i", doShow);
        ROS_INFO("[StateEstimator] Subscribing    to %s", sub.c_str());
        ROS_INFO("[StateEstimator] Publishing     to %s", pub.c_str());

        if (doShow)
        {
            ROS_INFO("[StateEstimator] Creating windows..");
            // cv::namedWindow("input",cv::WINDOW_NORMAL);
            cv::namedWindow("output",cv::WINDOW_NORMAL);
            cv::startWindowThread();

            // cv::moveWindow("input",100,50);
            cv::moveWindow("output",1050,50);

            cv::resizeWindow("output",857,600);
        }
    };

    /**
     * Destructor
     */
    ~StateEstimator()
    {
        if (doShow)
        {
            ROS_INFO("[StateEstimator] Destroying windows..");
            // cv::destroyWindow("input");
            cv::destroyWindow("output");
        }
    };
};

int main(int argc, char** argv)
{
    std::string name="state_estimator";
    std::string filename="/home/alecive/code/catkin_my_ws/src/snap_circuits/frame0005.jpg";

    ros::init(argc, argv, name.c_str());
    std::string sub = "/board_calibrator/image_undistorted";
    std::string pub = "/state_estimator/board_state";
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

    StateEstimator state_estimator(name,filename);
    // ros::spin();

    state_estimator.callback();

    return 0;
}
