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

#include <snapCircuits/snapCircuitsBoard.h>
#include <snap_circuits/snap_circuits_board.h> 

#define PXL_THRES 20
#define MIN_AREA  300

#define X_OFFS 25
#define Y_OFFS 25
#define X_STEP 58
#define Y_STEP 58

struct part
{
    int x;
    int y;
    int o;
    int h;
    int w;

    std::vector<cv::Point> hull;

    part(int _x, int _y, int _o, int _h, int _w, std::vector<cv::Point> _hull) :
                        x(_x), y(_y), o(_o), h(_h), w(_w), hull(_hull) { };
}; 

class StateEstimator
{
private:
    std::string name;
    std::string  sub;
    std::string  pubIm;
    std::string  pubSt;

    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber     imageSubscriber;
    image_transport::Publisher      imagePublisher;
    
    ros::Publisher boardStatePublisher;

    cv::RNG rng;
    cv::Mat img_in;

    bool doShow;

    cv::Mat filterByColor(const cv::Mat &in);

    std::vector<std::vector<cv::Point> > findPartsHull(const cv::Mat &in, cv::Mat &out);

    std::vector< part > detectParts(const std::vector< std::vector<cv::Point> > &hull, const std::vector<cv::Point> &pegs);

    bool createBoard(snapCircuits::snapCircuitsBoard &_b, const std::vector<part> &_p);

    std::string detectPartColor(part _p);

    /**
     * Callback on the subscriber's topic.
     * @param msgIn an RGB image
     */
    void callback(const sensor_msgs::ImageConstPtr& msgIn);

    /**
     * Publishes the image on the topic.
     * @param mat       the image as cv::Mat
     * @param encoding  the image encoding
     * @return          true/false if success/failure
     */
    bool publishImage(cv::Mat &mat, const std::string encoding);

public:

    /**
     * Constructor 
     */
    StateEstimator(std::string _name);

    /**
     * Destructor
     */
    ~StateEstimator();
};

