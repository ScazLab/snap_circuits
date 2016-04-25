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

#include <pthread.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define MIN_ANGLE     5
#define MIN_DIST     15
#define OUT_IMG_H   400
#define OUT_IMG_W   571

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
bool isEqual(const cv::Vec2f& _l1, const cv::Vec2f& _l2);

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

    pthread_mutex_t mutex;

    /**
     * Callback on the subscriber's topic
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

    /**
     * Finds the biggest blob in the image (that should be black and white)
     * @param  mat the image as a cv::Mat
     * @return     the biggest blob as a cv::Mat. It is returned as a filled blob,
     *             without any "hole" inside it.
     */
    cv::Mat findBiggestBlob(cv::Mat & mat);

    /**
     * Finds the intersection between two lines.
     * @param  a the first  line as a Vec2f (the output of the Hough transform)
     * @param  b the second line as a Vec2f (the output of the Hough transform)
     * @return   the intersection point
     */
    cv::Point2f findIntersection(cv::Vec2f a, cv::Vec2f b);

    /**
     * Converts a Vec2f line into a pair of points.
     * @param   line The line as a Vec2f (the output of the Hough transform)
     * @return       The vector of points that define the line
     */
    vector<cv::Point2f> lineToPointPair(cv::Vec2f line);

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
    bool sortCorners(std::vector<cv::Point2f>& corners);
    
public:

    /**
     * Constructor
     */
    BoardCalibrator(string _name);

    /**
     * Destructor
     */
    ~BoardCalibrator();
};
