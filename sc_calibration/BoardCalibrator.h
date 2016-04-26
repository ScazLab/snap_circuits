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

#include "Filters.h"

#define MIN_ANGLE     5
#define MIN_DIST     15
#define OUT_IMG_H   400
#define OUT_IMG_W   571
#define FILT_WINDOW   7

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

/**
 * BoardCalibrator class.
 */
class BoardCalibrator
{
private:
    std::string name;   // name
    std::string  sub;   // topic to be subscribed to
    std::string  pub;   // topic to publish to

    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber     imageSubscriber;
    image_transport::Publisher      imagePublisher;

    bool doShow;

    pthread_mutex_t mutex;

    std::vector<cvPointFilter> corners;

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
     * Cluster lines that are relatively close and parallel one another
     * (according to MIN_DIST and MIN_ANGLE respectively).
     * @param  lines the lines under consideration
     * @return       true/false if success/failure
     */
    bool clusterLines(std::vector<cv::Vec2f> &lines);

    /**
     * Finds the intersection between two lines.
     * @param  _l1 the first  line as a Vec2f (the output of the Hough transform)
     * @param  _l2 the second line as a Vec2f (the output of the Hough transform)
     * @return     the intersection point
     */
    cv::Point2f findIntersection(cv::Vec2f _l1, cv::Vec2f _l2);

    /**
     * Converts a Vec2f line into a pair of points.
     * @param   line The line as a Vec2f (the output of the Hough transform)
     * @return       The vector of points that define the line
     */
    std::vector<cv::Point2f> lineToPointPair(cv::Vec2f line);

    /**
     * Determine the top-left, bottom-left, top-right, and bottom-right corners.
     * It assumes to have 4 corners. It does the following:
     *  1. Computes the mass center.
     *  2. Points that have lower y-axis than the mass center are the top points.
     *  3. Given two top points, the one with lower x-axis is the top-left.
     *  4. Given two bottom points, apply the same rule as #3
     * @param  _c the corners under consideration
     * @return    true/false if success/failure
     */
    bool sortCorners(std::vector<cv::Point2f> &_c);

    /**
     * @brief Creates the output picture for debugging purposes
     * @details This function creates a b/w image with the result of the Canny
     * filtering on the thresholded input image, plus the lines that should represent
     * the board, plus their intersections (i.e. the board corners)
     * 
     * @param img_bw  the image to draw the graphics on
     * @param lines   the std::vector of cv::Vec2f lines
     * @param corners the std::vector of cv::Point2f which represent the intersection between the lines
     * 
     * @return true/false if success/failure
     */                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
    bool drawCameraLinesCorners(cv::Mat img_bw, std::vector<cv::Vec2f> lines, std::vector<cv::Point2f> corners);

    /**
     * @brief   Finds corners between a set of lines
     * @details This function finds intersections between a vector of cv::Vec2f lines.
     * Intersections that are below or above the image margins are discarded.
     * 
     * @param lines The std::vector of cv::Vec2f lines
     * @param cols  The number of columns of the image
     * @param rows  The number of rows of the image
     * @return      A std::vector of cv::Point2f which represent the intersection between the lines
     */
    std::vector<cv::Point2f> findCorners(std::vector<cv::Vec2f> lines, const int &cols, const int &rows);
    
public:

    /**
     * Constructor
     */
    BoardCalibrator(std::string _name);

    /**
     * Destructor
     */
    ~BoardCalibrator();
};
