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
#include <algorithm>
#include <vector>
#include <stdio.h>
#include <sstream>

#include <opencv2/core/core.hpp>

#define FILT_WINDOW   7

class MedianFilter
{
private: 
    int size;
    std::vector<int> data;
public:
    /**
     * Constructors
     */
    MedianFilter() : size(FILT_WINDOW) {};
    MedianFilter(int _size) : size(_size) {};

    /**
     * Copy operator
     */
    MedianFilter &operator=(const MedianFilter &_mf);

    /**
     * Pushes a point into the filter
     * 
     * @param _point The point to be pushed in
     * @return       true/false if success/failure
     */
    bool pushData(const int &_point);

    /**
     * Gets the median value of the buffer
     * @return  the median
     */
    int getMedian();

    /**
     * Gets a specific value of the buffer
     * @param  _i the index of the element to request
     * @return    the value requested
     */
    int getPoint(const int &_i);

    /**
     * Gets the size of the buffer
     * @return  the size of the buffer
     */
    int getDataSize() { return data.size(); };

    /**
     * Pushes the filter data into a string
     */
    std::string toString();

    /**
     * Destructor
     */
    ~MedianFilter() {};
};

class cvPointFilter
{
    // There will be a MedianFilter for each of the two
    // dimensions of the cv::Point2f
    MedianFilter x;
    MedianFilter y;

public:
    /**
     * Constructors
     */
    cvPointFilter() {};
    cvPointFilter(int _size) : x(_size) , y(_size) {};

    /**
     * Copy operator
     */
    cvPointFilter &operator=(const cvPointFilter &pf);

    /**
     * Pushes a point into the filter
     * 
     * @param _point The cv::Point2f to be pushed in
     * @return       true/false if success/failure
     */
    bool pushPoint2f(const cv::Point2f &_point);

    /**
     * Gets the median value of the buffer
     * @return  the median
     */
    cv::Point2f getMedian();

    /**
     * Gets a specific value of the buffer
     * @param  _i the index of the element to request
     * @return    the value requested
     */
    cv::Point2f getPoint(const int &_i);

    /**
     * Pushes the filter data into a string
     */
    std::string toString();

    /**
     * Destructor
     */
    ~cvPointFilter() {};
};
