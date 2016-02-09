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

#ifndef __UTILS_H__
#define __UTILS_H__

#include <string>
#include <stdio.h>
#include <stdlib.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "nanosvg/nanosvg.h"
#include "nanosvg/nanosvgrast.h"

#define N_ROWS 10    // Number of rows on the board
#define N_COLS  8

// sdt::map 

namespace snapCircuits {

class snapLocation {

private:
    int x;  // position on the x axis
    int y;  // position on the y axis
    int o;  // orientation (either 0, 90, 180, 270)

    int x_max; // max value for the x axis position (i.e. max number of cols)
    int y_max; // max value for the y axis position (i.e. max number of rows)

public:
    /* CONSTRUCTORS */
    snapLocation();
    snapLocation(int _x_max, int _y_max);
    snapLocation(int _x, int _y, int _o);
    snapLocation(int _x, int _y, int _o, int _x_max, int _y_max);

    /**
    * Copy Operator
    **/
    snapLocation &operator=(const snapLocation &_l);

    /* SETTERS */
    bool setX(const int &_x)        {         x=_x; return true; }; 
    bool setY(const int &_y)        {         y=_y; return true; }; 
    bool setO(const int &_o)        {         o=_o; return true; }; 
    bool setXMax(const int &_x_max) { x_max=_x_max; return true; }; 
    bool setYMax(const int &_y_max) { y_max=_y_max; return true; }; 

    bool setXYO(const int &_x, const int &_y, const int &_o);
    bool setXYMax(const int &_x_max, const int &_y_max);

    /* GETTERS */
    int getX()    {     return x; }; 
    int getY()    {     return y; }; 
    int getO()    {     return o; }; 
    int getXMax() { return x_max; }; 
    int getYMax() { return y_max; };

};

/*************************************************************************************/

/**
 * Converts a NSVGimage to cv::Mat
 * @param  _im    the input image
 * @param  _w     the desired width of the converted image
 * @param  _h     the desired height of the converted image
 * @param  result the resulting cv::Mat
 * @return        true/false if success/failure
 */
bool NSVGtocvMat(NSVGimage* _im, size_t _w, size_t _h, cv::Mat &result);

/**
 * Checks if a directory exists in the system given its absolute path
 * @param  path directory to check against
 * @return      true/false if success/failure
 */
bool dirExists(const char *path);

/**
 * Checks if a file exists in the system given its absolute path
 * @param  path file to check against
 * @return      true/false if success/failure
 */
bool fileExists(const char *path);

};

#endif

// empty line to make gcc happy
