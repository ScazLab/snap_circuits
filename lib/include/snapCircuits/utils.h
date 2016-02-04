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

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "nanosvg/nanosvg.h"
#include "nanosvg/nanosvgrast.h"

namespace snapCircuits {

struct Location {
    int x;  // position on the x axis
    int y;  // position on the y axis
    int o;  // orientation (either 90, 180, 270, 360)
};


/**
 * This method retrieves the name and label description of the part from its ID.
 * Originally, it was elegantly implemented as a std::map<std::string, std::tuple>, but
 * that would have required C++11 and I decided to drop it.
 * @param _id   is the id of the part
 * @param name  is the name  of the part that is going to be retrieved from the list
 * @param label is the label of the part that is going to be retrieved from the list
 * @return true/false if success/failure
 */
bool getNameLabelfromID(const std::string &_id, std::string &name, std::string &label);

/**
 * Converts a NSVGimage to cv::Mat
 * @param  _im    the input image
 * @param  _w     the desired width of the converted image
 * @param  _h     the desired height of the converted image
 * @param  result the resulting cv::Mat
 * @return        true/false if success/failure
 */
bool NSVGtocvMat(NSVGimage* _im, size_t _w, size_t _h, cv::Mat &result);

};

#endif

// empty line to make gcc happy
