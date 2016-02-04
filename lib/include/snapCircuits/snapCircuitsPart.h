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

#ifndef __SNAPCIRCUITS_PART_H__
#define __SNAPCIRCUITS_PART_H__

#include <string>
#include "snapCircuits/utils.h"

namespace snapCircuits {

class snapCircuitsPart
{
private:
    std::string                label;
    std::string                 name;

    snapCircuits::snapLocation  location;

    NSVGimage*                 image;

public:
    snapCircuitsPart();
    snapCircuitsPart(std::string _label);
    ~snapCircuitsPart();
    
};

};

#endif

// empty line to make gcc happy
