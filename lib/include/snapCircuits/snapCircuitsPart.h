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
    int ID;             // Order of appearance on the board

    std::string  name;  // Name as stored in the snapCircuits documentation
    std::string label;  // Label as written on the physical part.It is also the
                        // filename of the svg image corresponding to the part

    snapCircuits::snapLocation  location;  // Location

    NSVGimage* image;      // svg image 

public:
    /* CONSTRUCTORS */
    snapCircuitsPart(std::string _label);
    snapCircuitsPart(std::string _label, snapCircuits::snapLocation _location);

    /**
     * Initialization method (used in the constructors because overload of the 
     * constructor is allowed only from C++11 onward). It sets name and image
     * from the label
     */
    void init();

    /* SETTERS */
    bool setImage(NSVGimage* _image);

    /* GETTERS */
    NSVGimage* getImage() { return image; };

    ~snapCircuitsPart();
    
};

};

#endif

// empty line to make gcc happy
