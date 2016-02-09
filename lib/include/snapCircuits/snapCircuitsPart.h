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
                        // Filename of the svg image corresponding to the part

    std::string svg_file;  // Full path of the svg image file associated with the part
    NSVGimage*  svg_image; // svg image 

    snapCircuits::snapLocation  location;  // Location on the board

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

    /**
     * Retrieves the name of the part from its label.
     * Originally, it was elegantly implemented as a std::map<std::string, std::tuple>, but
     * that would have required C++11 and I decided to drop it.
     * @return true/false if success/failure
     */
    bool getNamefromLabel();

    /**
     * Retrieves the svg image file associated to a specific label. It depends on a parameter stored
     * in the parameter server that specifies the absolute path of the folder where the images
     * are stored (usually, snap_circuits/lib/svg ). 
     * @return         true/false if success/failure
     */
    bool getImageFilefromLabel();

    /**
     * Loads the svg image in memory (provided the existence of a proper SVG file
     * associated to it). Used for visualization purposes.
     * @return true/false if success/failure
     */
    bool loadSVGimage();

    /* SETTERS */
    bool setImage(NSVGimage* _image);

    /* GETTERS */
    NSVGimage* getImage() { return svg_image; };

    /* DESTRUCTOR */
    ~snapCircuitsPart();
};

};

#endif

// empty line to make gcc happy
