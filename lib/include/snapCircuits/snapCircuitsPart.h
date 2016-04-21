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

#include <snap_circuits/snap_circuits_part.h> 

namespace snapCircuits {

class snapCircuitsPart
{
private:
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
    snapCircuitsPart(snap_circuits::snap_circuits_part &_sp);
    snapCircuitsPart(snap_circuits::snap_circuits_part &_sp, int _x_max, int _y_max);

    /**
     * Initialization method (used in the constructors because overload of the 
     * constructor is allowed only from C++11 onward). It sets name and image
     * from the label
     */
    void init();

    /**
    * Standard Copy Operator
    **/
    snapCircuitsPart &operator=(const snapCircuitsPart &_p);

    /**
     * Copy operator overloaded with the corresponding ros message.
     * It populates the snapCircuitsPart from a ros msg.
     * @param  _sp the ros message
     */
    snapCircuitsPart &operator=(const snap_circuits::snap_circuits_part &_sp);

    /**
     * isEqual Operator
     * This function compares only labels and their positions.
     */
    bool operator==(const snapCircuitsPart &_p);

    /**
     * Converts the snapCircuitsPart to the ros msg
     * @return the ros message relative to the snapCircuitsPart
     */
    snap_circuits::snap_circuits_part toMsg();

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
     * Originally, this functionality was inside getImageFilefromLabel, but due to 
     * limitations of the nsvg library it was not possible to keep it there.
     * Further, it is useless to load the file in memory if it's not used for visualization
     * and/or saving it to file.
     * @return true/false if success/failure
     */
    bool loadSVGimage();

    /**
     * Puts a state of the part into a string
     * @param  verbosity the bigger the verbosity, the more text will be printed
     * @return           the state of the part as a std::string
     */
    std::string toString(int verbosity=0);

    /* SETTERS */
    bool setImage(NSVGimage* _image);

    bool setXYMax(const int &_x_max, const int &_y_max);
    bool setLocation(const int &_x, const int &_y, const int &_o);

    /* GETTERS */
    NSVGimage* getImage()       { return svg_image; };
    std::string getName()       { return name;      };
    std::string getLabel()      { return label;     };
    std::string getSVGfile()    { return svg_file;  };
    snapCircuits::snapLocation getLocation() { return location; };

    /* DESTRUCTOR */
    ~snapCircuitsPart();
};

};

#endif

// empty line to make gcc happy
