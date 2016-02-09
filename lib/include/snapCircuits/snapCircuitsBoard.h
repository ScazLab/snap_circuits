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

#ifndef __SNAPCIRCUITS_BOARD_H__
#define __SNAPCIRCUITS_BOARD_H__

#include "snapCircuits/utils.h"
#include "snapCircuits/snapCircuitsPart.h"

#include <vector>

namespace snapCircuits {

class snapCircuitsBoard
{
private:
    int MAX_ROWS;
    int MAX_COLS;

    std::vector<snapCircuitsPart> parts;

public:
    /* CONSTRUCTORS */
    snapCircuitsBoard();
    snapCircuitsBoard(int _x_max, int _y_max);

    /* DESTRUCTOR */
    ~snapCircuitsBoard();
    
};

};

#endif

// empty line to make gcc happy
