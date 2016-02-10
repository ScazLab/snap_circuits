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
    int n_rows;     // number of rows in the board
    int n_cols;     // number of cols in the board

    std::vector<snapCircuitsPart> parts;  // vector of parts (i.e. the complete board state)

    unsigned int current_id; // the ID to assign to the next part that is going to be added

public:
    /* CONSTRUCTORS */
    snapCircuitsBoard();
    snapCircuitsBoard(int _n_rows, int _n_cols);

    /**
     * Adds a snapCircuitsPart to the board. Its location will be automatically
     * calibrated to the size of the board, whereas its ID will be incrementally assigned
     * @param  _p the part to add
     * @return    true/false if success/failure
     */
    bool addPart(snapCircuitsPart _p);

    /**
     * Removes a snapCircuitsPart given its ID. The other parts will be not affected
     * @param  _ID the unique ID of the part to be removed
     * @return     true/false if success/failure
     */
    bool removePart(const int &_ID);

    /**
     * Prints the state of the board
     * @param verbosity the bigger the verbosity, the more text will be printed
     */
    void print(int verbosity=0);

    /**
     * Resets the state of the board to its initial state.
     * n_rows and n_cols will be preserved.
     * @return true/false if success/failure
     */
    bool reset();

    /* SETTERS */
    bool set_n_rows(const int &_r) { n_rows=_r; return true; };
    bool set_n_cols(const int &_c) { n_cols=_c; return true; };
    bool set_n_rows_and_cols(const int &_r, const int &_c);

    /* DESTRUCTOR */
    ~snapCircuitsBoard() {};
    
};

};

#endif

// empty line to make gcc happy
