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

#include <snap_circuits/snap_circuits_board.h> 

#include <vector>

namespace snapCircuits {

class snapCircuitsBoard
{
private:
    int n_rows;     // number of rows in the board
    int n_cols;     // number of cols in the board

    std::vector<snapCircuitsPart> parts;  // vector of parts (i.e. the complete board state)
    std::vector<int>                IDs;  // vector of IDs (to recover board state history)

    unsigned int cur_id; // the ID to assign to the next part that is going to be added
                         // (used in case the user removes some parts)

    NSVGimage*  svg_image; // the global svg image with the board and the parts

public:
    /* CONSTRUCTORS */
    snapCircuitsBoard();
    snapCircuitsBoard(int _n_rows, int _n_cols);

    /**
    * Standard Copy Operator
    **/
    snapCircuitsBoard &operator=(const snapCircuitsBoard &_b);

    /**
     * Copy operator overloaded with the corresponding ros message.
     * It populates the snapCircuitsBoard from a ros msg.
     * @param  _b the ros message
     */
    snapCircuitsBoard &operator=(const snap_circuits::snap_circuits_board &_b);

    /**
     * isEqual Operator. It compares the two boards against their properties 
     * (i.e. size, parts), AND the order with which the parts have been placed
     * on the board (i.e. the IDs and cur_id).
     *
     * @param  _b  The board to compare to
     * @return     true/false if success/failure
     */
    bool operator==(const snapCircuitsBoard &_b);
    
    /**
     * Converts the snapCircuitsBoard to the ros msg
     * @return the ros message relative to the snapCircuitsBoard
     */
    snap_circuits::snap_circuits_board toMsg();

    /**
     * Adds a snapCircuitsPart to the board. Its location will be automatically
     * calibrated to the size of the board, whereas its ID will be incrementally assigned
     * @param  _p the part to add
     * @return    true/false if success/failure
     */
    bool addPart(const snapCircuitsPart & _p);

    /**
     * Removes a snapCircuitsPart given its ID. The other parts will be not affected
     * @param  _ID the unique ID of the part to be removed
     * @return     true/false if success/failure
     */
    bool removePart(const int &_ID);

    /**
     * Creates an svg based on the existing parts and the board picture.
     * @return  true/false if success/failure
     */
    bool createSVGimage();

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

    /* GETTERS */
    NSVGimage* getImage()                     { return svg_image; };
    std::vector<int> getIDs()                 { return IDs; };
    int getID(int i)                          { return IDs[i]; };
    std::vector<snapCircuitsPart> getParts()  { return parts; };
    snapCircuitsPart* getPart(int i)          { return &(parts[i]); };
    int getNParts()                           { return parts.size()-1; }; // The first item is the board.

    /* SETTERS */
    bool set_n_rows(const int &_r)            { n_rows=_r; return true; };
    bool set_n_cols(const int &_c)            { n_cols=_c; return true; };
    bool set_n_rows_and_cols(const int &_r, const int &_c);

    bool setID(const int &_i, const int &_ID) { IDs[_i] = _ID; return true; };
    bool setIDs(const std::vector<int> _IDs)  { IDs = _IDs; return true; }; 

    /* DESTRUCTOR */
    ~snapCircuitsBoard();
};

};

#endif

// empty line to make gcc happy
