#include "snapCircuits/snapCircuitsBoard.h"
#include "nanosvg/nanosvgutils.h"
#include <ros/console.h>

using namespace std;
using namespace snapCircuits;

snapCircuitsBoard::snapCircuitsBoard()
{
    set_n_rows_and_cols(N_ROWS,N_COLS);
    reset();
}

snapCircuitsBoard::snapCircuitsBoard(int _n_rows, int _n_cols)
{
    set_n_rows_and_cols(_n_rows,_n_cols);
    reset();
}

bool snapCircuitsBoard::reset()
{
    cur_id = 0;
    parts.clear();
    IDs.clear();
    addPart(snapCircuitsPart("BG")); // The base grid is always the first element
    parts[0].setXYMax(n_rows,n_cols);

    svg_image = (NSVGimage*)malloc(sizeof(NSVGimage));
    if (svg_image == NULL) 
    {
        free(svg_image);
        return false;
    }
    memset(svg_image, 0, sizeof(NSVGimage));

    return true;
}

snapCircuitsBoard & snapCircuitsBoard::operator=(const snapCircuitsBoard &_b)
{
    set_n_rows_and_cols(_b.n_rows,_b.n_cols);
    reset();

    cur_id = _b.cur_id;
    parts  = _b.parts;
    IDs    = _b.IDs;

    return *this;
}

snapCircuitsBoard & snapCircuitsBoard::operator=(const snap_circuits::snap_circuits_board &_b)
{
    set_n_rows_and_cols(_b.n_rows,_b.n_cols);
    reset();

    // The first element, the board, is not going to be added twice
    for (int i = 1; i < _b.parts.size(); ++i)
    {
        snap_circuits::snap_circuits_part sp = _b.parts[i];
        addPart(snapCircuitsPart(sp,n_rows,n_cols));
        IDs[i]=_b.IDs[i];
    }

    cur_id = _b.cur_id;

    return *this;
}

bool snapCircuitsBoard::operator==(const snapCircuitsBoard &_b)
{
    bool res = n_rows==_b.n_rows &&
               n_cols==_b.n_cols &&
               parts.size()==_b.parts.size();

    if (res==false) return false;

    for (int i = 0; i < parts.size(); ++i)
    {
        res = res && parts[i]==_b.parts[i];
        res = res &&   IDs[i]==_b.IDs[i];
    }
    
    return res;
}

snap_circuits::snap_circuits_board snapCircuitsBoard::toMsg()
{
    snap_circuits::snap_circuits_board board_msg;

    board_msg.n_rows = n_rows;
    board_msg.n_cols = n_cols;
    board_msg.cur_id = cur_id;

    for (int i = 0; i < parts.size(); ++i)
    {
        board_msg.parts.push_back(parts[i].toMsg());
        board_msg.IDs.push_back(IDs[i]);
    }

    return board_msg;
}

bool snapCircuitsBoard::addPart(const snapCircuitsPart &_p)
{
    snapCircuitsPart p=_p;
    p.setXYMax(n_rows,n_cols);   // set the board dimension

    IDs.push_back(cur_id);
    parts.push_back(p);          // add the part to the board

    cur_id++;                    // increment that ID

    return true;
}

bool snapCircuitsBoard::removePart(const int &_ID)
{
    for (int i = 0; i < IDs.size(); ++i)
    {
        if (IDs[i]==_ID)
        {
            parts.erase(parts.begin()+i);
            IDs.erase(IDs.begin()+i);
            return true;
        }
    }
    
    ROS_ERROR("[snapCircuitsBoard::removePart] did not find any part with ID %i", _ID);
    return false;
}

bool snapCircuitsBoard::createSVGimage()
{
    // if (svg_image==NULL)
    // {
    //     parts[0].loadSVGimage();
    //     svg_image = parts[0].getImage();
    // }

    // printf("[%i] %i %i \n",i,parts[i].getImage()->shapes != NULL,svg_image!=NULL);

    for (int i = 0; i < parts.size(); ++i)
    {
        parts[i].loadSVGimage();

        printf("[%i] %i %i \n",i,parts[i].getImage()->shapes != NULL,svg_image!=NULL);
        if (parts[i].getImage()->shapes != NULL)
        {
            nsvgAppendShape(svg_image,parts[i].getImage()->shapes);
        }
    }
}

void snapCircuitsBoard::print(int verbosity)
{
    ROS_INFO("*****************************");
    ROS_INFO("BOARD: n_rows: %i \t n_cols %i \t current_id %i",n_rows,n_cols,cur_id);
    for (int i = 0; i < parts.size(); ++i)
    {
        ROS_INFO("Part #%i:\t%i %s",i,IDs[i],parts[i].toString(verbosity).c_str());
    }
    ROS_INFO("*****************************");
}

bool snapCircuitsBoard::set_n_rows_and_cols(const int &_r, const int &_c)
{
    set_n_rows(_r);
    set_n_cols(_c);
    return true;
}

snapCircuitsBoard::~snapCircuitsBoard()
{
    if (svg_image!=NULL)
    {
        nsvgDelete(svg_image);
    }
}
