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

bool snapCircuitsBoard::addPart(snapCircuitsPart _p)
{
    _p.setXYMax(n_rows,n_cols);   // set the board dimension
    _p.setID(current_id);         // set the ID to the part
    current_id++;                 // increment that ID

    parts.push_back(_p);                        // add the part to the board

    return true;
}

bool snapCircuitsBoard::removePart(const int &_ID)
{
    std::vector<snapCircuitsPart>::iterator it;

    for (it = parts.begin(); it != parts.end(); ++it)
    {
        if (it->getID()==_ID)
        {
            parts.erase(it);
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
    ROS_INFO("BOARD: n_rows: %i \t n_cols %i \t current_id %i",n_rows,n_cols,current_id);
    for (int i = 0; i < parts.size(); ++i)
    {
        ROS_INFO("Part #%i:\t%s",i,parts[i].toString(verbosity).c_str());
    }
    ROS_INFO("*****************************");
}

bool snapCircuitsBoard::reset()
{
    current_id = 0;
    parts.clear();
    addPart(snapCircuitsPart("BG")); // The base grid is always the first element

    svg_image = (NSVGimage*)malloc(sizeof(NSVGimage));
    if (svg_image == NULL) 
    {
        free(svg_image);
        return false;
    }
    memset(svg_image, 0, sizeof(NSVGimage));

    return true;
}

bool snapCircuitsBoard::set_n_rows_and_cols(const int &_r, const int &_c)
{
    return set_n_rows(_r) && set_n_cols(_c);
}

snapCircuitsBoard::~snapCircuitsBoard()
{
    if (svg_image!=NULL)
    {
        nsvgDelete(svg_image);
    }
}
