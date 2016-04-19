#include "snapCircuits/utils.h"

#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>

#include <ros/ros.h>

using namespace std;
using namespace cv;
using namespace snapCircuits;

std::string snapCircuits::int_to_string( int a )
{
    std::stringstream ss;
    ss << a;
    return ss.str();
};

/*************************************************************************************/
/*                                SNAP LOCATION                                      */
/*************************************************************************************/

snapLocation::snapLocation()
{
    setXYMax(N_ROWS,N_COLS);
    resetLocation();
};

snapLocation::snapLocation(int _x_max, int _y_max)
{
    setXYMax(_x_max,_y_max);
    resetLocation();
};

snapLocation::snapLocation(int _x, int _y, int _o)
{
    setXYMax(N_ROWS,N_COLS);
    if (!setXYO(_x,_y,_o))
    {
        resetLocation();
    }
};

snapLocation::snapLocation(int _x, int _y, int _o, int _x_max, int _y_max)
{
    setXYMax(_x_max,_y_max);
    if (!setXYO(_x,_y,_o))
    {
        resetLocation();
    }
};

snapLocation & snapLocation::operator=(const snapLocation &_l)
{
    x = _l.x;
    y = _l.y;
    o = _l.o;

    x_max = _l.x_max;
    y_max = _l.y_max;

    return *this;
}

bool snapLocation::operator==(const snapLocation &_l)
{
    return x==_l.x && y==_l.y && o==_l.o &&
           x_max==_l.x_max && y_max==_l.y_max;
}

snap_circuits::snap_location snapLocation::toMsg()
{
    snap_circuits::snap_location loc_msg;
    loc_msg.x = x;
    loc_msg.y = y;
    loc_msg.o = o;

    return loc_msg;
}

void snapLocation::resetLocation()
{
    x = -1;
    y = -1;
    o = -1;
}

std::string snapLocation::toString(int verbosity)
{
    std::stringstream res;

    res << "[x : " << x << " y: " << y << " o: " << o << "]";
    if (verbosity>0)
    {
        res << "\t[x_max: " << x_max << " y_max: " << y_max << "]";
    }

    return res.str();
}

bool snapLocation::setX(const int &_x)
{
    if (_x>=0 && _x<x_max)
    {
        x=_x;
        return true;
    }
    else
    {
        ROS_ERROR("[snapLocation::setX] out of bounds assignment: requested %i, allowed [%i %i]",_x,0,x_max-1);
        return false;
    }
}

bool snapLocation::setY(const int &_y)
{
    if (_y>=0 && _y<y_max)
    {
        y=_y;
        return true;
    }
    else
    {
        ROS_ERROR("[snapLocation::setY] out of bounds assignment: requested %i, allowed [%i %i]",_y,0,y_max-1);
        return false;
    }
}

bool snapLocation::setO(const int &_o)
{
    if (_o==0 || _o==90 || _o==180 || _o==270 || _o==360)
    {
        o=_o;
        if (o==360)
        {
            o=0;
        }
        return true;
    }
    else
    {
        ROS_ERROR("[snapLocation::setO] orientation %i is not permitted. Please use [0, 90, 180, 270].",_o);
        return false;
    }

}

bool snapLocation::setXMax(const int &_x_max)
{
    if (_x_max<1)
    {
        ROS_ERROR("[snapLocation::setYMax] cannot assign a x_max lower than 1 (it was %i)",_x_max);
        return false;
    }

    x_max=_x_max;
    return true;
}

bool snapLocation::setYMax(const int &_y_max)
{
    if (_y_max<1)
    {
        ROS_ERROR("[snapLocation::setXMax] cannot assign a y_max lower than 1 (it was %i)",_y_max);
        return false;
    }

    y_max=_y_max;
    return true;
} 

bool snapLocation::setXYO(const int &_x, const int &_y, const int &_o)
{
    return setX(_x) && setY(_y) && setO(_o);
};

bool snapLocation::setXYMax(const int &_x_max, const int &_y_max)
{
    return setXMax(_x_max) && setYMax(_y_max);
}

/*************************************************************************************/
/*                                    UTILS                                          */
/*************************************************************************************/

bool snapCircuits::NSVGtocvMat(NSVGimage* _im, size_t _w, size_t _h, cv::Mat &result)
{
    NSVGrasterizer* rast = nsvgCreateRasterizer();          // Create rasterizer (can be used to render multiple images).
    unsigned char* img = (unsigned char*) malloc(_w*_h*4);  // Allocate memory for image
    nsvgRasterize(rast, _im, 0,0,1, img, _w, _h, _w*4);     // Rasterize

    if (img == NULL)
    {
        ROS_ERROR("[snapCircuits::NSVGtocvMat] destination image data is empty!\n");
        return false;
    }

    Mat res = Mat(_h, _w, CV_8UC4);
    res.data = img;
    cv::cvtColor(res, res, CV_RGBA2BGRA);
    result = res.clone();   // I am cloning the image so that I can later free the malloc

    nsvgDeleteRasterizer(rast);     // Delete the rasterizer
    free(img);                      // Free the memory that has been allocated for the image

    return true;
}

bool snapCircuits::dirExists(const char *path)
{
    struct stat info;

    if( stat( path, &info ) != 0 )
    {
        ROS_ERROR_NAMED("test_only", "[snapCircuits::dirExists] Cannot access %s", path);
        return false;
    }
    else if( info.st_mode & S_IFDIR )
    {
        ROS_DEBUG_NAMED("test_only", "[snapCircuits::dirExists] %s is a directory", path);
        return true;
    }
    else
    {
        ROS_ERROR_NAMED("test_only", "[snapCircuits::dirExists] %s is not a directory", path );
        return false;
    }

    return false;
}

bool snapCircuits::fileExists(const char *path)
{
    struct stat info;

    if( stat( path, &info ) != 0 )
    {
        ROS_WARN_NAMED("test_only", "[snapCircuits::fileExists] Cannot access %s", path);
        return false;
    }
    else if( info.st_mode & S_IFREG )
    {
        ROS_DEBUG_NAMED("test_only", "[snapCircuits::fileExists] %s is a file", path);
        return true;
    }
    else
    {
        ROS_ERROR_NAMED("test_only", "[snapCircuits::fileExists] %s is not a file", path );
        return false;
    }

    return false;
}
