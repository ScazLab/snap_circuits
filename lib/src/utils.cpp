#include "snapCircuits/utils.h"

#include <sys/types.h>
#include <sys/stat.h>

#include <ros/ros.h>

using namespace std;
using namespace cv;
using namespace snapCircuits;

/*************************************************************************************/
/*                                SNAP LOCATION                                      */
/*************************************************************************************/

snapLocation::snapLocation()
{
    setXYO(-1,-1,-1);
    setXYMax(X_MAX,Y_MAX);
};

snapLocation::snapLocation(int _x_max, int _y_max)
{
    setXYO(-1,-1,-1);
    setXYMax(_x_max,_y_max);
};

snapLocation::snapLocation(int _x, int _y, int _o)
{
    setXYO(_x,_y,_o);
    setXYMax(X_MAX,Y_MAX);
};

snapLocation::snapLocation(int _x, int _y, int _o, int _x_max, int _y_max)
{
    setXYO(_x,_y,_o);
    setXYMax(_x_max,_y_max);
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

bool snapLocation::setXYO(const int &_x, const int &_y, const int &_o)
{
    return setX(_x) && setY(_y) && setO(_o);
};

bool snapLocation::setXYMax(const int &_x_max, const int &_y_max)
{
    return setXMax(_x_max) && setYMax(_y_max);
}

/*************************************************************************************/

bool snapCircuits::NSVGtocvMat(NSVGimage* _im, size_t _w, size_t _h, cv::Mat &result)
{
    // Create rasterizer (can be used to render multiple images).
    NSVGrasterizer* rast = nsvgCreateRasterizer();
    // Allocate memory for image
    unsigned char* img = (unsigned char*) malloc(_w*_h*4);
    // Rasterize
    nsvgRasterize(rast, _im, 0,0,1, img, _w, _h, _w*4);

    if (img == NULL)
    {
        ROS_ERROR("[snapCircuits::NSVGtocvMat] destination image data is empty!\n");
        return false;
    }

    Mat res = Mat(_h, _w, CV_8UC4);
    res.data = img;
    cv::cvtColor(res, res, CV_RGBA2BGRA);
    result = res.clone();   // I am cloning the image so that I can later free the malloc

    nsvgDeleteRasterizer(rast);
    free(img);

    return true;
}

bool snapCircuits::dirExists(const char *path)
{
    struct stat info;

    if( stat( path, &info ) != 0 )
    {
        ROS_ERROR_NAMED("test_only", "Cannot access %s", path);
        return false;
    }
    else if( info.st_mode & S_IFDIR )
    {
        ROS_DEBUG_NAMED("test_only", "%s is a directory", path);
        return true;
    }
    else
    {
        ROS_ERROR_NAMED("test_only", "%s is not a directory", path );
        return false;
    }

    return false;
}

bool snapCircuits::fileExists(const char *path)
{
    struct stat info;

    if( stat( path, &info ) != 0 )
    {
        ROS_ERROR_NAMED("test_only", "Cannot access %s", path);
        return false;
    }
    else if( info.st_mode & S_IFREG )
    {
        ROS_DEBUG_NAMED("test_only", "%s is a file", path);
        return true;
    }
    else
    {
        ROS_ERROR_NAMED("test_only", "%s is not a directory", path );
        return false;
    }

    return false;
}
