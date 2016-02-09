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

bool snapCircuits::getNamefromLabel(const std::string &_label, std::string &name)
{
    if      (_label=="1")     { name="1-Snap Wire"; }
    else if (_label=="2")     { name="2-Snap Wire"; }
    else if (_label=="3")     { name="3-Snap Wire"; }
    else if (_label=="4")     { name="4-Snap Wire"; }
    else if (_label=="5")     { name="5-Snap Wire"; }
    else if (_label=="6")     { name="6-Snap Wire"; }
    else if (_label=="WC")    { name="Whistle Chip"; }
    else if (_label=="S1")    { name="Slide Switch"; }
    else if (_label=="S2")    { name="Press Switch"; }
    else if (_label=="RP")    { name="Photoresistor"; }
    else if (_label=="D1")    { name="Red LED"; }
    else if (_label=="L1")    { name="2.5V Lamp"; }
    else if (_label=="B1")    { name="Battery - 2x1.5V AA"; }
    else if (_label=="SP")    { name="Speaker"; }
    else if (_label=="U1")    { name="Music Integrated Circuit"; }
    else if (_label=="U2")    { name="Alarm Integrated Circuit"; }
    else if (_label=="U3")    { name="Space War Integrated Circuit"; }
    else if (_label=="M1")    { name="Motor Fan"; }
    else if (_label=="R1")    { name="100 Ohm Resistor"; }
    else {
        ROS_ERROR("[snapCircuits::getNamefromLabel] Wrong label name: %s",_label.c_str());
        return false;
    }

    return true;
}

bool snapCircuits::checkImageFilefromLabel(const std::string &_label, std::string* path)
{
    string svg_folder="";

    if (ros::param::get("snap_circuits/svg_folder", svg_folder))
    {
        ROS_INFO("[snapCircuits::getImagefromLabel] svg folder: %s",svg_folder.c_str());

        if (dirExists(svg_folder.c_str()))
        {
            string fullpath=svg_folder+_label+".svg";

            if (fileExists(fullpath.c_str()))
            {
                *path=fullpath;
                return true;
            }
        }
    }

    *path="";
    return false;
}

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
