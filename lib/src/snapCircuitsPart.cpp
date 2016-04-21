#include "snapCircuits/snapCircuitsPart.h"

#include <sstream>

#include <ros/ros.h>

using namespace std;
using namespace snapCircuits;

snapCircuitsPart::snapCircuitsPart(string _label) : label(_label)
{
    init();
}

snapCircuitsPart::snapCircuitsPart(std::string _label, snapLocation _location) :
                                   label(_label), location(_location)
{
    init();
}

snapCircuitsPart::snapCircuitsPart(snap_circuits::snap_circuits_part &_sp) :
                                   label(_sp.label), location(_sp.loc)
{
    init();
}

snapCircuitsPart::snapCircuitsPart(snap_circuits::snap_circuits_part &_sp,
                                   int _x_max, int _y_max) :
                                   label(_sp.label), location(_sp.loc)
{
    init();
    setXYMax(_x_max,_y_max);
}

void snapCircuitsPart::init()
{
    svg_image=NULL;

    getNamefromLabel();
    getImageFilefromLabel();  
}

snapCircuitsPart & snapCircuitsPart::operator=(const snapCircuitsPart &_p)
{
    // We don't want to pass the pointer to the NSVGimage, we can create it on the fly
    name     = _p.name;
    label    = _p.label;
    svg_file = _p.svg_file;
    location = _p.location;

    return *this;
}

snapCircuitsPart & snapCircuitsPart::operator=(const snap_circuits::snap_circuits_part &_sp)
{
    label    = _sp.label;
    getNamefromLabel();
    getImageFilefromLabel();
    location = _sp.loc;

    return *this;
}

bool snapCircuitsPart::operator==(const snapCircuitsPart &_p)
{
    return label==_p.label && location==_p.location;
}

snap_circuits::snap_circuits_part snapCircuitsPart::toMsg()
{
    snap_circuits::snap_circuits_part prt_msg;
    prt_msg.label = label;
    prt_msg.loc   = location.toMsg();

    return prt_msg;
}

bool snapCircuitsPart::getNamefromLabel()
{
    if      (label=="BG")    { name="Base Grid"; }
    else if (label=="1")     { name="1-Snap Wire"; }
    else if (label=="2")     { name="2-Snap Wire"; }
    else if (label=="3")     { name="3-Snap Wire"; }
    else if (label=="4")     { name="4-Snap Wire"; }
    else if (label=="5")     { name="5-Snap Wire"; }
    else if (label=="6")     { name="6-Snap Wire"; }
    else if (label=="WC")    { name="Whistle Chip"; }
    else if (label=="S1")    { name="Slide Switch"; }
    else if (label=="S2")    { name="Press Switch"; }
    else if (label=="RP")    { name="Photoresistor"; }
    else if (label=="D1")    { name="Red LED"; }
    else if (label=="L1")    { name="2.5V Lamp"; }
    else if (label=="B1")    { name="Battery"; }
    else if (label=="SP")    { name="Speaker"; }
    else if (label=="U1")    { name="Music IC"; }
    else if (label=="U2")    { name="Alarm IC"; }
    else if (label=="U3")    { name="Space War IC"; }
    else if (label=="M1")    { name="Motor Fan"; }
    else if (label=="R1")    { name="100 Ohm Resistor"; }
    else {
        ROS_ERROR("[snapCircuitsPart::getNamefromLabel] Wrong label name: %s",label.c_str());
        return false;
    }

    return true;
}

bool snapCircuitsPart::getImageFilefromLabel()
{
    string svg_folder="";

    if (ros::param::get("snap_circuits/svg_folder", svg_folder))
    {
        ROS_DEBUG("[snapCircuitsPart::getImageFilefromLabel] svg folder: %s",svg_folder.c_str());

        if (dirExists(svg_folder.c_str()))
        {
            string fullpath=svg_folder+label+".svg";

            if (fileExists(fullpath.c_str()))
            {
                svg_file=fullpath;
                return true;
            }
        }
    }

    svg_file="";
    return false;
}

bool snapCircuitsPart::loadSVGimage()
{
    if (svg_file=="")
    {
        ROS_WARN("[snapCircuitsPart::loadSVGimage] No svg file associated to the part. Retrieving image file..");

        if(!getImageFilefromLabel())
        {
            ROS_ERROR("[snapCircuitsPart::loadSVGimage] No svg file associated to the part has been found.");
            svg_image=NULL;
            return false;
        }
    }

    svg_image=nsvgParseFromFile(svg_file.c_str(), "px", 96);
    return true;
}

std::string snapCircuitsPart::toString(int verbosity)
{
    std::stringstream res;

    res << "label: "  << label;
    if (verbosity>0)
    {
        res << "\t name: " << name;
    }
    res << "\t location: " << location.toString(verbosity);
    if (verbosity>1)
    {
        res << "\n svg_file: " << svg_file;
    }

    return res.str();
}

bool snapCircuitsPart::setImage(NSVGimage* _image)
{
    svg_image = _image;

    return true;
}

bool snapCircuitsPart::setXYMax(const int &_x_max, const int &_y_max)
{
    return location.setXYMax(_x_max,_y_max);
}

bool snapCircuitsPart::setLocation(const int &_x, const int &_y, const int &_o)
{
    return location.setXYO(_x, _y, _o);
}

snapCircuitsPart::~snapCircuitsPart()
{
    if (svg_image!=NULL)
    {
        nsvgDelete(svg_image);
    }
}
