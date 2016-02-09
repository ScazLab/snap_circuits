#include "snapCircuits/snapCircuitsPart.h"
#include <ros/console.h>
#include <ros/ros.h>

using namespace std;
using namespace snapCircuits;

snapCircuitsPart::snapCircuitsPart(string _label) : label(_label), ID(-1)
{
    init();
}

snapCircuitsPart::snapCircuitsPart(std::string _label, snapLocation _location) :
                                   ID(-1), location(_location)
{
    init();
}

void snapCircuitsPart::init()
{
    ROS_INFO("[snapCircuitsPart::init]");

    svg_image=NULL;

    getNamefromLabel();
    getImageFilefromLabel();

    // Originally, nsvgParseFromFile was inside checkImageFilefromLabel (which was called 
    // getImagefromLabel), but due to limitations of the nsvg library it was not possible to
    // keep it there. I decided to move it in the constructor of the snapCircuitsPart
    // (it does not harm so much)
    
}

bool snapCircuitsPart::getNamefromLabel()
{
    if      (label=="1")     { name="1-Snap Wire"; }
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
    else if (label=="B1")    { name="Battery - 2x1.5V AA"; }
    else if (label=="SP")    { name="Speaker"; }
    else if (label=="U1")    { name="Music Integrated Circuit"; }
    else if (label=="U2")    { name="Alarm Integrated Circuit"; }
    else if (label=="U3")    { name="Space War Integrated Circuit"; }
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
        ROS_INFO("[snapCircuitsPart::getImagefromLabel] svg folder: %s",svg_folder.c_str());

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

bool snapCircuitsPart::setImage(NSVGimage* _image)
{
    svg_image = _image;

    return true;
}

snapCircuitsPart::~snapCircuitsPart()
{
    if (svg_image!=NULL)
    {
        nsvgDelete(svg_image);
    }
}
