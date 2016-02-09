#include "snapCircuits/snapCircuitsPart.h"
#include <ros/console.h>

using namespace std;
using namespace snapCircuits;

snapCircuitsPart::snapCircuitsPart(string _label) : label(_label), ID(-1)
{
    init();
}

snapCircuitsPart::snapCircuitsPart(std::string _label, snapCircuits::snapLocation _location) :
                                   ID(-1), location(_location)
{
    init();
}

void snapCircuitsPart::init()
{
    ROS_INFO("[snapCircuitsPart::init]");

    image=NULL;
    std::string svg_file="";

    snapCircuits::getNamefromLabel(label,name);

    // Originally, nsvgParseFromFile was inside checkImageFilefromLabel (which was called 
    // getImagefromLabel), but due to limitations of the nsvg library it was not possible to
    // keep it there. I decided to move it in the constructor of the snapCircuitsPart
    // (it does not harm so much)
    if (snapCircuits::checkImageFilefromLabel(label,&svg_file))
    {
        image=nsvgParseFromFile(svg_file.c_str(), "px", 96);
    }
}

bool snapCircuitsPart::setImage(NSVGimage* _image)
{
    image = _image;

    return true;
}

snapCircuitsPart::~snapCircuitsPart()
{
    if (image!=NULL)
    {
        nsvgDelete(image);
    }
}
