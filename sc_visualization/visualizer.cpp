#define NANOSVG_IMPLEMENTATION      // Expands implementation of nanosvg.h
#define NANOSVGRAST_IMPLEMENTATION  // Expands implementation of nanosvgrast.h

#include "snap_circuits/snap_circuits_board.h"
#include "snapCircuits/snapCircuitsBoard.h"

#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace snapCircuits;

class Visualizer
{
private:
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber;

    image_transport::ImageTransport imageTransport;
    image_transport::Publisher      imagePublisher;

    snapCircuitsBoard board;

    void boardCallback(const snap_circuits::snap_circuits_board::ConstPtr& msg)
    {
        board=*msg;
        board.print();
    };

public:
    Visualizer(string sub, string pub) : imageTransport(nodeHandle)
    {
        subscriber     = nodeHandle.subscribe(sub.c_str(),1,&Visualizer::boardCallback, this);
        imagePublisher = imageTransport.advertise(pub,1);
    };

    ~Visualizer() {};
};

int main(int argc, char** argv)
{
    std::string sub = "/snap_circuits/board_state";
    std::string pub = "/snap_circuits/board_visualization";

    if (argc>1)
    {
        sub=std::string(argv[1]);
    
        if (argc>2)
        {
            pub=std::string(argv[2]);
        }
    }

    ros::init(argc, argv, "visualizer");
    Visualizer visualizer(sub,pub);
    ros::spin();

    return 0;
}
