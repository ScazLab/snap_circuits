#include "snapCircuits/utils.h"
#include "snapCircuits/snapCircuitsPart.h"
#include "snapCircuits/snapCircuitsBoard.h"

#define NANOSVG_IMPLEMENTATION      // Expands implementation
#define NANOSVGRAST_IMPLEMENTATION  // Expands implementation

#include "nanosvg/nanosvgrast.h"
#include "nanosvg/nanosvgutils.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>

using namespace cv;
using namespace std;
using namespace snapCircuits;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "snap_circuits_visualizer");

    snapCircuitsPart spPart("WC",snapLocation(7,1,360,10,8));

    // if (spPart.loadSVGimage())
    // {
    //     for (NSVGshape* shape = spPart.getImage()->shapes; shape != NULL; shape = shape->next) {
    //         printNSVGshape(shape);
    //     }
    // }

    snapCircuitsBoard board;
    board.addPart(spPart);
    board.addPart(snapCircuitsPart("WC",snapLocation(1,1,90)));
    
    board.removePart(1);
    
    board.addPart(snapCircuitsPart("WC",snapLocation(2,4,90)));
    board.addPart(snapCircuitsPart("WC",snapLocation(1,5,270)));
    board.addPart(snapCircuitsPart("WC",snapLocation(3,3,180)));
    board.print(1);
    board.removePart(10);

    // board.createSVGimage();

    // if (board.getImage()!=NULL)
    if (false)
    {
        size_t w=1024;
        size_t h=600;

        Mat mat;
        snapCircuits::NSVGtocvMat(board.getImage(),w,h,mat);
        ROS_INFO("Created cv Mat. Mat size %i %i\n",mat.rows,mat.cols);

        // Display image
        while (1)
        {
            imshow("Original Image", mat);

            int c = cvWaitKey(30);
            if (c == ' ')
            {
                break;
            }
            if (c == 'q' || c == 'Q' || c == 27)
            {
                return 0;
            }
        }

        // nsvgDelete(image);

        // Write to file
        imwrite("/tmp/out.png", mat);
    }

    ROS_INFO("Closing..");
    return 0;
}
