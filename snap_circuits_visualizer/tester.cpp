#include "snapCircuits/utils.h"
#include "snapCircuits/snapCircuitsPart.h"
#include "snapCircuits/snapCircuitsBoard.h"

#define NANOSVG_IMPLEMENTATION      // Expands implementation
#define NANOSVGRAST_IMPLEMENTATION  // Expands implementation

#include "nanosvg/nanosvgrast.h"
#include "nanosvg/nanosvgutils.h"

#include <vector>

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

    snapCircuitsBoard board;
    board.addPart(spPart);
    board.addPart(snapCircuitsPart("S1",snapLocation(1,1,90)));
    board.removePart(1);
    board.addPart(snapCircuitsPart("S1",snapLocation(2,4,90)));
    board.addPart(snapCircuitsPart("S1",snapLocation(1,5,270)));
    board.addPart(snapCircuitsPart("S1",snapLocation(3,3,180)));
    board.print(1);
    board.removePart(10);

    spPart.loadSVGimage();

    if (spPart.getImage()!=NULL)
    {
        for (NSVGshape* shape = spPart.getImage()->shapes; shape != NULL; shape = shape->next) {
            printNSVGshape(*shape);
            for (NSVGpath*   path = shape->paths; path != NULL; path = path->next) {
                printNSVGpath(*path);
            }
        }

        size_t w=512;
        size_t h=512;

        Mat mat;
        snapCircuits::NSVGtocvMat(spPart.getImage(),w,h,mat);
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

    return 0;
}
