#include "snapCircuits/utils.h"

#define NANOSVG_IMPLEMENTATION      // Expands implementation
#define NANOSVGRAST_IMPLEMENTATION  // Expands implementation

#include "nanosvg/nanosvg.h"
#include "nanosvg/nanosvgrast.h"

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int main(int argc, char const *argv[])
{
    // Load SVG
    NSVGimage* image;
    image = nsvgParseFromFile("/home/alecive/Desktop/Test!/FlatWoken/power_B.svg", "px", 96);

    size_t w=512;
    size_t h=512;

    Mat mat;
    snapCircuits::NSVGtocvMat(image,w,h,mat);
    printf("Created cv Mat. Mat size %i %i\n",mat.rows,mat.cols);

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

    nsvgDelete(image);

    imwrite("/tmp/out.png", mat);

    return 0;
}
