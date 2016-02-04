#include "snapCircuits/utils.h"

#define NANOSVG_IMPLEMENTATION      // Expands implementation
#define NANOSVGRAST_IMPLEMENTATION  // Expands implementation

#include "nanosvg/nanosvg.h"
#include "nanosvg/nanosvgrast.h"

#include <stdio.h>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> 

using namespace  cv;
using namespace std;

int main(int argc, char const *argv[])
{
    // Load SVG
    NSVGimage* image;
    image = nsvgParseFromFile("/home/alecive/Desktop/Test!/FlatWoken/power_B.svg", "px", 96);

    size_t w=512;
    size_t h=512;

    // Create rasterizer (can be used to render multiple images).
    struct NSVGrasterizer* rast = nsvgCreateRasterizer();
    // Allocate memory for image
    unsigned char* img = (unsigned char*) malloc(w*h*4);
    // Rasterize
    nsvgRasterize(rast, image, 0,0,1, img, w, h, w*4);

    if (img == NULL)
    {
        printf("ERROR: destination image data is empty!\n");
    }

    // std::vector<char> data(img, img + w*h*4);
    // Mat mat = imdecode(Mat(data), 1);
    Mat mat = Mat(h, w, CV_8UC4);
    mat = cv::Scalar(255,255,255);
    mat.data = img;
    cv::cvtColor(mat, mat, CV_RGBA2BGRA);

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

    nsvgDeleteRasterizer(rast);
    nsvgDelete(image);

    imwrite("/tmp/out.png", mat);

    return 0;
}
