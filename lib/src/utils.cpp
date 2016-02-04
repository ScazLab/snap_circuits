#include "snapCircuits/utils.h"

using namespace std;
using namespace  cv;

bool snapCircuits::getNameLabelfromID(const string &_id, string &name, string &label)
{
    return true;
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
        printf("ERROR: destination image data is empty!\n");
        return false;
    }

    Mat res = Mat(_h, _w, CV_8UC4);
    res.data = img;
    cv::cvtColor(res, res, CV_RGBA2BGRA);
    result = res.clone();

    nsvgDeleteRasterizer(rast);
    free(img);

    return true;
}
