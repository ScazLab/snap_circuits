#include "Filters.h"

using namespace std;

MedianFilter &MedianFilter::operator=(const MedianFilter &_mf)
{
    size = _mf.size;
    data = _mf.data;

    return *this;
}

bool MedianFilter::pushData(const int &_point)
{
    if (data.size()>=size)
    {
        data.erase(data.begin());
    }
    data.push_back(_point);

    return true;
};

int MedianFilter::getMedian()
{
    if (data.size() == 0) return -1;

    std::vector<int> tmp = data;
    sort(tmp.begin(),tmp.end());

    // if the size is even, let's keep the leftmost value
    // (i.e. we value the past slight more than the present)
    return tmp[int((tmp.size()-1)/2)];
};

int MedianFilter::getPoint(const int &_i)
{
    return data[_i];
};

std::string MedianFilter::toString()
{
    std::stringstream res;
    res << "Median: " << getMedian() << "\t Buffer: [ ";

    for (int i = 0; i < data.size(); ++i)
    {
        res << data[i] << " ";
    }
    res << "]";

    return res.str();
};

/*************************************************************/

cvPointFilter &cvPointFilter::operator=(const cvPointFilter &_pf)
{
    x=_pf.x;
    y=_pf.y;

    return *this;
}

bool cvPointFilter::pushPoint2f(const cv::Point2f &_point)
{
    return x.pushData(int(_point.x+0.5)) && y.pushData(int(_point.y+0.5));
};

cv::Point2f cvPointFilter::getMedian()
{
    return cv::Point2f(x.getMedian(), y.getMedian());
};

cv::Point2f cvPointFilter::getPoint(const int &_i)
{
    return cv::Point2f(x.getPoint(_i), y.getPoint(_i));
};

std::string cvPointFilter::toString()
{
    std::stringstream res;
    res << "Median: " << getMedian() << "\t Buffer: ( ";

    for (int i = 0; i < x.getDataSize(); ++i)
    {
        res << "[" << x.getPoint(i) << " " << y.getPoint(i) << "]";
    }
    res << ")";

    return res.str();
};
