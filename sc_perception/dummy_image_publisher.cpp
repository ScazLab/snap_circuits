#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char** argv)
{
    std::string name="dummy_image_publisher";

    ros::init(argc, argv, name.c_str());
    ros::NodeHandle nH;

    std::string pub = "/snap_circuits/image_undistorted";
    nH.param<std::string>(("/"+name+"/pub").c_str(), pub, "/snap_circuits/image_undistorted");

    std::string filename="";
       
    // Dirty way to process command line arguments. It seems that
    // there is not a straightforward standard ROS way, unfortunately.
    if (argc>1)
    {
        filename=std::string(argv[1]);
        nH.setParam( ("/"+name+"/filename").c_str(), filename.c_str());
    }

    ROS_INFO("[dummyImagePublisher] Name       set to %s", name.c_str());
    ROS_INFO("[dummyImagePublisher] Filename   set to %s", filename.c_str());
    ROS_INFO("[dummyImagePublisher] Publishing     to %s", pub.c_str());

    ros::Publisher imgPub = nH.advertise<sensor_msgs::Image>(pub.c_str(), 1);
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    while (nH.ok()) 
    {
        std::string new_file;
        nH.param<std::string>(("/"+name+"/filename").c_str(), new_file, filename); 

        if (filename!=new_file)
        {
            filename=new_file;

            cv_bridge::CvImage msgOut;
            msgOut.encoding = sensor_msgs::image_encodings::BGR8;

            cv::Mat mat = cv::imread(filename.c_str(), CV_LOAD_IMAGE_COLOR);

            if (!mat.data)
            {
                ROS_ERROR("Could not find or import the image");
            }
            else
            {
                msgOut.image = mat;

                ROS_INFO("[dummyImagePublisher] Publishing image from %s",new_file.c_str());
                imgPub.publish(msgOut);
            }
        }
        
        loop_rate.sleep();
    }

    ROS_INFO("[dummyImagePublisher] Closing..");
}
