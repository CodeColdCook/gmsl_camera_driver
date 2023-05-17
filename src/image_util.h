#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

void UndistortImage(const cv::Mat *image, cv::Mat* rectify_image, const cv::Mat camK, const cv::Mat &distort_param) 
{
    cv::Mat map1, map2;
    cv::Size imageSize;
    imageSize = image->size();
    cv::initUndistortRectifyMap(camK, distort_param, cv::Mat(),
        cv::getOptimalNewCameraMatrix(camK, distort_param, imageSize, 1, imageSize, 0),
    imageSize, CV_16SC2, map1, map2);

    cv::remap(*image, *rectify_image, map1, map2, cv::INTER_LINEAR);
}

bool LoadCamConfig(cv::Mat &camK, cv::Mat &camD, const std::string &camera_intrinsic_path)
{
    // read;
    cv::FileStorage fs;
    fs.open(camera_intrinsic_path, cv::FileStorage::READ);
    if ( !fs.isOpened())
    {
        std::cout << "--- LoadIntrinsic: " << "can not open " << camera_intrinsic_path << std::endl;
        fs.release();
        return false;
    }
    try
    {
        fs["K"] >> camK;
        fs["d"] >> camD;
    }
    catch(const cv::Exception& e)
    {
        std::cout << e.what() << std::endl;
        fs.release();
        return false;
    }

    fs.release();
    return true;
}