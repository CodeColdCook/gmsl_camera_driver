//
// Created by alex on 18-12-8.
//
//
// #include <opencv2/opencv.hpp>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>
#include <zconf.h>
#include <csignal>
#include <thread>
#include "MvGmslCamera.h"
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "cv_bridge/cv_bridge.h"

#include "image_util.h"

using namespace std::chrono;

using std::string;
sig_atomic_t exitRequested = 0;
int  fps = 20;
uint64_t timestampbefore = 0;
uint64_t LinuxGetFrameTimeBefore = 0;
uint node_num = 1;
unsigned int tmp_w;
unsigned int tmp_h;
uint cam_num = 1;
uint cam_h = 720;
uint cam_w = 1280;
uint imgWidth{1280};
uint imgHeight{720};
// uint imgWidth{640};
// uint imgHeight{360};
std::string camera_fmt_str="UYVY";
std::string dev_node = "/dev/video0";

std::string cam_cfg_path_0 = 
    "/home/nvidia/features/gmsl_driver_ros/src/opencv_gmsl_test/config/cam_0.yaml";
std::string cam_cfg_path_1 = 
    "/home/nvidia/features/gmsl_driver_ros/src/opencv_gmsl_test/config/cam_1.yaml";
cv::Mat camK_0, camK_1, dist_0, dist_1;

void handler(int) {
    std::cout << "will exit..." << std::endl;
    exitRequested = true;
}

void CheckTimeStampLog(uint64_t timestamp)
{
    uint64_t FrameInterval = 0;
    char buffer[256] = {0};
    uint64_t LinuxFrameInterval{};
    struct timeval cur_time;
    uint64_t LinuxGetFrameTime{};
    uint64_t time_interval{};
    uint64_t FrameTransferDelay{};
    FILE * file_diff = NULL;

    if(0 == timestamp)
    {
        /*camera Data is not available during camera preparation*/
        return;
    }
    FrameInterval = timestamp - timestampbefore;
    timestampbefore = timestamp;
    gettimeofday(&cur_time, NULL);
    LinuxGetFrameTime = cur_time.tv_sec * 1000000000 + cur_time.tv_usec * 1000;
    LinuxFrameInterval = LinuxGetFrameTime - LinuxGetFrameTimeBefore;
    LinuxGetFrameTimeBefore = LinuxGetFrameTime;
    FrameTransferDelay = LinuxGetFrameTime - timestamp;
    time_interval = 1000000000 / fps;
    if((FrameInterval > (time_interval + 5000000) || FrameInterval < (time_interval - 5000000)))
    {
        file_diff = fopen("/tmp/cameras_sdk_demo.log","a+");
        sprintf(buffer,"Timestamp : %ld FrameInterval  :  %ld FrameTransferDelay : %ld LinuxGetFrameTime : %ld LinuxFrameInterval : %ld\n"
                        ,timestamp,FrameInterval,FrameTransferDelay,LinuxGetFrameTime,LinuxFrameInterval);
        fwrite(buffer,sizeof(char),strlen(buffer),file_diff);
        fflush(file_diff);
        fclose(file_diff);
    }
    if(atoi(getenv("CHECK_TIME")))
    {
        printf("Timestamp : %ld FrameInterval : %ld FrameTransferDelay : %ld   LinuxGetFrameTime : %ld LinuxFrameInterval : %ld\n"
                        ,timestamp,FrameInterval,FrameTransferDelay,LinuxGetFrameTime,LinuxFrameInterval);
    }
}

static void
print_usage(void) {
    printf("\n\tUsage: example [OPTIONS]\n\n"
           "\tExample: \n"
           "\t./cameras_opencv_demo -r 25 -d /dev/video0 -s 1280x720\n\n"
           "\tSupported options:\n"
           "\t-d\t\tSet V4l2 video device node\n"
           "\t-s\t\tSet output resolution of video device\n"
           "\t-r\t\tSet renderer frame rate (25 fps by default.)\n"
           "\t-h\t\tPrint this usage\n\n"
           "\tNOTE: It runs infinitely until you terminate it with <ctrl+c>\n");
}

static bool
parse_cmdline(int argc, char **argv) {
    int c;

    if (argc < 2) {
        print_usage();
        exit(EXIT_SUCCESS);
    }

    while ((c = getopt(argc, argv, "d:s:r:f:m:h")) != -1) {
        switch (c) {
            case 'd':
                dev_node = optarg;
                break;
            case 'm':
                node_num = strtol(optarg, NULL, 10);
                break;
            case 's':
                if (sscanf(optarg, "%dx%d",
                           &tmp_w, &tmp_h) != 2) {
                    return false;
                }
            //    ctx->cam_fmt = optarg;
                //todo 1280 is singal camera output width ,it may change in future
                if (tmp_w % 1280 != 0) {
                    print_usage();
                    exit(EXIT_SUCCESS);
                }
                cam_num = tmp_w / 1280;
                break;
            case 'r':
                fps = strtol(optarg, NULL, 10);
                break;
            case 'f':
                camera_fmt_str = optarg;
                break;
            case 'h':
                print_usage();
                exit(EXIT_SUCCESS);
                break;
            default:
                print_usage();
                return false;
        }
    }
    return true;
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "gmsl_camera_driver_node");
    ros::NodeHandle nh;
    std::vector<ros::Publisher> v_pubs;
    // camK_0, camK_1, dist_0, dist_1;
    LoadCamConfig(camK_0, dist_0, cam_cfg_path_0);
    LoadCamConfig(camK_1, dist_1, cam_cfg_path_1);

    uint window_num = 0;

    // if (!parse_cmdline(argc, argv)) {
    //     return -1;
    // }
    // dev_node = "/dev/video0";
    cam_num = 2; 
    fps = 25;

    camera_context_t ctx[] = {
        {
            dev_node : dev_node,
            camera_fmt_str :camera_fmt_str,
            output_fmt_str :"ABGR32",
            cam_num :cam_num,
            cam_w : cam_w,
            cam_h : cam_h,
            out_w :imgWidth,
            out_h :imgHeight,
        },
        {
            dev_node : "/dev/video1",
            camera_fmt_str :camera_fmt_str,
            output_fmt_str :"ABGR32",
            cam_num :cam_num,
            cam_w : cam_w,
            cam_h : cam_h,
            out_w :imgWidth,
            out_h :imgHeight,
        },
    };

    setenv("CHECK_TIME","0",0);
    miivii::MvGmslCamera mvcam(ctx, node_num, fps);

    for(int i = 0; i < node_num; i++){
        window_num += ctx[i].cam_num;
    }

    // pubs
    std::string pubName = "GmslCamera";
    for (uint32_t i = 0; i < window_num; i++) {
        ros::Publisher pub;
        pub = nh.advertise<sensor_msgs::Image>(pubName + std::to_string(i),1);
        v_pubs.push_back(pub);
    }

    bool group_a_key(true), group_b_key(false);
    std::cout << "Camera type in group A is " << mvcam.GetCameraType(group_a_key) << std::endl;
    std::cout << "Camera type in group B is " << mvcam.GetCameraType(group_b_key) << std::endl;

    std::string windowName("DisplayCamera ");
    for (uint32_t i = 0; i < window_num; i++) {
        cv::namedWindow(windowName + std::to_string(i), cv::WindowFlags::WINDOW_AUTOSIZE);
        cv::moveWindow(windowName + std::to_string(i), 200 * i, 200 * i);
    }
    cv::Mat outMat[window_num];
    uint8_t *outbuf[window_num];
    cv::Mat imgbuf[window_num];
    signal(SIGINT, &handler);
    bool quit = false;
    uint64_t timestamp{};
    sleep(3);  // camera ready time

    while (!quit) {
        if (exitRequested) {
            quit = true;
            break;
        }
        if (mvcam.GetImageCvMat(outMat, timestamp)) {
            for (uint32_t i = 0; i < window_num; i++) {
                if (ctx[0].output_fmt_str == "UYVY") {
                    cv::cvtColor(outMat[i], outMat[i], cv::COLOR_YUV2BGR_UYVY);
                    // cv::imshow(windowName + std::to_string(i), outMat[i]);
                } else if (ctx[0].output_fmt_str == "ABGR32") {
                    cv::cvtColor(outMat[i], outMat[i], cv::COLOR_RGBA2BGR);
                    // cv::imshow(windowName + std::to_string(i), outMat[i]);
                }
            }
            CheckTimeStampLog(timestamp);
        } else {
            std::cerr << "Can't get image form camera." << std::endl;
        }
        if (cv::waitKey(1) == 27) {// Wait for 'esc' key press to exit
            break;
        }

//        }
        /*use raw data to get image*/

            if (mvcam.GetImagePtr(outbuf, timestamp)) {
                for (uint32_t i = 0; i < window_num; i++) {
                    if (ctx[0].output_fmt_str == "UYVY") {
                        imgbuf[i] = cv::Mat(imgHeight, imgWidth, CV_8UC2, outbuf[i]);
                        cv::Mat mrgba(imgHeight, imgWidth, CV_8UC3);
                        cv::cvtColor(imgbuf[i], mrgba, cv::COLOR_YUV2BGR_UYVY);
                        // cv::imshow(windowName + std::to_string(i), mrgba);
                        if(ros::ok())
                        {
                            if(i == 0)
                            {
                                UndistortImage(&mrgba, &mrgba, camK_0, dist_0);
                            }
                            else if(i == 1)
                            {
                                UndistortImage(&mrgba, &mrgba, camK_1, dist_1);
                            }
                            if(mrgba.empty())
                                break;
                            sensor_msgs::ImagePtr img_bgr_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mrgba).toImageMsg();
                            img_bgr_msg->header.frame_id = pubName;
                            img_bgr_msg->header.stamp = ros::Time::now();
                            v_pubs[i].publish(img_bgr_msg);
                        }
                        cv::imshow(windowName + std::to_string(i), mrgba);
                        
                    } else if (ctx[0].output_fmt_str == "ABGR32") {
                        imgbuf[i] = cv::Mat(imgHeight, imgWidth, CV_8UC4, outbuf[i]);
                        cv::cvtColor(imgbuf[i], imgbuf[i], cv::COLOR_RGBA2BGR);
                        // cv::imshow(windowName + std::to_string(i), imgbuf[i]);
                        if(ros::ok())
                        {
                            if(i == 0)
                            {
                                UndistortImage(&imgbuf[i], &imgbuf[i], camK_0, dist_0);
                            }
                            else if(i == 1)
                            {
                                UndistortImage(&imgbuf[i], &imgbuf[i], camK_1, dist_1);
                            }
                            if(imgbuf[i].empty())
                                break;
                            sensor_msgs::ImagePtr img_bgr_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgbuf[i]).toImageMsg();
                            img_bgr_msg->header.frame_id = pubName;
                            img_bgr_msg->header.stamp = ros::Time::now();
                            v_pubs[i].publish(img_bgr_msg);
                        }
                        cv::imshow(windowName + std::to_string(i), imgbuf[i]);
                    }
                }
                CheckTimeStampLog(timestamp);
            } else {
                std::cerr << "Can't get image form camera." << std::endl;
            }

        if (cv::waitKey(1) == 27) {// Wait for 'esc' key press to exit
            break;
        }
    }
    return 0;
}
