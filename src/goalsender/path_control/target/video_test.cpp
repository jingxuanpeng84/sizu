#include "path_control/include/video_device.hpp"

int main()
{
    videoDevice myvideo = videoDevice("udpsrc address=230.1.1.1 port=1720 multicast-iface=enP4p65s0 ! application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1");
    if (myvideo.startWork())
    {
        std::cout << "open success\n";
        myvideo.openWindowShow();
    }
    // std::this_thread::sleep_for(std::chrono::seconds(9));
    // myvideo.closeWindowShow();
    // std::this_thread::sleep_for(std::chrono::seconds(4));
    // myvideo.endWork();
    // myvideo.watingDeviceEnding();
    // if (myvideo.startWork())
    // {
    //     std::cout << "open success\n";
    //     myvideo.openWindowShow();
    // }
    // std::this_thread::sleep_for(std::chrono::seconds(9));
    // myvideo.closeWindowShow();
    // std::this_thread::sleep_for(std::chrono::seconds(4));


    cv::Mat *frame = new cv::Mat();
    unsigned int dataSize;
    while (1)
    {
        if (myvideo.readDeviceData(frame, dataSize))
        {
            // std::cout << "read success\n";
            cv::imshow("video", *frame);
            cv::waitKey(1);
        }
        else
        {
            // std::cout << "read error\n";
        }
    }
    delete frame;

    myvideo.endWork();
    myvideo.watingDeviceEnding();
    return 0;
}