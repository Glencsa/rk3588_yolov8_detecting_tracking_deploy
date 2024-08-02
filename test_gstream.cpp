#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <iconv.h>
#include <sstream>
#include <string>
#include <string.h>

using namespace std;
using namespace cv;


int main()
{
    //string gsurl = "v4l2src device=/dev/video21 ! video/x-raw, width=640, height=480, format=(string)YUY2 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

//string gsurl = "v4l2src device=/dev/video21 ! image/jpeg, format=NV12,width=640, height=480 ! queue! jpegparse ! queue ! mppjpegdec ! queue ! videoconvert ! queue ! appsink";

//string gsurl = "rtspsrc location=rtsp://admin:admin@192.168.1.155:554 ! rtph264depay ! queue !  h264parse ! queue ! mppvideodec ! videorate ! videoconvert !  queue !   appsink sync=false";

// string gsurl = "rtspsrc location=rtsp://admin:admin@192.168.1.155:554/ latency=0 ! rtph264depay !  h264parse !  avdec_h264 ! rgaconvert !  video/x-raw, format=(string)BGR  !  appsink sync=false";
    //VideoCapture cap = VideoCapture(21);//sync=false avdec_h264
   // string gsurl = "rtspsrc location=rtsp://192.168.2.119:554/ ! rtph264depay ! queue !  h264parse ! queue ! avdec_h264 ! videorate ! videoconvert !  queue !   appsink sync=false";
     string gsurl =  "rtspsrc location=rtsp://192.168.2.119/554 latency=0 ! rtph264depay !  h264parse !  avdec_h264 ! videoconvert !  queue !  appsink sync=false ";
    VideoCapture cap = VideoCapture(gsurl,cv::CAP_GSTREAMER);
    if(!cap.isOpened())
    {
        std::cout<<"cannot open captrue..."<<std::endl;
        return 0;
    }
     double fps = cap.get(cv::CAP_PROP_FPS);
    std::cout << "Frame rate: " << fps << std::endl;
    
    int fps1 = cap.get(5);
    cout<<"fps:"<<fps1<<endl;
    Mat frame;
    cv::TickMeter tm;
    tm.start();
    int frameCount = 0;
    double totalFps = 0.0;
    bool readreturn = false;
    while(1)
    {  
        readreturn = cap.read(frame);
        cv::imshow("Camera", frame);
        //imshow("RTSP",frame);
        // ���¼�ʱ��
            tm.stop();
            double current_fps = 1000.0 / tm.getTimeMilli();
            tm.reset();
            tm.start();
// �ۼ�֡���ܺ�
            totalFps += current_fps;
            frameCount++;

            // ��ӡƽ��֡����Ϣ
            if (frameCount >= 10) {
                double avgFps = totalFps / frameCount;
                std::cout << "Average FPS: " << avgFps << std::endl;
                frameCount = 0;
                totalFps = 0.0;
            }
            // ��ӡ֡����Ϣ
        //std::cout << "FPS: " << current_fps << std::endl;
        
        if ( cv::waitKey(10) == 27) 
        {
            cout << "Esc key is pressed by user" << endl;
            break;
        }
    }

    cap.release();
    return 0;
}
//g++ test_gstream.cpp `pkg-config --cflags --libs opencv4` --std=c++11
