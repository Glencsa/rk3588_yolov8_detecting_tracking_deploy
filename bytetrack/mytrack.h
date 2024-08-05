#pragma once
#include "include/BYTETracker.h"
#include "commonm.h"
#include <vector>
#include <unordered_map>
#include "opencv2/opencv.hpp"


class MyTrack{
public:
    BYTETracker m_byte_tracker;
    std::vector<STrack>m_stracks;
    unordered_map<int,vector<cv::Point>>m_history_trackpoints;
public:
    MyTrack(int frame_rate, int track_buffer);
    MyTrack(){};
    ~MyTrack();
    void Add_frame(const object_detect_result_list &ob_result_list);
    
};
