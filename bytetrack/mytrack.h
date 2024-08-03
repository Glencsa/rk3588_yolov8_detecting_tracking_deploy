#pragma once
#include "include/BYTETracker.h"
#include "commonm.h"
class MyTrack{
public:
    BYTETracker m_byte_tracker;
    std::vector<STrack>m_stracks;
public:
    MyTrack(int frame_rate, int track_buffer);
    MyTrack(){};
    ~MyTrack();
    void Add_frame(const object_detect_result_list &ob_result_list);

};
