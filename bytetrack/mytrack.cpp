#include "mytrack.h"

MyTrack::MyTrack(int frame_rate = 30, int track_buffer = 30)
{
    this->m_byte_tracker=BYTETracker(frame_rate,track_buffer);
}

MyTrack::~MyTrack()
{
}

void MyTrack::Add_frame(const object_detect_result_list &ob_result)
{
    int obj_num=ob_result.count;
    std::vector<Object>objects_lst;
    for (int i = 0; i < obj_num; i++)
    {
        const object_detect_result& res=ob_result.results[i];
        Object obj;
        obj.rect=cv::Rect_<float>(res.box.left,res.box.top,res.box.right-res.box.left,res.box.bottom-res.box.top);
        obj.label=res.cls_id;
        obj.prob=res.prop;
        objects_lst.push_back(obj);
        // this->m_byte_tracker.update()
    }
    m_stracks= m_byte_tracker.update(objects_lst);
    //记录历史轨迹
    for (int i = 0; i < m_stracks.size(); i++)
    {
        int st_id= m_stracks[i].track_id;
        m_history_trackpoints[st_id].push_back(
            cv::Point(
                (int(m_stracks[i].tlbr[0]+m_stracks[i].tlbr[2])/2+0.5),
                (int(m_stracks[i].tlbr[1]+m_stracks[i].tlbr[3])/2+0.5)
            )
        );
    }
    
}
