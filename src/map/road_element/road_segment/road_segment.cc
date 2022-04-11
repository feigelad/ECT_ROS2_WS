#include "road_segment.h"

namespace nirvana
{
  namespace map
  {
    namespace road
    {
      int RoadSegment::GetId() const
      {
        return road_id_.load();
      }
      void RoadSegment::SetId(int id)
      {
        road_id_.exchange(id);
      }
      void RoadSegment::InsertLaneLine(int lane_line_id)
      {
        std::unique_lock<std::mutex> lck(mutex_);
        lane_line_ids_.emplace(lane_line_id);
      }
      void RoadSegment::InsertLane(int lane_id)
      {
        std::unique_lock<std::mutex> lck(mutex_);
        lane_ids_.emplace(lane_id);
      }
      std::set<int> RoadSegment::GetLaneLineIds() const
      {
        std::unique_lock<std::mutex> lck(mutex_);
        return lane_line_ids_;
      }
      std::set<int> RoadSegment::GetLaneIds() const
      {
        std::unique_lock<std::mutex> lck(mutex_);
        return lane_ids_;
      }
      void RoadSegment::DeleteLaneLine(int lane_line_id)
      {
        std::unique_lock<std::mutex> lck(mutex_);
        if (lane_line_ids_.find(lane_line_id) != lane_line_ids_.end())
          lane_line_ids_.erase(lane_line_id);
      }
      void RoadSegment::DeleteLane(int lane_id)
      {
        std::unique_lock<std::mutex> lck(mutex_);
        if (lane_ids_.find(lane_id) != lane_ids_.end())
          lane_ids_.erase(lane_id);
      }
      void RoadSegment::Clear()
      {
        std::unique_lock<std::mutex> lck(mutex_);
        lane_line_ids_.clear();
        lane_ids_.clear();
      }
    } // namespace road
  }   // namespace map
} // namespace nirvana