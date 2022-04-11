#pragma once
#include <map>
#include <mutex>
#include <atomic>
#include "../Iroad.h"

namespace nirvana
{
  namespace map
  {
    namespace road
    {
      class RoadSegment : public IRoad
      {
      private:
        /* data */
      public:
        RoadSegment(int id = 0)
            : road_id_(id) {}
        virtual ~RoadSegment()
        {
          Clear();
        }

        virtual int GetId() const;
        virtual void SetId(int id);
        virtual void InsertLaneLine(int lane_line_id);
        virtual void InsertLane(int lane_id);
        virtual std::set<int> GetLaneLineIds() const;
        virtual std::set<int> GetLaneIds() const;
        virtual void DeleteLaneLine(int lane_line_id);
        virtual void DeleteLane(int lane_id);
        virtual void Clear();

      private:
        std::atomic<int> road_id_;
        mutable std::mutex mutex_;
        std::set<int> lane_line_ids_;
        std::set<int> lane_ids_;
      };
    } // namespace road
  }   // namespace map
} // namespace nirvana