#pragma once

#include <string>
#include <atomic>
#include <memory>

namespace nirvana
{
  namespace map
  {
    namespace road
    {
      class ILane
      {
      public:
        virtual int GetParentId() = 0;
        virtual void SetId(int id) = 0;
        virtual void SetLaneLines(int line1_id, int line2_id) = 0;
        virtual void SetNextLaneId(int left_lane_id, int right_lane_id) = 0;
        virtual void SetRefPointId(int ref, int ref_opposite) = 0;
        virtual int GetId() = 0;
        virtual int GetLeftLaneId() const = 0;
        virtual int GetRightLaneId() const = 0;
        virtual int GetLeftLaneLineId() const = 0;
        virtual int GetRightLaneLineId() const = 0;
        virtual int GetRefPointId() const = 0;
        virtual int GetRefPointOppositeId() const = 0;
        virtual int GetLeftLaneLineId() = 0;
        virtual int GetRightLaneLineId() = 0;
      };
    } // namespace road
  }   // namespace map
} // namespace nirvana