#pragma once

#include <string>
#include <memory>
#include <map>
#include <atomic>
#include "ILane.h"
#include "../../common/map_types.h"

namespace nirvana
{
  namespace map
  {
    namespace road
    {
      class Lane : public ILane
      {
      public:
        Lane(int prt_id = 0,
             int lane_id = 0,
             int ref_pt_id = 0,
             int ref_pt_opp_id = 0,
             int left_lane_id = 0,
             int right_lane_id = 0,
             int line1_id = 0,
             int line2_id = 0)
            : parent_id_(prt_id),
              lane_id_(lane_id),
              ref_point_id_(ref_pt_id),
              ref_point_opposite_id_(ref_pt_opp_id),
              left_lane_id_(left_lane_id),
              right_lane_id_(right_lane_id),
              left_lane_line_id_(line1_id),
              right_lane_line_id_(line2_id) {}
        virtual ~Lane() {}

        virtual int GetParentId();
        virtual void SetId(int id);
        virtual void SetLaneLines(int line1_id, int line2_id);
        virtual void SetNextLaneId(int left_lane_id, int right_lane_id);
        virtual void SetRefPointId(int ref, int ref_opposite);
        virtual int GetId();
        virtual int GetLeftLaneId() const;
        virtual int GetRightLaneId() const;
        virtual int GetLeftLaneLineId() const;
        virtual int GetRightLaneLineId() const;
        virtual int GetRefPointId() const;
        virtual int GetRefPointOppositeId() const;
        virtual int GetLeftLaneLineId();
        virtual int GetRightLaneLineId();

      private:
        std::atomic<int> parent_id_;
        std::atomic<int> lane_id_;
        std::atomic<int> ref_point_id_;
        std::atomic<int> ref_point_opposite_id_;
        std::atomic<int> left_lane_id_;
        std::atomic<int> right_lane_id_;
        std::atomic<int> left_lane_line_id_;
        std::atomic<int> right_lane_line_id_;
      };
    } // namespace road
  }   // namespace map
} // namespace nirvana