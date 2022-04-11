#include "lane.h"

namespace nirvana
{
  namespace map
  {
    namespace road
    {
      int Lane::GetParentId()
      {
        return parent_id_.load();
      }

      int Lane::GetId()
      {
        return lane_id_.load();
      }
      void Lane::SetId(int id)
      {
        lane_id_.exchange(id);
      }
      void Lane::SetNextLaneId(int left_lane_id, int right_lane_id)
      {
        left_lane_id_.exchange(left_lane_id);
        right_lane_id_.exchange(right_lane_id);
      }
      void Lane::SetRefPointId(int ref, int ref_opposite)
      {
        ref_point_id_.exchange(ref);
        ref_point_opposite_id_.exchange(ref_opposite);
      }
      int Lane::GetLeftLaneId() const
      {
        return left_lane_id_.load();
      }
      int Lane::GetRightLaneId() const
      {
        return right_lane_id_.load();
      }
      int Lane::GetLeftLaneLineId() const
      {
        return left_lane_line_id_.load();
      }
      int Lane::GetRightLaneLineId() const
      {
        return right_lane_line_id_.load();
      }
      int Lane::GetRefPointId() const
      {
        return ref_point_id_.load();
      }
      int Lane::GetRefPointOppositeId() const
      {
        return ref_point_opposite_id_.load();
      }
      int Lane::GetLeftLaneLineId()
      {
        return left_lane_line_id_.load();
      }
      int Lane::GetRightLaneLineId()
      {
        return right_lane_id_.load();
      }
      void Lane::SetLaneLines(int line1_id, int line2_id)
      {
        left_lane_line_id_.exchange(line1_id);
        right_lane_line_id_.exchange(line2_id);
      }
    } // namespace road
  }   // namespace map
} // namespace nirvana