#include "lanecenter.h"

namespace nirvana
{
  namespace map
  {
    namespace road
    {
      uint16_t LaneCenter::GetId() const
      {
        std::unique_lock<std::mutex> lck(mutex_);
        return lane_id_;
      }
      enum LaneCenter::LaneDirect LaneCenter::GetLaneDirect() const
      {
        std::unique_lock<std::mutex> lck(mutex_);
        return lane_direct_;
      }

      std::vector<LaneCenterPoint> LaneCenter::GetLaneCenter() const
      {
        std::unique_lock<std::mutex> lck(mutex_);
        return lane_center_;
      }

      std::shared_ptr<ILane> LaneCenter::GetParent() const
      {
        std::unique_lock<std::mutex> lck(mutex_);
        return parent_;
      }

      void LaneCenter::SetId(uint16_t id)
      {
        std::unique_lock<std::mutex> lck(mutex_);
        lane_id_ = id;
      }
      void LaneCenter::SetLaneDirect(enum LaneDirect direct)
      {
        std::unique_lock<std::mutex> lck(mutex_);
        lane_direct_ = direct;
      }
      void LaneCenter::SetLaneCenter(const std::vector<LaneCenterPoint> &lcd)
      {
        std::unique_lock<std::mutex> lck(mutex_);
        lane_center_ = lcd;
      }

      void LaneCenter::SetParent(const std::shared_ptr<ILane> &p)
      {
        std::unique_lock<std::mutex> lck(mutex_);
        parent_ = p;
      }
    } // namespace road
  }   // namespace map
} // namespace nirvana