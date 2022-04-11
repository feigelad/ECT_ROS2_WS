#pragma once

#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include "common/geometry/point/point.h"
#include "../../lane/ILane.h"

namespace nirvana
{
  namespace map
  {
    namespace road
    {
      class LaneCenter
      {
      public:
        enum LaneDirect
        {
          Lane_Direct_Positive = 0,
          Lane_Direct_Nagative,
          Lane_Direct_Both
        };

      public:
        LaneCenter(/* args */);
        LaneCenter(uint16_t id, enum LaneDirect ld, const std::vector<LaneCenterPoint> &lcd)
            : lane_id_(id), lane_direct_(ld), lane_center_(lcd) {}
        LaneCenter(const LaneCenter &lc)
        {
          this->lane_id_ = lc.lane_id_;
          this->lane_direct_ = lc.lane_direct_;
          this->lane_center_ = lc.lane_center_;
        }
        ~LaneCenter();

        uint16_t GetId() const;
        enum LaneDirect GetLaneDirect() const;
        std::vector<LaneCenterPoint> GetLaneCenter() const;
        std::shared_ptr<ILane> GetParent() const;
        void SetId(uint16_t id);
        void SetLaneDirect(enum LaneDirect direct);
        void SetLaneCenter(const std::vector<LaneCenterPoint> &lcd);
        void SetParent(const std::shared_ptr<ILane> &p);

      private:
        mutable std::mutex mutex_;
        uint16_t lane_id_;
        enum LaneDirect lane_direct_;
        std::vector<LaneCenterPoint> lane_center_;
        LaneCenterPoint start_point_;
        LaneCenterPoint end_point_;
        std::shared_ptr<ILane> parent_;
      };
    } // namespace road
  }   // namespace map
} // namespace nirvana