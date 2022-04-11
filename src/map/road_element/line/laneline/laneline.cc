#include "laneline.h"

namespace nirvana
{
  namespace map
  {
    namespace road
    {
      int LaneLine::GetParentId() const
      {
        return parent_id_.load();
      }
      void LaneLine::SetId(int id)
      {
        line_id_ = id;
      }
      int LaneLine::GetId() const
      {
        return line_id_.load();
      }
      void LaneLine::SetCurvType(enum common::geometry::CurvType ct)
      {
      }
      enum common::geometry::CurvType LaneLine::GetCurvType() const
      {
      }
      void LaneLine::SetLineType(enum LineType tp)
      {
        std::unique_lock<std::mutex> lck(mutex_);
        line_type_ = tp;
      }
      enum LineType LaneLine::GetLineType() const
      {
        std::unique_lock<std::mutex> lck(mutex_);
        return line_type_;
      }
      void LaneLine::GernerateLine(const std::vector<LaneLinePoint> &lps)
      {
        std::unique_lock<std::mutex> lck(mutex_);
        line_control_points_ = lps;
      }
      std::vector<LaneLinePoint> LaneLine::GetLine() const
      {
        std::unique_lock<std::mutex> lck(mutex_);
        //TODO:
      }
      double LaneLine::GetLength() const
      {
      }

      std::vector<double> LaneLine::GetLineParam() const
      {
        std::unique_lock<std::mutex> lck(mutex_);
        return param_poly3_;
      }
    } // namespace road
  }   // namespace map
} // namespace nirvana