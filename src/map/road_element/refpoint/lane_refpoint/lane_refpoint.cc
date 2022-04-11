#include "lane_refpoint.h"

namespace nirvana
{
  namespace map
  {
    namespace road
    {
      int LaneRefPoint::GetParentId()
      {
        return parent_id_.load();
      }
      void LaneRefPoint::SetId(int id)
      {
        id_.exchange(id);
      }
      int LaneRefPoint::GetId() const
      {
        return id_.load();
      }
      void LaneRefPoint::SetYaw(double yw)
      {
        yaw_.exchange(yw);
      }
      double LaneRefPoint::GetYaw() const
      {
        return yaw_.load();
      }
      void LaneRefPoint::SetCordX(double xx)
      {
        this->SetX(xx);
      }
      double LaneRefPoint::GetCordX() const
      {
        return this->GetX();
      }
      void LaneRefPoint::SetCordY(double yy)
      {
        this->SetY(yy);
      }
      double LaneRefPoint::GetCordY() const
      {
        return this->GetY();
      }
      void LaneRefPoint::SetCordZ(double zz)
      {
        this->SetZ(zz);
      }
      double LaneRefPoint::GetCordZ() const
      {
        return this->GetZ();
      }
    } // namespace road
  }   // namespace map
} // namespace nirvana