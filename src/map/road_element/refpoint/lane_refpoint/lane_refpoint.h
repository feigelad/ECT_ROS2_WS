#pragma once
#include <string>
#include "../Irefpoint.h"
#include "common/geometry/point/point.h"

namespace nirvana
{
  namespace map
  {
    namespace road
    {
      class LaneRefPoint : public common::geometry::Point<double>, public IRefPoint
      {
      public:
        LaneRefPoint(int parent_id = 0, int id = 0, double xx = 0, double yy = 0, double zz = 0, double yw = 0)
            : parent_id_(parent_id), id_(id), yaw_(yw)
        {
          SetX(xx);
          SetY(yy);
          SetZ(zz);
        }
        LaneRefPoint(const LaneRefPoint &p)
        {
          this->parent_id_.exchange(p.parent_id_.load());
          this->SetX(p.GetX());
          this->SetY(p.GetY());
          this->SetZ(p.GetZ());
          this->SetId(p.GetId());
          this->SetYaw(p.GetYaw());
        }

        virtual ~LaneRefPoint() {}

        LaneRefPoint &operator=(const LaneRefPoint &p)
        {
          this->parent_id_.exchange(p.parent_id_.load());
          this->SetX(p.GetX());
          this->SetY(p.GetY());
          this->SetZ(p.GetZ());
          this->SetId(p.GetId());
          this->SetYaw(p.GetYaw());
          return *this;
        }
        virtual int GetParentId();
        virtual void SetId(int id);
        virtual int GetId() const;
        virtual void SetYaw(double yw);
        virtual double GetYaw() const;
        virtual void SetCordX(double xx);
        virtual double GetCordX() const;
        virtual void SetCordY(double yy);
        virtual double GetCordY() const;
        virtual void SetCordZ(double zz);
        virtual double GetCordZ() const;

      private:
        std::atomic<int> parent_id_;
        std::atomic<int> id_;
        std::atomic<double> yaw_;
      };
    } // namespace road
  }   // namespace map
} // namespace nirvana
