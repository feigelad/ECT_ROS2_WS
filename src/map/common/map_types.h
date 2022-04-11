#pragma once

#include <string>
#include "../../common/geometry/point/point.h"
// #include "common/geometry/curv/curv.h"

namespace nirvana
{
  namespace map
  {
    enum MapCmdCode
    {
      MapSuspend = 0,
      MapCreate,
      MapUse,
      MapLoad,
      MapSelect,
      MapCollect,
      MapSave,
      MapClear
    };

    enum LineType
    {
      Dash_Line = 0,
      Double_Dash_Line,
      Solid_Line,
      Double_Solid_Line
    };

    enum Road_Type
    {
      Road_Normal = 0,
      Road_Junction,
      Road_Area
    };

    class LaneLinePoint
        : public common::geometry::Point<double>
    {
    public:
      LaneLinePoint()
          : Point<double>(), d(0) {}
      LaneLinePoint(double xx, double yy, double zz, double dd)
          : Point<double>(xx, yy, zz), d(dd) {}
      LaneLinePoint(const LaneLinePoint &p)
      {
        this->SetX(p.GetX());
        this->SetY(p.GetY());
        this->SetZ(p.GetZ());
        this->d.exchange(p.d.load());
      }
      virtual ~LaneLinePoint() {}

      LaneLinePoint &operator=(const LaneLinePoint &p)
      {
        this->SetX(p.GetX());
        this->SetY(p.GetY());
        this->SetZ(p.GetZ());
        this->d.exchange(p.d.load());
        return *this;
      }
      virtual void SetDirect(double dd)
      {
        d = dd;
      }

      virtual double GetDirect() const
      {
        return d.load();
      }

    private:
      std::atomic<double> d;
    };

    class LaneLineRefPoint
        : public LaneLinePoint
    {
    public:
      LaneLineRefPoint()
          : LaneLinePoint(), id_(0) {}
      LaneLineRefPoint(uint16_t id, double xx, double yy, double zz, double dd)
          : LaneLinePoint(xx, yy, zz, dd), id_(id) {}
      LaneLineRefPoint(const LaneLineRefPoint &p)
      {
        this->SetX(p.GetX());
        this->SetY(p.GetY());
        this->SetZ(p.GetZ());
        this->SetDirect(p.GetDirect());
        this->id_.exchange(p.id_.load());
      }
      virtual ~LaneLineRefPoint() {}

      LaneLineRefPoint &operator=(const LaneLineRefPoint &p)
      {
        this->SetX(p.GetX());
        this->SetY(p.GetY());
        this->SetZ(p.GetZ());
        this->SetDirect(p.GetDirect());
        this->id_.exchange(p.id_.load());
        return *this;
      }

      virtual void SetId(uint16_t id)
      {
        id_ = id;
      }

      virtual uint16_t GetId() const
      {
        return id_.load();
      }

    private:
      std::atomic<int> id_;
    };

    class LaneCenterPoint
        : public common::geometry::Point<double>
    {
    public:
      LaneCenterPoint()
          : Point<double>(), yaw(0), s(0) {}
      LaneCenterPoint(double xx, double yy, double zz, double yw, double ss)
          : Point<double>(xx, yy, zz), yaw(yw), s(ss) {}
      LaneCenterPoint(const LaneCenterPoint &lcp)
          : Point<double>(lcp.GetX(), lcp.GetY(), lcp.GetZ())
      {
        yaw = lcp.GetYaw();
        s = lcp.GetS();
      }
      LaneCenterPoint &operator=(const LaneCenterPoint &p)
      {
        this->SetX(p.GetX());
        this->SetY(p.GetY());
        this->SetZ(p.GetZ());
        this->SetS(p.GetS());
        this->SetYaw(p.GetYaw());
        return *this;
      }
      virtual ~LaneCenterPoint() {}
      virtual void SetS(double ss)
      {
        s = ss;
      }

      virtual double GetS() const
      {
        return s.load();
      }

      virtual void SetYaw(double yw)
      {
        yaw = yw;
      }

      virtual double GetYaw() const
      {
        return yaw.load();
      }

    private:
      std::atomic<double> yaw;
      std::atomic<double> s;
    };

    class LaneRefPoint
        : public LaneCenterPoint
    {
    public:
      LaneRefPoint()
          : LaneCenterPoint(), id_(0) {}
      LaneRefPoint(uint16_t id, double xx, double yy, double zz, double yw, double ss)
          : LaneCenterPoint(xx, yy, zz, yw, ss), id_(id) {}
      LaneRefPoint(const LaneRefPoint &p)
      {
        this->SetX(p.GetX());
        this->SetY(p.GetY());
        this->SetZ(p.GetZ());
        this->SetS(p.GetS());
        this->SetYaw(p.GetYaw());
        this->id_.exchange(p.id_.load());
      }

      virtual ~LaneRefPoint() {}

      LaneRefPoint &operator=(const LaneRefPoint &p)
      {
        this->SetX(p.GetX());
        this->SetY(p.GetY());
        this->SetZ(p.GetZ());
        this->SetS(p.GetS());
        this->SetYaw(p.GetYaw());
        this->id_.exchange(p.id_.load());
        return *this;
      }

      virtual void SetId(uint16_t id)
      {
        id_.exchange(id);
      }

      virtual uint16_t GetId() const
      {
        return id_.load();
      }

    private:
      std::atomic<uint16_t> id_;
    };

  } // namespace map
} // namespace nirvana
