#pragma once

#include <string>
#include <vector>
#include <mutex>
#include "../ILine.h"
#include "common/geometry/curv/curv.h"

namespace nirvana
{
  namespace map
  {
    namespace road
    {
      class LaneLine : public ILine
      {
      public:
        LaneLine()
            : line_id_(0),
              curv_type_(common::geometry::Curv_Straight),
              line_type_(Dash_Line),
              line_length_(0) {}
        virtual ~LaneLine();
        virtual int GetParentId() const;
        virtual void SetId(int id);
        virtual int GetId() const;
        virtual void SetCurvType(enum common::geometry::CurvType ct);
        virtual enum common::geometry::CurvType GetCurvType() const;
        virtual void SetLineType(enum LineType tp);
        virtual enum LineType GetLineType() const;
        virtual void GernerateLine(const std::vector<LaneLinePoint> &lps);
        virtual std::vector<LaneLinePoint> GetLine() const;
        virtual double GetLength() const;
        virtual std::vector<double> GetLineParam() const;

      private:
        std::atomic<int> line_id_;
        std::atomic<int> parent_id_;
        mutable std::mutex mutex_;
        enum common::geometry::CurvType curv_type_;
        enum LineType line_type_;
        double line_length_;
        std::vector<LaneLinePoint> line_control_points_;
        std::vector<double> param_poly3_; /* curve function [ax, bx, cx, dx, ay, by, cy, dy]
                                           * px = ax + bx * p + cx * p^2 + dx * p^3
                                           * py = ay + by * p + cy * p^2 + dy * p^3
                                           */
      };

    } // namespace road
  }   // namespace map
} // namespace nirvana