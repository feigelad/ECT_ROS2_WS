#pragma once

#include <string>

#include "../../map/common/map_types.h"

namespace nirvana
{
  namespace map
  {
    namespace road
    {
      class ILine
      {
      public:
        virtual int GetParentId() const = 0;
        virtual void SetId(int id) = 0;
        virtual int GetId() const = 0;
        virtual void SetCurvType(enum common::geometry::CurvType ct) = 0;
        virtual enum common::geometry::CurvType GetCurvType() const = 0;
        virtual void SetLineType(enum LineType tp) = 0;
        virtual enum LineType GetLineType() const = 0;
        virtual void GernerateLine(const std::vector<LaneLinePoint> &lps) = 0;
        virtual std::vector<LaneLinePoint> GetLine() const = 0;
        virtual double GetLength() const = 0;
        virtual std::vector<double> GetLineParam() const = 0;
      };

    } // namespace road
  }   // namespace map
} // namespace nirvana