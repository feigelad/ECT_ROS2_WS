#pragma once

namespace nirvana
{
  namespace common
  {
    namespace geometry
    {

      enum CurvType
      {
        Curv_Straight = 0,
        Curv_Spiral,
        Curv_Arc,
        Curv_Cubic_Polyn,
        Curv_Parametric_Cubic_Polyn
      };

      struct ParaCubicPolyn
      {
        /* curve function [ax, bx, cx, dx, ay, by, cy, dy]
         * px = ax + bx * p + cx * p^2 + dx * p^3
         * py = ay + by * p + cy * p^2 + dy * p^3
         */
        double ax;
        double bx;
        double cx;
        double dx;
        double ay;
        double by;
        double cy;
        double dy;
      };
      class Curv
      {
      public:
      public:
        Curv(/* args */);
        ~Curv();

      private:
        /* data */
      };
    } // namespace geometry
  }   // namespace common
} // namespace nirvana
