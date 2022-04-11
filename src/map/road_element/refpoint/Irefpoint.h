#pragma once

#include <string>

namespace nirvana
{
  namespace map
  {
    namespace road
    {
      class IRefPoint
      {
        virtual int GetParentId() = 0;
        virtual void SetId(int id) = 0;
        virtual int GetId() const = 0;
        virtual void SetYaw(double yw) = 0;
        virtual double GetYaw() const = 0;
        virtual void SetCordX(double xx) = 0;
        virtual double GetCordX() const = 0;
        virtual void SetCordY(double yy) = 0;
        virtual double GetCordY() const = 0;
        virtual void SetCordZ(double zz) = 0;
        virtual double GetCordZ() const = 0;
      };
    } // namespace road
  }   // namespace map
} // namespace nirvana