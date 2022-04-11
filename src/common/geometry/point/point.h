#pragma once

#include <atomic>

namespace nirvana
{
  namespace common
  {
    namespace geometry
    {
      template <typename T>
      class Point
      {
      public:
        Point(/* args */)
            : x(0), y(0), z(0) {}
        Point(const Point &p)
        {
          this->x = p.GetX();
          this->y = p.GetY();
          this->z = p.GetZ();
        }
        Point(T xx, T yy, T zz)
        {
          this->x = xx;
          this->y = yy;
          this->z = zz;
        }
        virtual ~Point() {}

        Point &operator=(const Point &p)
        {
          this->x = p.GetX();
          this->y = p.GetY();
          this->z = p.GetZ();
          return *this;
        }

        virtual T GetX() const;
        virtual T GetY() const;
        virtual T GetZ() const;
        virtual void SetX(T xx);
        virtual void SetY(T yy);
        virtual void SetZ(T zz);

      private:
        std::atomic<T> x;
        std::atomic<T> y;
        std::atomic<T> z;
      };

      template <typename T>
      T Point<T>::GetX() const
      {
        return x.load();
      }

      template <typename T>
      T Point<T>::GetY() const
      {
        return y.load();
      }

      template <typename T>
      T Point<T>::GetZ() const
      {
        return z.load();
      }
      template <typename T>
      void Point<T>::SetX(T xx)
      {
        x = xx;
      }

      template <typename T>
      void Point<T>::SetY(T yy)
      {
        y = yy;
      }

      template <typename T>
      void Point<T>::SetZ(T zz)
      {
        z = zz;
      }
    } // namespace geometry
  }   // namespace common
} // namespace nirvana