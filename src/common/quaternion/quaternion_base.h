#pragma once

#include <mutex>
#include <atomic>
#include <eigen3/Eigen/Core>
// #include ""

namespace nirvana
{
  namespace common
  {
    class QuaternionBase
    {
    public:
      QuaternionBase()
          : x_(0.), y_(0.), z_(0.), w_(0.) {}
      QuaternionBase(double x, double y, double z, double w)
          : x_(x), y_(y), z_(z), w_(w) {}
      QuaternionBase(const QuaternionBase &rq)
      {
        this->x_ = rq.x_;
        this->y_ = rq.y_;
        this->z_ = rq.z_;
        this->w_ = rq.w_;
      }
      virtual ~QuaternionBase() {}

      double GetQuaternionX() const;
      double GetQuaternionY() const;
      double GetQuaternionZ() const;
      double GetQuaternionW() const;
      void GetQuaternionW(double &x, double &y, double &z, double &w);
      Eigen::Matrix4d GetMatrix() const;
      void GetAxisAngle(Eigen::Vector3d &axis, float &angle);
      QuaternionBase GetConjugate();
      QuaternionBase operator*(const QuaternionBase &rq) const;
      Eigen::Vector3d operator*(const Eigen::Vector3d &rq) const;
      void FromAxis(const Eigen::Vector3d &v, float angle);
      void FromEuler(float pitch, float yaw, float roll);

    private:
      void Normalise();
      void NomaliseVector3(Eigen::Vector3d &vn);

    private:
      mutable std::mutex mutex_;
      double x_;
      double y_;
      double z_;
      double w_;
    };
  }
}
