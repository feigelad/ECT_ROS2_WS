#include "quaternion_base.h"

namespace nirvana
{
  namespace common
  {
    double QuaternionBase::GetQuaternionX() const
    {
      std::unique_lock<std::mutex> lck(mutex_);
      return x_;
    }
    double QuaternionBase::GetQuaternionY() const
    {
      std::unique_lock<std::mutex> lck(mutex_);
      return y_;
    }
    double QuaternionBase::GetQuaternionZ() const
    {
      std::unique_lock<std::mutex> lck(mutex_);
      return z_;
    }
    double QuaternionBase::GetQuaternionW() const
    {
      std::unique_lock<std::mutex> lck(mutex_);
      return w_;
    }
    void QuaternionBase::GetQuaternionW(double &x, double &y, double &z, double &w)
    {
      std::unique_lock<std::mutex> lck(mutex_);
      x = x_;
      y = y_;
      z = z_;
      w = w_;
    }
    Eigen::Matrix4d QuaternionBase::GetMatrix() const
    {
      std::unique_lock<std::mutex> lck(mutex_);
      Eigen::Matrix4d matRes;
      float x2 = x_ * x_;
      float y2 = y_ * y_;
      float z2 = z_ * z_;
      float xy = x_ * y_;
      float xz = x_ * z_;
      float yz = y_ * z_;
      float wx = w_ * x_;
      float wy = w_ * y_;
      float wz = w_ * z_;

      // This calculation would be a lot more complicated for non-unit length quaternions
      // Note: The constructor of Matrix4 expects the Matrix in column-major format like expected by
      //   OpenGL
      matRes << 1.0f - 2.0f * (y2 + z2), 2.0f * (xy - wz), 2.0f * (xz + wy), 0.0f, 2.0f * (xy + wz), 1.0f - 2.0f * (x2 + z2), 2.0f * (yz - wx), 0.0f, 2.0f * (xz - wy), 2.0f * (yz + wx), 1.0f - 2.0f * (x2 + y2), 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;
      return matRes;
    }
    void QuaternionBase::GetAxisAngle(Eigen::Vector3d &axis, float &angle)
    {
      std::unique_lock<std::mutex> lck(mutex_);
      float scale = sqrt(x_ * x_ + y_ * y_ + z_ * z_);
      axis[0] = x_ / scale;
      axis[1] = y_ / scale;
      axis[2] = z_ / scale;
      angle = acos(w_) * 2.0f;
    }
    QuaternionBase QuaternionBase::GetConjugate()
    {
      std::unique_lock<std::mutex> lck(mutex_);
      return QuaternionBase(-1.0 * x_, -1.0 * y_, -1.0 * z_, w_);
    }
    QuaternionBase QuaternionBase::operator*(const QuaternionBase &rq) const
    {
      // the constructor takes its arguments as (x, y, z, w)
      double x, y, z, w;
      {
        std::unique_lock<std::mutex> lck(mutex_);
        x = x_;
        y = y_;
        z = z_;
        w = w_;
      }

      return QuaternionBase(w * rq.GetQuaternionX() + x * rq.GetQuaternionW() + y * rq.GetQuaternionZ() - z * rq.GetQuaternionY(),
                            w * rq.GetQuaternionY() + y * rq.GetQuaternionW() + z * rq.GetQuaternionX() - x * rq.GetQuaternionZ(),
                            w * rq.GetQuaternionZ() + z * rq.GetQuaternionW() + x * rq.GetQuaternionY() - y * rq.GetQuaternionX(),
                            w * rq.GetQuaternionW() - x * rq.GetQuaternionX() - y * rq.GetQuaternionY() - z * rq.GetQuaternionZ());
    }
    Eigen::Vector3d QuaternionBase::operator*(const Eigen::Vector3d &vec) const
    {
      
      Eigen::Vector3d vn(vec);
      // vn.normalise();

      // QuaternionBase vecQuat(vn[0], vn[1], vn[2], 0.);

      // QuaternionBase resQuat = vecQuat * GetConjugate();
      // resQuat = *this * resQuat;

      // return (Eigen::Vector3d(resQuat.x, resQuat.y, resQuat.z));
      return vn;
    }
    void QuaternionBase::FromAxis(const Eigen::Vector3d &v, float angle)
    {
      angle *= 0.5f;
      Eigen::Vector3d vn(v);
      //nomalise vn
      NomaliseVector3(vn);

      float sinAngle = sin(angle);
      std::unique_lock<std::mutex> lck(mutex_);
      x_ = (vn[0] * sinAngle);
      y_ = (vn[1] * sinAngle);
      z_ = (vn[2] * sinAngle);
      w_ = (cos(angle));
    }
    void QuaternionBase::FromEuler(float pitch, float yaw, float roll)
    {
      // Basically we create 3 Quaternions, one for pitch, one for yaw, one for roll
      // and multiply those together.
      // the calculation below does the same, just shorter
      float pi_over_180 = M_PI / 180.0;

      float p = pitch * pi_over_180 / 2.0;
      float y = yaw * pi_over_180 / 2.0;
      float r = roll * pi_over_180 / 2.0;

      float sinp = sin(p);
      float siny = sin(y);
      float sinr = sin(r);
      float cosp = cos(p);
      float cosy = cos(y);
      float cosr = cos(r);

      {
        std::unique_lock<std::mutex> lck(mutex_);
        this->x_ = (sinr * cosp * cosy - cosr * sinp * siny);
        this->y_ = (cosr * sinp * cosy + sinr * cosp * siny);
        this->z_ = (cosr * cosp * siny - sinr * sinp * cosy);
        this->w_ = (cosr * cosp * cosy + sinr * sinp * siny);
      }
      Normalise();
    }

    void QuaternionBase::Normalise()
    {
      std::unique_lock<std::mutex> lck(mutex_);
      float mag2 = x_ * x_ + y_ * y_ + z_ * z_ + w_ * w_;
      float mag = sqrt(mag2);
      if (mag2 > 0)
      {
        x_ /= mag;
        y_ /= mag;
        z_ /= mag;
        w_ /= mag;
      }
    }
    void QuaternionBase::NomaliseVector3(Eigen::Vector3d &vn)
    {
      float mag2 = vn[0] * vn[0] + vn[1] * vn[1] + vn[2] * vn[2];
      float mag = sqrt(mag2);
      if (mag2 > 0)
      {
        vn[0] /= mag;
        vn[1] /= mag;
        vn[2] /= mag;
      }
    }
  }
}