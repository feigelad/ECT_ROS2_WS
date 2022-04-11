#include "polyn.h"
#include <math.h>

namespace nirvana
{
  namespace common
  {
    namespace geometry
    {
      bool Polyn::SetPolynDegree(int degree)
      {
        polyn_degree_.exchange(degree);
        factor_vec_.resize(degree + 1);
      }
      void Polyn::SetMinInput(double min)
      {
        input_range_min_.exchange(min);
      }
      void Polyn::SetMaxInput(double max)
      {
        input_range_max_.exchange(max);
      }
      bool Polyn::SetPolynFactor(const std::vector<double> &factor)
      {
        if (GetPolynDegree() + 1 != factor_vec_.size())
          return false;
        std::unique_lock<std::mutex> lck(mutex_);
        factor_vec_ = factor;
        return true;
      }
      int Polyn::GetPolynDegree() const
      {
        return polyn_degree_.load();
      }

      std::vector<double> Polyn::GetPolynFactorVec() const
      {
        std::unique_lock<std::mutex> lck(mutex_);
        return factor_vec_;
      }

      double Polyn::GetMinInput() const
      {
        return input_range_min_.load();
      }

      double Polyn::GetMaxInput() const
      {
        return input_range_max_.load();
      }

      double Polyn::GetValue(double input)
      {
        double res = 0;
        std::unique_lock<std::mutex> lck(mutex_);
        double in = std::min(input_range_max_.load(), std::max(input_range_min_.load(), input));
        for (int i = 0; i < factor_vec_.size(); i++)
        {
          res += factor_vec_[i] * pow(in, i);
        }
        return res;
      }

    } // namespace geometry

  } // namespace common

} // namespace nirvana
