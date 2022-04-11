#pragma once

#include <atomic>
#include <mutex>
#include <vector>
// #include <eigen3/Eigen/Core>

namespace nirvana
{
  namespace common
  {
    namespace geometry
    {
      class Polyn
      {
      public:
        Polyn()
        {
          input_range_min_.exchange(0 - __DBL_MAX__);
          input_range_max_.exchange(__DBL_MAX__);
          polyn_degree_.exchange(0);
          factor_vec_.clear();
        }
        Polyn(int degree)
        {
          input_range_min_.exchange(0 - __DBL_MAX__);
          input_range_max_.exchange(__DBL_MAX__);
          polyn_degree_.exchange(degree);
          factor_vec_.clear();
          factor_vec_.resize(degree + 1);
        }
        Polyn(int degree, double min, double max)
        {
          input_range_min_.exchange(min);
          input_range_max_.exchange(max);
          polyn_degree_.exchange(degree);
          factor_vec_.clear();
          factor_vec_.resize(degree + 1);
        }
        Polyn(const Polyn &p)
        {
          this->polyn_degree_.exchange(p.GetPolynDegree());
          this->input_range_min_.exchange(p.GetMinInput());
          this->input_range_max_.exchange(p.GetMaxInput());
          this->factor_vec_ = p.GetPolynFactorVec();
        }
        virtual ~Polyn() {}

        Polyn &operator=(const Polyn &p)
        {
          this->polyn_degree_.exchange(p.GetPolynDegree());
          this->input_range_min_.exchange(p.GetMinInput());
          this->input_range_max_.exchange(p.GetMaxInput());
          this->factor_vec_ = p.GetPolynFactorVec();
          return *this;
        }

        virtual bool SetPolynDegree(int degree);
        virtual void SetMinInput(double min);
        virtual void SetMaxInput(double max);
        virtual bool SetPolynFactor(const std::vector<double> &factor);
        virtual int GetPolynDegree() const;
        virtual std::vector<double> GetPolynFactorVec() const;
        virtual double GetMinInput() const;
        virtual double GetMaxInput() const;
        virtual double GetValue(double input);

      private:
        std::atomic<int> polyn_degree_;
        std::atomic<double> input_range_min_;
        std::atomic<double> input_range_max_;
        mutable std::mutex mutex_;
        std::vector<double> factor_vec_;
      };
    } // namespace geometry
  }   // namespace common
} // namespace nirvana