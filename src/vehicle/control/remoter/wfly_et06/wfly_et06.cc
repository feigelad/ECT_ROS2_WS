#include "wfly_et06.h"
#include "message/msg/ecar_chassis_stat.hpp"

#include <math.h>

namespace nirvana
{
  namespace vehicle
  {
    namespace control
    {
      namespace remoter
      {
        bool WflyEt06::Init(float freq_threshold, float chan1_default, float chan1_max, float chan1_min,
                            float chan2_default, float chan2_max, float chan2_min,
                            float chan3_default, float chan3_max, float chan3_min,
                            float chan4_default, float chan4_max, float chan4_min,
                            float chan5_default, float chan5_max, float chan5_min,
                            float chan6_default, float chan6_max, float chan6_min)
        {
          freq_threshold_.exchange(freq_threshold);
          chan1_duty_default_.exchange(chan1_default);
          chan1_duty_range_max_.exchange(chan1_max);
          chan1_duty_range_min_.exchange(chan1_min);
          chan2_duty_default_.exchange(chan2_default);
          chan2_duty_range_max_.exchange(chan2_max);
          chan2_duty_range_min_.exchange(chan2_min);
          chan3_duty_default_.exchange(chan3_default);
          chan3_duty_range_max_.exchange(chan3_max);
          chan3_duty_range_min_.exchange(chan3_min);
          chan4_duty_default_.exchange(chan4_default);
          chan4_duty_range_max_.exchange(chan4_max);
          chan4_duty_range_min_.exchange(chan4_min);
          chan5_duty_default_.exchange(chan5_default);
          chan5_duty_range_max_.exchange(chan5_max);
          chan5_duty_range_min_.exchange(chan5_min);
          chan6_duty_default_.exchange(chan6_default);
          chan6_duty_range_max_.exchange(chan6_max);
          chan6_duty_range_min_.exchange(chan6_min);
          return true;
        }

        bool WflyEt06::RemoterEnableRq(double stamp, void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;
          static double s_start_stamp = stamp;

          float chan1_freq_val = ecar_stat->remoter_stat1_msg.cdcu_rmtchn1_freq.value;
          float chan2_freq_val = ecar_stat->remoter_stat1_msg.cdcu_rmtchn2_freq.value;
          float chan3_freq_val = ecar_stat->remoter_stat2_msg.cdcu_rmtchn3_freq.value;
          float chan4_freq_val = ecar_stat->remoter_stat2_msg.cdcu_rmtchn4_freq.value;
          float chan5_freq_val = ecar_stat->remoter_stat3_msg.cdcu_rmtchn5_freq.value;
          float chan6_freq_val = ecar_stat->remoter_stat3_msg.cdcu_rmtchn6_freq.value;

          bool is_freq_error = chan1_freq_val < freq_threshold_.load() || chan2_freq_val < freq_threshold_.load() || chan3_freq_val < freq_threshold_.load() || chan4_freq_val < freq_threshold_.load() || chan5_freq_val < freq_threshold_.load() || chan6_freq_val < freq_threshold_.load();

          float chan2_duty_val = ecar_stat->remoter_stat1_msg.cdcu_rmtchn2_duty.value;
          float chan4_duty_val = ecar_stat->remoter_stat2_msg.cdcu_rmtchn4_duty.value;
          float chan5_duty_val = ecar_stat->remoter_stat3_msg.cdcu_rmtchn5_duty.value;
          float chan6_duty_val = ecar_stat->remoter_stat3_msg.cdcu_rmtchn6_duty.value;

          float chan2_thres = chan2_duty_default_.load() + (chan2_duty_range_max_.load() - chan2_duty_default_.load()) * 0.9;
          float chan4_thres = chan4_duty_default_.load() + (chan4_duty_range_min_.load() - chan4_duty_default_.load()) * 0.9;
          float chan5_thres = chan5_duty_default_.load();
          float chan6_thres_inc = (chan6_duty_range_max_.load() - chan6_duty_default_.load()) * 0.5;

          if (chan5_duty_val > chan5_thres || chan2_duty_val < chan2_thres || chan4_duty_val > chan4_thres || fabs(chan6_duty_val - chan6_duty_default_.load()) > chan6_thres_inc)
            s_start_stamp = stamp;
          if (stamp > s_start_stamp + 3)
            auto_mode_.exchange(true);

          if (is_freq_error)
            auto_mode_.exchange(false);

          if (chan5_duty_val > chan5_thres)
            auto_mode_.exchange(false);

          return auto_mode_.load();
        }

        float WflyEt06::TargetDrive(int gear, void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;

          float chan3_duty_val = ecar_stat->remoter_stat2_msg.cdcu_rmtchn3_duty.value;
          float zero_range = fabs(chan3_duty_range_max_.load() - chan3_duty_range_min_.load()) * 0.05;
          if (fabs(chan3_duty_val - chan3_duty_default_.load()) < zero_range)
            return 0;

          if ((chan3_duty_val - chan3_duty_default_.load()) > zero_range)
            return 100 * gear * (chan3_duty_val - chan3_duty_default_.load() - zero_range) / (chan3_duty_range_max_.load() - chan3_duty_default_.load() - zero_range);

          if ((chan3_duty_val - chan3_duty_default_.load()) < (-1 * zero_range))
            return -100 * gear * (chan3_duty_val - chan3_duty_default_.load() + zero_range) / (chan3_duty_range_min_.load() - chan3_duty_default_.load() + zero_range);
        }

        int WflyEt06::TargetGear(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;

          float chan6_duty_val = ecar_stat->remoter_stat3_msg.cdcu_rmtchn6_duty.value;

          float chan6_thres_max = chan6_duty_default_.load() + (chan6_duty_range_max_.load() - chan6_duty_default_.load()) * 0.5;
          float chan6_thres_min = chan6_duty_default_.load() + (chan6_duty_range_min_.load() - chan6_duty_default_.load()) * 0.5;

          if (chan6_duty_val > chan6_thres_max)
            return 1;
          else if (chan6_duty_val < chan6_thres_min)
            return -1;
          else
            return 0;
        }

        float WflyEt06::TargetSteer(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;

          float chan1_duty_val = ecar_stat->remoter_stat1_msg.cdcu_rmtchn1_duty.value;

          float chan1_thres_max = chan1_duty_default_.load() + (chan1_duty_range_max_.load() - chan1_duty_default_.load()) * 0.05;
          float chan1_thres_min = chan1_duty_default_.load() + (chan1_duty_range_min_.load() - chan1_duty_default_.load()) * 0.05;

          if (chan1_duty_val > chan1_thres_max)
            return 30 * (chan1_duty_val - chan1_thres_max) / (chan1_duty_range_max_.load() - chan1_thres_max);
          else if (chan1_duty_val < chan1_thres_min)
            return -30 * (chan1_duty_val - chan1_thres_min) / (chan1_duty_range_min_.load() - chan1_thres_min);
          else
            return 0;
        }

        float WflyEt06::TargetBrake(void *const stat)
        {
          ::message::msg::EcarChassisStat *ecar_stat = (::message::msg::EcarChassisStat *)stat;

          float chan2_duty_val = ecar_stat->remoter_stat1_msg.cdcu_rmtchn2_duty.value;

          float chan2_thres_max = chan2_duty_default_.load() + (chan2_duty_range_max_.load() - chan2_duty_default_.load()) * 0.05;
          // float chan2_thres_min = chan2_duty_default_.load() + (chan2_duty_range_min_.load() - chan2_duty_default_.load()) * 0.05;

          if (chan2_duty_val > chan2_thres_max)
            return 80 * (chan2_duty_val - chan2_thres_max) / (chan2_duty_range_max_.load() - chan2_thres_max);
          else
            return 0;
        }
      }
    }
  }
}