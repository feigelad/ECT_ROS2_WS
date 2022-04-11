#include "can_message.h"
#include <numeric>
#include <type_traits>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      size_t CanMessage::SignalCount() const
      {
        size_t res = 0;
        std::unique_lock<std::mutex> lck(mutex_);
        for (auto itr = signal_map_.begin(); itr != signal_map_.end(); ++itr)
        {
          res += itr->second.size();
        }
        return res;
      }
      void CanMessage::Parse(const uint8_t *data, uint8_t len, double stamp)
      {
        // if(id != ID())
        //   return;
        std::unique_lock<std::mutex> lck(mutex_);
        for (auto itr = signal_map_.begin();
             itr != signal_map_.end(); itr++)
        {
          for (auto itr1 = itr->second.begin();
               itr1 != itr->second.end(); itr1++)
          {
            if (itr->first == typeid(bool).name())
            {
              CanSignal<bool> *sig = (CanSignal<bool> *)itr1->second.get();
              sig->Parse(data, len, stamp);
              sig = nullptr;
            }
            else if (itr->first == typeid(uint8_t).name())
            {
              CanSignal<uint8_t> *sig = (CanSignal<uint8_t> *)itr1->second.get();
              sig->Parse(data, len, stamp);
              sig = nullptr;
            }
            else if (itr->first == typeid(uint16_t).name())
            {
              CanSignal<uint16_t> *sig = (CanSignal<uint16_t> *)itr1->second.get();
              sig->Parse(data, len, stamp);
              sig = nullptr;
            }
            else if (itr->first == typeid(uint32_t).name())
            {
              CanSignal<uint32_t> *sig = (CanSignal<uint32_t> *)itr1->second.get();
              sig->Parse(data, len, stamp);
              sig = nullptr;
            }
            else if (itr->first == typeid(uint64_t).name())
            {
              CanSignal<uint64_t> *sig = (CanSignal<uint64_t> *)itr1->second.get();
              sig->Parse(data, len, stamp);
              sig = nullptr;
            }
            else if (itr->first == typeid(int8_t).name())
            {
              CanSignal<int8_t> *sig = (CanSignal<int8_t> *)itr1->second.get();
              sig->Parse(data, len, stamp);
              sig = nullptr;
            }
            else if (itr->first == typeid(int16_t).name())
            {
              CanSignal<int16_t> *sig = (CanSignal<int16_t> *)itr1->second.get();
              sig->Parse(data, len, stamp);
              sig = nullptr;
            }
            else if (itr->first == typeid(int32_t).name())
            {
              CanSignal<int32_t> *sig = (CanSignal<int32_t> *)itr1->second.get();
              sig->Parse(data, len, stamp);
              sig = nullptr;
            }
            else if (itr->first == typeid(int64_t).name())
            {
              CanSignal<int64_t> *sig = (CanSignal<int64_t> *)itr1->second.get();
              sig->Parse(data, len, stamp);
              sig = nullptr;
            }
            else if (itr->first == typeid(float).name())
            {
              CanSignal<float> *sig = (CanSignal<float> *)itr1->second.get();
              sig->Parse(data, len, stamp);
              sig = nullptr;
            }
            else if (itr->first == typeid(double).name())
            {
              CanSignal<double> *sig = (CanSignal<double> *)itr1->second.get();
              sig->Parse(data, len, stamp);
              sig = nullptr;
            }
            else
            {
            }
          }
        }
      }
      void CanMessage::UpdateFrame(uint8_t *data, uint8_t len)
      {
        std::unique_lock<std::mutex> lck(mutex_);
        for (auto itr = signal_map_.begin();
             itr != signal_map_.end(); itr++)
        {
          for (auto itr1 = itr->second.begin();
               itr1 != itr->second.end(); itr1++)
          {
            if (itr->first == typeid(bool).name())
            {
              CanSignal<bool> *sig = (CanSignal<bool> *)itr1->second.get();
              sig->UpdateData(data, len);
              sig = nullptr;
            }
            else if (itr->first == typeid(uint8_t).name())
            {
              CanSignal<uint8_t> *sig = (CanSignal<uint8_t> *)itr1->second.get();
              sig->UpdateData(data, len);
              sig = nullptr;
            }
            else if (itr->first == typeid(uint16_t).name())
            {
              CanSignal<uint16_t> *sig = (CanSignal<uint16_t> *)itr1->second.get();
              sig->UpdateData(data, len);
              sig = nullptr;
            }
            else if (itr->first == typeid(uint32_t).name())
            {
              CanSignal<uint32_t> *sig = (CanSignal<uint32_t> *)itr1->second.get();
              sig->UpdateData(data, len);
              sig = nullptr;
            }
            else if (itr->first == typeid(uint64_t).name())
            {
              CanSignal<uint64_t> *sig = (CanSignal<uint64_t> *)itr1->second.get();
              sig->UpdateData(data, len);
              sig = nullptr;
            }
            else if (itr->first == typeid(int8_t).name())
            {
              CanSignal<int8_t> *sig = (CanSignal<int8_t> *)itr1->second.get();
              sig->UpdateData(data, len);
              sig = nullptr;
            }
            else if (itr->first == typeid(int16_t).name())
            {
              CanSignal<int16_t> *sig = (CanSignal<int16_t> *)itr1->second.get();
              sig->UpdateData(data, len);
              sig = nullptr;
            }
            else if (itr->first == typeid(int32_t).name())
            {
              CanSignal<int32_t> *sig = (CanSignal<int32_t> *)itr1->second.get();
              sig->UpdateData(data, len);
              sig = nullptr;
            }
            else if (itr->first == typeid(int64_t).name())
            {
              CanSignal<int64_t> *sig = (CanSignal<int64_t> *)itr1->second.get();
              sig->UpdateData(data, len);
              sig = nullptr;
            }
            else if (itr->first == typeid(float).name())
            {
              CanSignal<float> *sig = (CanSignal<float> *)itr1->second.get();
              sig->UpdateData(data, len);
              sig = nullptr;
            }
            else if (itr->first == typeid(double).name())
            {
              CanSignal<double> *sig = (CanSignal<double> *)itr1->second.get();
              sig->UpdateData(data, len);
              sig = nullptr;
            }
            else
            {
            }
          }
        }
        UpdateRollingCounter(data, len);
        UpdateChecksum(data, len);
      }
      uint8_t CanMessage::CalcCheckSum(const uint8_t *input, const int32_t len)
      {
        return static_cast<uint8_t>(std::accumulate(input, input + len, 0) ^ 0xFF);
      }

      void CanMessage::UpdateRollingCounter(uint8_t *data, uint8_t len)
      {
        if (len <= 2)
          return;
        roll_cnt_.exchange(roll_cnt_.load() + 1);
        data[len - 2] |= static_cast<uint8_t>(roll_cnt_.load() & 0x0f);
      }

      void CanMessage::UpdateChecksum(uint8_t *data, uint8_t len)
      {
        if (len <= 2)
          return;
        uint16_t sum = 0;
        for (int i = 0; i < len - 1; i++)
          sum += data[i];
        data[len - 1] = sum ^ 0xFF;
      }

      void CanMessage::Clear()
      {
        signal_map_.clear();
      }

      CanMessage &CanMessage::operator=(const CanMessage &msg)
      {
        std::unique_lock<std::mutex> lck(mutex_);
        this->signal_map_ = msg.signal_map_;
        return *this;
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana