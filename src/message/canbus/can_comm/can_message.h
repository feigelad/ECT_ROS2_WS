#pragma once

#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <typeinfo>
#include <memory>
#include <atomic>

#include "can_signal.h"

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {

      class CanMessage
      {
      public:
        CanMessage() {}
        virtual ~CanMessage() { Clear(); }

        virtual std::string Name() {}
        virtual uint32_t ID() {}
        virtual uint8_t IdExtend() {}
        virtual uint32_t Period() {}

        template <typename SignalType>
        bool AddSignal(const CanSignal<SignalType> &sig)
        {
          std::string tp = typeid(SignalType).name();
          std::string name = sig.GetName();
          std::unique_lock<std::mutex>
              lck(mutex_);
          auto itr = signal_map_.find(tp);
          if (itr != signal_map_.end())
          {
            auto itr1 = signal_map_[tp].find(name);
            if (itr1 == signal_map_[tp].end())
            {
              signal_map_[tp][name] = std::shared_ptr<CanSignal<SignalType>>(new CanSignal<SignalType>(sig));
              return true;
            }
            else
            {
              return false; //Signal already exists
            }
          }
          else
          {
            signal_map_[tp][name] = std::shared_ptr<CanSignal<SignalType>>(new CanSignal<SignalType>(sig));
            return true;
          }
        }

        template <typename SignalType>
        void DeleteSignal(std::string name)
        {
          std::string tp = typeid(SignalType).name();
          std::unique_lock<std::mutex> lck(mutex_);
          auto itr = signal_map_.find(tp);
          if (itr != signal_map_.end())
          {
            auto itr1 = signal_map_[tp].find(name);
            if (itr1 != signal_map_[tp].end())
            {
              signal_map_[tp].erase(name);
              // if (tp == typeid(bool).name()) {
              //   // bool *tmp = static_cast<bool *>(signal_map_[tp][name]);
              //   signal_map_[tp].erase(name);
              //   // delete tmp;
              //   // tmp = nullptr;
              // }
              // else if (tp == typeid(uint8_t).name()) {
              //   uint8_t *tmp = static_cast<uint8_t *>(signal_map_[tp][name]);
              //   signal_map_[tp].erase(name);
              //   delete tmp;
              //   tmp = nullptr;
              // }
              // else if (tp == typeid(uint16_t).name()) {
              //   uint16_t *tmp = static_cast<uint16_t *>(signal_map_[tp][name]);
              //   signal_map_[tp].erase(name);
              //   delete tmp;
              //   tmp = nullptr;
              // }
              // else if (tp == typeid(uint32_t).name()) {
              //   uint32_t *tmp = static_cast<uint32_t *>(signal_map_[tp][name]);
              //   signal_map_[tp].erase(name);
              //   delete tmp;
              //   tmp = nullptr;
              // }
              // else if (tp == typeid(int8_t).name()) {
              //   int8_t *tmp = static_cast<int8_t *>(signal_map_[tp][name]);
              //   signal_map_[tp].erase(name);
              //   delete tmp;
              //   tmp = nullptr;
              // }
              // else if (tp == typeid(int16_t).name()) {
              //   int16_t *tmp = static_cast<int16_t *>(signal_map_[tp][name]);
              //   signal_map_[tp].erase(name);
              //   delete tmp;
              //   tmp = nullptr;
              // }
              // else if (tp == typeid(int32_t).name()) {
              //   int32_t *tmp = static_cast<int32_t *>(signal_map_[tp][name]);
              //   signal_map_[tp].erase(name);
              //   delete tmp;
              //   tmp = nullptr;
              // }
              // else if (tp == typeid(float).name()) {
              //   float *tmp = static_cast<float *>(signal_map_[tp][name]);
              //   signal_map_[tp].erase(name);
              //   delete tmp;
              //   tmp = nullptr;
              // }
              // else if (tp == typeid(double).name()) {
              //   double *tmp = static_cast<double *>(signal_map_[tp][name]);
              //   signal_map_[tp].erase(name);
              //   delete tmp;
              //   tmp = nullptr;
              // }
              // else{}
            }
          }
        }

        template <typename SignalType>
        std::shared_ptr<CanSignal<SignalType>> GetSignal(std::string name)
        {
          std::string tp = typeid(SignalType).name();
          std::unique_lock<std::mutex> lck(mutex_);
          if ((signal_map_.find(tp) != signal_map_.end()) && (signal_map_[tp].find(name) != signal_map_[tp].end()))
          {
            return signal_map_[tp][name];
          }
          return nullptr;
        }

        size_t SignalCount() const;

        virtual uint8_t CalcCheckSum(const uint8_t *input, const int32_t len);
        void Clear();

      public:
        CanMessage &operator=(const CanMessage &msg);

      public:
        virtual void Parse(const uint8_t *data, uint8_t len, double stamp);
        virtual void UpdateFrame(uint8_t *data, uint8_t len);

      private:
        void UpdateRollingCounter(uint8_t *data, uint8_t len);
        void UpdateChecksum(uint8_t *data, uint8_t len);

      private:
        mutable std::mutex mutex_;
        std::atomic<uint8_t> roll_cnt_;
        std::unordered_map<std::string, std::unordered_map<std::string, std::shared_ptr<void>>> signal_map_;
      };

    } // namespace canbus
  }   // namespace message
} // namespace nirvana
