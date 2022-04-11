#pragma once

#include <string>
#include <iostream>
#include <math.h>
#include <typeinfo>

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {

      typedef struct
      {
        uint64_t Byte0 : 8;
        uint64_t Byte1 : 8;
        uint64_t Byte2 : 8;
        uint64_t Byte3 : 8;
        uint64_t Byte4 : 8;
        uint64_t Byte5 : 8;
        uint64_t Byte6 : 8;
        uint64_t Byte7 : 8;
      } CanFrmBytes;

      typedef union {
        uint64_t frm_u64;
        CanFrmBytes frm_bytes;
      } FrmData;
      enum ByteOrder
      {
        MOTOROLA = 0,
        INTEL = 1
      };
      template <typename SignalType>
      class CanSignal
      {
      public:
        CanSignal() = delete;
        CanSignal(std::string name, ByteOrder order, uint8_t pos, uint8_t len,
                  double factor, double offset, std::string &sig_name, std::string &sig_unit, double *stamp,
                  SignalType *val, SignalType min_val = 0, SignalType max_val = 0,
                  std::string unit = std::string(""))
            : name_(name), byte_order_(order), bit_pos_(pos), bit_len_(len), factor_(factor),
              offset_(offset), max_val_(max_val), min_val_(min_val), unit_(unit), time_stamp_(stamp), value_(val)
        {
          sig_name = name;
          sig_unit = unit;
        }

        CanSignal(const CanSignal<SignalType> &sig)
        {
          this->name_ = sig.GetName();
          this->byte_order_ = sig.GetByteOrder();
          this->bit_pos_ = sig.GetBitPos();
          this->bit_len_ = sig.GetBitLength();
          this->factor_ = sig.GetFactor();
          this->offset_ = sig.GetOffset();
          this->time_stamp_ = sig.GetTimeStamp();
          this->value_ = sig.GetValue();
          this->max_val_ = sig.GetMax();
          this->min_val_ = sig.GetMin();
          this->unit_ = sig.GetUnit();
        }
        virtual ~CanSignal() { value_ = nullptr; }

        // CanSignal<SignalType>& operator=(const CanSignal<SignalType> &sig);
        std::string GetName() const;
        ByteOrder GetByteOrder() const;
        uint8_t GetBitPos() const;
        uint8_t GetBitLength() const;
        float GetFactor() const;
        float GetOffset() const;
        std::string GetDataTypeStr();
        SignalType GetMax() const;
        SignalType GetMin() const;
        std::string GetUnit() const;
        // void SetValue(const SignalType &val);
        double *GetTimeStamp() const;
        SignalType *GetValue() const;
        bool Parse(const uint8_t *data, uint8_t len, double stamp);
        bool UpdateData(uint8_t *data, uint8_t len);

      private:
        bool ParseUintData(uint64_t *const res, const uint8_t *data, uint8_t len,
                           uint8_t bit_pos, uint8_t bit_len, ByteOrder order);
        bool FillFrameData(uint8_t *data, uint8_t len, uint64_t unint_val,
                           uint8_t bit_pos, uint8_t bit_len, ByteOrder order);

      private:
        //  mutable std::mutex mutex_;
        std::string name_;
        ByteOrder byte_order_;
        uint8_t bit_pos_;
        uint8_t bit_len_;
        float factor_;
        float offset_;
        double *time_stamp_;
        SignalType *value_;
        SignalType max_val_;
        SignalType min_val_;
        std::string unit_;
      };

      // template <typename SignalType>
      // CanSignal<SignalType>& CanSignal<SignalType>::operator=(const CanSignal<SignalType> &sig) {
      //   // std::unique_lock<std::mutex> lck(mutex_);
      //   this->name_ = sig.GetName();
      //   this->byte_order_ = sig.GetByteOrder();
      //   this->bit_pos_ = sig.GetBitPos();
      //   this->bit_len_ = sig.GetBitLength();
      //   this->factor_ = sig.GetFactor();
      //   this->offset_ = sig.GetOffset();
      //   this->value_ = sig.GetValue();
      //   this->max_val_ = sig.GetMax();
      //   this->min_val_ = sig.GetMin();
      //   this->unit_ = sig.GetUnit();
      //   return *this;
      // }

      template <typename SignalType>
      std::string CanSignal<SignalType>::GetName() const
      {
        // std::unique_lock<std::mutex> lck(mutex_);
        return name_;
      }

      template <typename SignalType>
      ByteOrder CanSignal<SignalType>::GetByteOrder() const
      {
        // std::unique_lock<std::mutex> lck(mutex_);
        return byte_order_;
      }
      template <typename SignalType>
      uint8_t CanSignal<SignalType>::GetBitPos() const
      {
        // std::unique_lock<std::mutex> lck(mutex_);
        return bit_pos_;
      }

      template <typename SignalType>
      uint8_t CanSignal<SignalType>::GetBitLength() const
      {
        // std::unique_lock<std::mutex> lck(mutex_);
        return bit_len_;
      }

      template <typename SignalType>
      float CanSignal<SignalType>::GetFactor() const
      {
        // std::unique_lock<std::mutex> lck(mutex_);
        return factor_;
      }

      template <typename SignalType>
      float CanSignal<SignalType>::GetOffset() const
      {
        // std::unique_lock<std::mutex> lck(mutex_);
        return offset_;
      }

      template <typename SignalType>
      std::string CanSignal<SignalType>::GetDataTypeStr()
      {
        // std::unique_lock<std::mutex> lck(mutex_);
        return typeid(SignalType).name();
      }

      template <typename SignalType>
      SignalType CanSignal<SignalType>::GetMax() const
      {
        // std::unique_lock<std::mutex> lck(mutex_);
        return max_val_;
      }

      template <typename SignalType>
      SignalType CanSignal<SignalType>::GetMin() const
      {
        // std::unique_lock<std::mutex> lck(mutex_);
        return min_val_;
      }

      template <typename SignalType>
      std::string CanSignal<SignalType>::GetUnit() const
      {
        // std::unique_lock<std::mutex> lck(mutex_);
        return unit_;
      }
      // template <typename SignalType>
      // void CanSignal<SignalType>::SetValue(const SignalType &val) {
      //   // std::unique_lock<std::mutex> lck(mutex_);
      //   *value_ = val;
      // }

      template <typename SignalType>
      double *CanSignal<SignalType>::GetTimeStamp() const
      {
        // std::unique_lock<std::mutex> lck(mutex_);
        return time_stamp_;
      }

      template <typename SignalType>
      SignalType *CanSignal<SignalType>::GetValue() const
      {
        // std::unique_lock<std::mutex> lck(mutex_);
        return value_;
      }

      template <typename SignalType>
      bool CanSignal<SignalType>::Parse(const uint8_t *data, uint8_t len, double stamp)
      {
        if (value_ == nullptr || time_stamp_ == nullptr)
        {
          //Invalid signal
          return false;
        }

        if (data == nullptr)
        {
          // AERROR << "Invalid data pointer! [" << GetName() << "]";
          return false;
        }

        if ((len == 0) || (len > 8))
        {
          // AERROR << "Invalid data length! [" << GetName() << "]";
          return false;
        }

        // uint8_t bit_pos = GetBitPos();
        // uint8_t bit_len = GetBitLength();
        // ByteOrder order = GetByteOrder();
        uint64_t uint_val;
        if (false == ParseUintData(&uint_val, data, len, bit_pos_, bit_len_, byte_order_))
        {
          // AERROR << "Convert bytes to uint value failed. [" << GetName() << "]";
          return false;
        }
        *value_ = std::max<SignalType>(min_val_, std::min<SignalType>(max_val_, static_cast<SignalType>(static_cast<SignalType>(uint_val) * static_cast<SignalType>(factor_) + static_cast<SignalType>(offset_))));
        *time_stamp_ = stamp;
        // SetValue(value);
        return true;
      }

      template <typename SignalType>
      bool CanSignal<SignalType>::UpdateData(uint8_t *data, uint8_t len)
      {
        if (value_ == nullptr || time_stamp_ == nullptr)
        {
          return false;
        }

        uint64_t uint_val = static_cast<uint64_t>((*value_ - offset_) / factor_);
        if (false == FillFrameData(data, len, uint_val, bit_pos_, bit_len_, byte_order_))
        {
          // AERROR << "Convert raw data to bytes failed. [" << GetName() << "]";
          return false;
        }

        return true;
      }

      //Private Functions
      template <typename SignalType>
      bool CanSignal<SignalType>::ParseUintData(uint64_t *const res,
                                                const uint8_t *data, uint8_t len,
                                                uint8_t bit_pos, uint8_t bit_len,
                                                ByteOrder order)
      {
        uint64_t index, mask;
        FrmData frame;

        if (order == MOTOROLA)
        {
          index = 56 + (bit_pos % 8) - (bit_pos / 8 * 8);
          mask = (static_cast<uint64_t>(0x1) << bit_len) - 1;

          frame.frm_bytes.Byte0 = data[7];
          frame.frm_bytes.Byte1 = data[6];
          frame.frm_bytes.Byte2 = data[5];
          frame.frm_bytes.Byte3 = data[4];
          frame.frm_bytes.Byte4 = data[3];
          frame.frm_bytes.Byte5 = data[2];
          frame.frm_bytes.Byte6 = data[1];
          frame.frm_bytes.Byte7 = data[0];

          *res = (frame.frm_u64 >> index) & mask;
          return true;
        }
        else if (order == INTEL)
        {
          index = bit_pos;
          mask = (static_cast<uint64_t>(0x1) << bit_len) - 1;

          frame.frm_bytes.Byte0 = data[0];
          frame.frm_bytes.Byte1 = data[1];
          frame.frm_bytes.Byte2 = data[2];
          frame.frm_bytes.Byte3 = data[3];
          frame.frm_bytes.Byte4 = data[4];
          frame.frm_bytes.Byte5 = data[5];
          frame.frm_bytes.Byte6 = data[6];
          frame.frm_bytes.Byte7 = data[7];

          *res = (frame.frm_u64 >> index) & mask;
          return true;
        }
        else
        {
          // AERROR << "Invalid byte order! [" << GetName() << "]";
          return false;
        }
      }
      template <typename SignalType>
      bool CanSignal<SignalType>::FillFrameData(uint8_t *data, uint8_t len,
                                                uint64_t unint_val, uint8_t bit_pos,
                                                uint8_t bit_len, ByteOrder order)
      {
        uint64_t index, mask;
        FrmData frame;
        if (order == MOTOROLA)
        {
          index = 56 + (bit_pos % 8) - (bit_pos / 8 * 8);
          mask = (static_cast<uint64_t>(0x1) << bit_len) - 1;
          frame.frm_u64 = unint_val & mask;
          frame.frm_u64 <<= index;
          data[7] |= frame.frm_bytes.Byte0;
          data[6] |= frame.frm_bytes.Byte1;
          data[5] |= frame.frm_bytes.Byte2;
          data[4] |= frame.frm_bytes.Byte3;
          data[3] |= frame.frm_bytes.Byte4;
          data[2] |= frame.frm_bytes.Byte5;
          data[1] |= frame.frm_bytes.Byte6;
          data[0] |= frame.frm_bytes.Byte7;
          return true;
        }
        else if (order == INTEL)
        {
          index = bit_pos;
          mask = (static_cast<uint64_t>(0x1) << bit_len) - 1;
          frame.frm_u64 = unint_val & mask;
          frame.frm_u64 <<= index;
          data[0] |= frame.frm_bytes.Byte0;
          data[1] |= frame.frm_bytes.Byte1;
          data[2] |= frame.frm_bytes.Byte2;
          data[3] |= frame.frm_bytes.Byte3;
          data[4] |= frame.frm_bytes.Byte4;
          data[5] |= frame.frm_bytes.Byte5;
          data[6] |= frame.frm_bytes.Byte6;
          data[7] |= frame.frm_bytes.Byte7;
          return true;
        }
        else
        {
          // AERROR << "Invalid byte order! [" << GetName() << "]";
          return false;
        }
      }

    } // namespace canbus
  }   // namespace message
} // namespace nirvana