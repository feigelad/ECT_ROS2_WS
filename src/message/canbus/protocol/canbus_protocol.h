/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 * @brief The class of Canbus Protocol
 */

#pragma once

#include <atomic>
#include <cmath>
#include <numeric>
// #include <any.h>
// #include "common/any.h"

#include "../can_comm/can_message.h"
#include "../can_comm/canbus_error.h"

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {

      enum MessageDirect
      {
        MSG_SEND = 0,
        MSG_RECV
      };

      /**
       * @class ProtocolData
       *
       * @brief This is the base class of protocol data.
       */

      // template <typename MsgT>
      class CanbusProtocol : public CanMessage
      {
      public:
        /**
   * @brief construct protocol data.
   */
        // CanbusProtocol() = default;
        CanbusProtocol(uint32_t id, std::string name, uint8_t ide, uint32_t prd, MessageDirect direct)
            : name_(name), id_(id), ide_(ide), std_period_(prd), direct_(direct) {}

        virtual canbus_error_t Init(void *const msg) {}

        virtual std::string Name() const
        {
          return name_;
        }
        virtual uint32_t ID() const
        {
          return id_;
        }
        virtual uint8_t IdExtend() const
        {
          return ide_;
        }
        virtual uint32_t Period() const
        {
          return std_period_;
        }
        virtual uint32_t RealPeriod() const
        {
          return real_period_;
        }
        virtual void SetStamp(double stamp)
        {
          real_period_ = stamp - last_frm_stamp_;
          last_frm_stamp_ = stamp;
        }
        virtual double LastFrmStamp() const
        {
          return last_frm_stamp_;
        }
        virtual MessageDirect Direct() const
        {
          return direct_;
        }

        virtual void ParseMsg(const uint8_t *data, uint8_t len, double stamp) {}

        virtual void UpdateMsg(uint8_t *data, uint8_t len) {}

      private:
        std::string name_;
        uint32_t id_;
        uint8_t ide_;
        uint32_t std_period_;
        uint32_t real_period_;
        double last_frm_stamp_;
        MessageDirect direct_;
      };
    } // namespace canbus
  }   // namespace message
} // namespace nirvana