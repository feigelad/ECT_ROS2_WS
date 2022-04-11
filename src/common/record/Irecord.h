#pragma once

#include <string>

namespace nirvana
{
  namespace common
  {
    namespace record
    {
      enum RecordType
      {
        Record_Write = 0,
        Record_Read
      };

      class IRecord
      {
      public:
        virtual bool Init() = 0;
        virtual void Open(enum RecordType type, std::string file) = 0;
        virtual void Write(double stamp, std::string topic, void *const msg) = 0;
        // virtual void Read()
      };
    }
  }
}