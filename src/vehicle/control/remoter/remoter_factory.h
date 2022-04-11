#pragma once

#include "Iremoter.h"
namespace nirvana
{
  namespace vehicle
  {
    namespace control
    {
      namespace remoter
      {
        class RemoterFactory
        {
        public:
          IRemoter *CreateWflyEt06();
        };
      }
    } // namespace control
  }
} // namespace nirvana