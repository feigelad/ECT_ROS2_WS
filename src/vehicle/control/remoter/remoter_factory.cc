#include "remoter_factory.h"
#include "wfly_et06/wfly_et06.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace control
    {
      namespace remoter
      {
        IRemoter *RemoterFactory::CreateWflyEt06()
        {
          return new WflyEt06();
        }
      }
    } // namespace control
  }
} // namespace nirvana