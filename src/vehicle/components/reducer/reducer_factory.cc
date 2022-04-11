#include "reducer_factory.h"
#include "reducer1/reducer1.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace reducer
      {
        IReducer *ReducerFactory::CreateReducer1()
        {
          return new reducer::Reducer1();
        }
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana