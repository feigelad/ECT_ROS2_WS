#include "dynamic_factory.h"
#include "monorail_model/monorail_model.h"
namespace nirvana
{
  namespace vehicle
  {
    namespace dynamic
    {
      IDynamic *DynamicFactory::CreateMonorailModel()
      {
        return new dynamic::MonorailModel();
      }
    } // namespace dynamic
  }   // namespace control
} // namespace nirvana