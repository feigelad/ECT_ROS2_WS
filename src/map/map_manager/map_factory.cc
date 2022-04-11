
#include "map_factory.h"
namespace nirvana
{
  namespace map
  {
    IMap *MapFactory::CreateEcarHDMap()
    {
      return (new nirvana::map::HDMap());
    }
  } // namespace map
} // namespace nirvana
