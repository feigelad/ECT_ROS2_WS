#pragma once

#include <memory>

// #include "ILog.h"
#include "ecar_hd_map/hd_map.h"
namespace nirvana
{
  namespace map
  {
    class MapFactory
    {
    public:
      IMap *CreateEcarHDMap();
    };
  } // namespace map
} // namespace nirvana