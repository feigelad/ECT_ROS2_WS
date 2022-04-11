#include "motion_factory.h"
#include "ecar_motion/ecar_motion.h"
namespace nirvana
{
  namespace vehicle
  {
    namespace status
    {
      IMotionStat *MotionFactory::CreateEcarMotion()
      {
        return new status::EcarMotion();
      }
    } // namespace dynamic
  }   // namespace control
} // namespace nirvana