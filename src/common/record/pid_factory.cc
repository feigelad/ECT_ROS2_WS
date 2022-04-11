
#include "pid_factory.h"
#include "pos_pid/pos_pid.h"
namespace nirvana
{
  namespace common
  {
    namespace algorithm
    {
      IPid *PidFactory::CreatePosPid()
      {
        return (new PosPid());
      }
    } // namespace log
  }   // namespace common
} // namespace nirvana