
#include "log_factory.h"
namespace nirvana
{
  namespace common
  {
    namespace log
    {
      ILog *LogFactory::CreateRosLogger()
      {
        return (new RosLog());
      }
    } // namespace log
  }   // namespace common
} // namespace nirvana