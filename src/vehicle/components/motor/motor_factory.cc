#include "motor_factory.h"
#include "DBStar_2p2kw/motor_dbstar_2p2kw.h"

namespace nirvana
{
  namespace vehicle
  {
    namespace components
    {
      namespace motor
      {
        IMotor *MotorFactory::CreateDBStar_2p2kw()
        {
          return new motor::MotorDBStar2p2kw();
        }
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana