#include "canbus_protocol_factory.h"

namespace nirvana
{
  namespace message
  {
    namespace canbus
    {
      CanbusProtocol *CanbusProtocolFactory::CreateCanbusProtocol(uint32_t id)
      {
        switch (id)
        {
        case 0x211:
          return new BrakeStat211();
        case 0x212:
          return new BrakeDiag212();
        case 0x213:
          return new ParkStat213();
        case 0x214:
          return new ParkDiag214();
        case 0x215:
          return new SteerStat215();
        case 0x216:
          return new SteerDiag216();
        case 0x217:
          return new ThrotStat217();
        case 0x218:
          return new ThrotDiag218();
        case 0x219:
          return new BodyStat219();
        case 0x220:
          return new BodyDiag220();
        case 0x221:
          return new AnticrshStat221();
        case 0x222:
          return new BatStat222();
        case 0x223:
          return new BatDiag223();
        case 0x240:
          return new VehStat240();
        case 0x250:
          return new VehDynaStat250();
        case 0x251:
          return new FrontWheelSpd251();
        case 0x252:
          return new RearWheelSpd252();
        case 0x254:
          return new TotalTripStat254();
        case 0x255:
          return new RemoteTripStat255();
        case 0x256:
          return new AutomaticTripStat256();
        case 0x257:
          return new CloudTripStat257();
        case 0x260:
          return new PowerStat260();
        case 0x270:
          return new RemoterStat270();
        case 0x271:
          return new RemoterStat271();
        case 0x272:
          return new RemoterStat272();
        case 0x2a0:
          return new VehInfo2a0();
        case 0x111:
          return new BrakeCmd111();
        case 0x112:
          return new ParkCmd112();
        case 0x113:
          return new SteerCmd113();
        case 0x114:
          return new ThrotCmd114();
        case 0x115:
          return new BodyCmd115();
        case 0x117:
          return new PowerCmd117();
        case 0x118:
          return new CloudDriveCmd118();
        case 0x119:
          return new CloudBodyCmd119();
        //imu m2
        case 0x701:
          return new WeekTimeStat701();
        case 0x702:
          return new UtcTimeStat702();
        case 0x703:
          return new RtkPos1Stat703();
        case 0x704:
          return new RtkPos2Stat704();
        case 0x705:
          return new RtkSpeedStat705();
        case 0x706:
          return new RtkAttitudeStat706();
        case 0x707:
          return new RtkPosCovStat707();
        case 0x708:
          return new RtkSpeedCovStat708();
        case 0x709:
          return new RtkAttitudeCovStat709();
        case 0x710:
          return new RtkStat710();
        case 0x721:
          return new IMUGyro1Stat721();
        case 0x722:
          return new IMUGyro2Stat722();
        case 0x723:
          return new IMUGyro3Stat723();
        case 0x099:
          return new IMUInputCmd099();
        //imu ins570d
        case 0x500:
          return new IMUAccStat500();
        case 0x501:
          return new IMUGyroStat501();
        case 0x502:
          return new IMUAttStat502();
        case 0x503:
          return new IMUPos1Stat503();
        case 0x504:
          return new IMUPos2Stat504();
        case 0x505:
          return new IMUSpeedStat505();
        case 0x506:
          return new IMUDataInfoStat506();
        case 0x507:
          return new IMUStdCov507();
        default:
          return nullptr;
        }
      }
    } // namespace canbus
  }   // namespace message
} // namespace nirvana