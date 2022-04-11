#pragma once

#include <string>
#include <memory>

// #include "../road_element/refpoint/Irefpoint.h"
// #include "../road_element/line/ILine.h"
// #include "../road_element/lane/ILane.h"
// #include "../road_element/Iroad.h"
#include "../common/map_types.h"
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

namespace nirvana
{
  namespace map
  {
    class IMap
    {
    public:
      //       IMap(/* args */) = default;
      //       virtual ~IMap() = default;
      virtual bool Init(std::string name) = 0;
      virtual std::string Name() = 0;
      virtual void LoadMapFromXmlFile(std::string path) = 0;
      virtual void WriteMap2XmlFile(std::string path) = 0;
      virtual void LoadMapFromString(const std::string &map_str) = 0;
      virtual void WriteMap2String(std::string &map_str) = 0;
      virtual bool CollectGlobalPath(const geometry_msgs::PoseStamped &lc_pt) = 0;
      virtual bool Empty() const = 0;
      virtual nav_msgs::Path GetGlobalPath() const = 0;
      virtual common::geometry::Point<double> GetBasePoint() const = 0;
      virtual double GetRangeXMin() const = 0;
      virtual double GetRangeXMax() const = 0;
      virtual double GetRangeYMin() const = 0;
      virtual double GetRangeYMax() const = 0;
      virtual double GetRangeZMin() const = 0;
      virtual double GetRangeZMax() const = 0;
      virtual void Clear() = 0;
      // virtual std::shared_ptr<LaneLineRefPoint> InsertLaneLineRefPoint(const std::shared_ptr<LaneLineRefPoint> &p) = 0;
      // virtual void DeleteLaneLineRefPoint(uint16_t id) = 0;
      // virtual void AddLaneRefPoint(const LaneRefPoint &p) = 0;
      // virtual void AddLaneRefPoint(const LaneLineRefPoint &p1,
      //                              const LaneLineRefPoint &p2) = 0;
      // virtual void DeleteLaneRefPoint(uint16_t id) = 0;
      // virtual std::shared_ptr<road::ILine> GenerateLaneLine(const LaneLineRefPoint &src,
      //                                                       const LaneLineRefPoint &dst) = 0;

      // virtual std::shared_ptr<road::LaneCenter> GenerateLaneCenter(const LaneRefPoint &src,
      //                                                              const std::shared_ptr<road::LaneLine> &left,
      //                                                              const std::shared_ptr<road::LaneLine> &right) = 0;
      // virtual std::shared_ptr<road::LaneCenter> LinkLaneRefPoint(const LaneRefPoint &src,
      //                                                            const LaneRefPoint &dst) = 0;
    };

  } // namespace map
} // namespace nirvana
