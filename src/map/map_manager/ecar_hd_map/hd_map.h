#pragma once

#include <map>
#include <vector>
#include <mutex>
#include "../Imap.h"
#include <Poco/AutoPtr.h>
#include <Poco/DOM/Document.h>
#include <Poco/DOM/DOMParser.h>
#include <Poco/DOM/DOMWriter.h>
#include <Poco/XML/XMLWriter.h>
#include <Poco/DOM/NodeIterator.h>
#include <Poco/DOM/NodeFilter.h>
#include <Poco/DOM/Node.h>
#include <Poco/DOM/NamedNodeMap.h>
#include <Poco/DOM/Element.h>
#include <Poco/DOM/Comment.h>
#include <Poco/DOM/ProcessingInstruction.h>
// #include "common/graph/graph.h"
// #include "common/geometry/curv/curv.h"
// #include "boost/graph/property_maps/container_property_map.hpp"
// #include "boost/geometry/geometries/pointing_segment.hpp"
namespace nirvana
{
  namespace map
  {
    class HDMap : public IMap
    {
      // typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, road::LaneRefPoint, road::LaneCenter> Graph;

    public:
      HDMap(/* args */)
          : map_rangex_min_(-1.0), map_rangex_max_(1.0), map_rangey_min_(-1.0), map_rangey_max_(1.0), map_rangez_min_(-1.0), map_rangez_max_(1.0)
      {
        map_range_inited_.exchange(false);
        map_base_confirmed_.exchange(false);
      }
      virtual ~HDMap() {}
      virtual bool Init(std::string name);
      virtual std::string Name();
      virtual void LoadMapFromXmlFile(std::string path);
      virtual void WriteMap2XmlFile(std::string path);
      virtual void LoadMapFromString(const std::string &map_str);
      virtual void WriteMap2String(std::string &map_str);
      virtual bool CollectGlobalPath(const geometry_msgs::PoseStamped &lc_pt);
      virtual bool Empty() const
      {
        std::unique_lock<std::mutex> lck(mutex_);
        //TODO: need to add other map elements, lifei, 2021/3/3
        return global_path_.poses.empty() /*|| */ || false;
      }
      virtual nav_msgs::Path GetGlobalPath() const;
      virtual common::geometry::Point<double> GetBasePoint() const
      {
        std::unique_lock<std::mutex> lck(mutex_);
        return map_base_;
      }
      virtual double GetRangeXMin() const
      {
        return map_rangex_min_.load();
      }
      virtual double GetRangeXMax() const
      {
        return map_rangex_max_.load();
      }
      virtual double GetRangeYMin() const
      {
        return map_rangey_min_.load();
      }
      virtual double GetRangeYMax() const
      {
        return map_rangey_max_.load();
      }
      virtual double GetRangeZMin() const
      {
        return map_rangez_min_.load();
      }
      virtual double GetRangeZMax() const
      {
        return map_rangez_max_.load();
      }
      virtual void Clear();
      // virtual void AddLaneLineRefPoint(road::LaneLineRefPoint p);
      // virtual void DeleteLaneLineRefPoint(uint16_t id);
      // virtual void AddLaneRefPoint(road::LaneRefPoint p);
      // virtual void AddLaneRefPoint(const road::LaneLineRefPoint &p1,
      //                              const road::LaneLineRefPoint &p2);
      // virtual void DeleteLaneRefPoint(uint16_t id);
      // virtual std::shared_ptr<road::LaneLine> GenerateLaneLine(const road::LaneLineRefPoint &src,
      //                                                          const road::LaneLineRefPoint &dst,
      //                                                          enum common::geometry::CurvType type);
      // virtual std::shared_ptr<road::LaneCenter> GenerateLaneCenter(const road::LaneRefPoint &src,
      //                                                              const std::shared_ptr<road::LaneLine> &left,
      //                                                              const std::shared_ptr<road::LaneLine> &right,
      //                                                              enum common::geometry::CurvType type);
      // virtual std::shared_ptr<road::LaneCenter> LinkLaneRefPoint(const road::LaneRefPoint &src,
      //                                                            const road::LaneRefPoint &dst);
      // virtual IRoad

    private:
      void GenerateMap(Poco::AutoPtr<Poco::XML::Document> pMapDoc);
      void ParseMap(Poco::AutoPtr<Poco::XML::Document> pMapDoc);
      void MapRangeSet(const geometry_msgs::PoseStamped &pt);

    private:
      /* data */
      mutable std::mutex mutex_;
      std::string map_name_;
      std::atomic<bool> map_range_inited_;
      std::atomic<double> map_rangex_min_;
      std::atomic<double> map_rangex_max_;
      std::atomic<double> map_rangey_min_;
      std::atomic<double> map_rangey_max_;
      std::atomic<double> map_rangez_min_;
      std::atomic<double> map_rangez_max_;
      std::atomic<double> map_base_confirmed_;
      common::geometry::Point<double> map_base_;
      // std::map<int, std::shared_ptr<road::IRefPoint>> map_ref_points_;
      // std::map<int, std::shared_ptr<road::ILine>> map_lane_lines_;
      // std::map<int, std::shared_ptr<road::ILane>> map_lanes_;
      // std::map<int, std::shared_ptr<road::IRoad>> map_roads_;
      // common::graph::Graph<std::shared_ptr<road::IRefPoint>, road::LaneCenter> map_graph_;
      nav_msgs::Path global_path_;
    };

  } // namespace map
} // namespace nirvana
