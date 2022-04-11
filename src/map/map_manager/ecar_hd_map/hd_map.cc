#include "hd_map.h"
#include <sys/stat.h>
#include <dirent.h>
#include <math.h>
#include <unistd.h>
#include <sstream>
#include <iostream>
#include <iomanip>

namespace nirvana
{
  namespace map
  {
    bool HDMap::Init(std::string name)
    {
      if (name == "")
        return false;
      map_name_ = name.substr(0, name.rfind("."));
      return true;
    }

    std::string HDMap::Name()
    {
      std::unique_lock<std::mutex> lck(mutex_);
      return map_name_;
    }

    void HDMap::LoadMapFromXmlFile(std::string path)
    {
      Poco::XML::DOMParser map_parser;
      Poco::AutoPtr<Poco::XML::Document> pMapDoc = map_parser.parse(path);
      ParseMap(pMapDoc);
    }

    void HDMap::LoadMapFromString(const std::string &map_str)
    {
      Poco::XML::DOMParser map_parser;
      Poco::AutoPtr<Poco::XML::Document> pMapDoc = map_parser.parseString(map_str);
      ParseMap(pMapDoc);
    }

    void HDMap::WriteMap2XmlFile(std::string path)
    {
      Poco::AutoPtr<Poco::XML::Document> pMapDoc = new Poco::XML::Document;
      GenerateMap(pMapDoc);

      Poco::XML::DOMWriter writer;
      writer.setOptions(Poco::XML::XMLWriter::PRETTY_PRINT); // PRETTY_PRINT = 4
      std::string full_path = path;

      struct stat buffer;
      if (stat(full_path.c_str(), &buffer) != 0)
      {
        std::vector<std::string> file_names;
        int pos = full_path.find_last_of("/");
        std::string str = full_path.substr(0, pos);
        while (access(str.c_str(), 0) == -1)
        {
          pos = str.find_last_of("/");
          file_names.push_back(str.substr(pos, str.length() - 1));
          str = str.substr(0, pos);
        }
        for (int f = file_names.size() - 1; f >= 0; f--)
        {
          str += file_names[f];
          mkdir(str.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        }
        FILE *file = fopen(full_path.c_str(), "w");
        fclose(file);
      }
      writer.writeNode(full_path, pMapDoc); //直接写进文件

      // //Roads
      // for (auto itr_road = map_roads_.begin(); itr_road != map_roads_.end(); itr_road++)
      // {
      //   Poco::AutoPtr<Poco::XML::Element> pMapChild = pMapDoc->createElement("map_road");
      //   pMapChild->setAttribute("id", std::to_string(itr_road->second->GetId()));
      //   std::map<int, std::shared_ptr<road::LaneLine>> lane_lines = itr_road->second->GetLaneLines();
      //   std::map<int, std::shared_ptr<road::Lane>> lanes = itr_road->second->GetLanes();
      //   //Lane lines
      //   for (auto itr_laneline = lane_lines.begin(); itr_laneline != lane_lines.end(); itr_laneline++)
      //   {
      //     Poco::AutoPtr<Poco::XML::Element> pMapGrandChild = pMapDoc->createElement("map_laneline");
      //     pMapGrandChild->setAttribute("id", std::to_string(itr_laneline->second->GetId()));
      //     pMapGrandChild->setAttribute("curv_type", std::to_string(itr_laneline->second->GetCurvType()));
      //     pMapGrandChild->setAttribute("line_type", std::to_string(itr_laneline->second->GetLineType()));
      //     std::vector<double> params = itr_laneline->second->GetLineParam();
      //     for (int i = 0; i < params.size(); i++)
      //     {
      //       pMapGrandChild->setAttribute("param_" + std::to_string(i), std::to_string(params[i]));
      //     }
      //     pMapChild->appendChild(pMapGrandChild);
      //   }

      // //Lanes
      // for (auto itr_lane = lanes.begin(); itr_lane != lanes.end(); itr_lane++)
      // {
      //   Poco::AutoPtr<Poco::XML::Element> pMapGrandChild1 = pMapDoc->createElement("map_Lane");
      //   pMapGrandChild1->setAttribute("id", std::to_string(itr_lane->second->GetId()));
      //   pMapGrandChild1->setAttribute("left_lane_id", std::to_string(itr_lane->second->GetLeftLaneId()));
      //   pMapGrandChild1->setAttribute("right_lane_id", std::to_string(itr_lane->second->GetRightLaneId()));
      //   pMapGrandChild1->setAttribute("left_lane_line_id", std::to_string(itr_lane->second->GetLeftLaneLineId()));
      //   pMapGrandChild1->setAttribute("right_lane_line_id", std::to_string(itr_lane->second->GetRightLaneLineId()));
      //   pMapGrandChild1->setAttribute("ref_point_id", std::to_string(itr_lane->second->GetRefPointId()));
      //   pMapGrandChild1->setAttribute("ref_point_opposite_id", std::to_string(itr_lane->second->GetRefPointOppositeId()));
      //   pMapChild->appendChild(pMapGrandChild1);
      // }
      //TODO:other road elements, lifei, 2020/08/31

      // pMapDoc->appendChild(pMapChild);
    }

    void HDMap::WriteMap2String(std::string &map_str)
    {
      Poco::AutoPtr<Poco::XML::Document> pMapDoc = new Poco::XML::Document;
      GenerateMap(pMapDoc);

      Poco::XML::DOMWriter writer;
      std::stringstream sstr;
      writer.writeNode(sstr, pMapDoc);
      map_str = sstr.str();
    }

    bool HDMap::CollectGlobalPath(const geometry_msgs::PoseStamped &lc_pt)
    {
      geometry_msgs::PoseStamped pt = lc_pt;
      geometry_msgs::PoseStamped pt_map;
      pt_map.header.frame_id = "map_coord_base";
      pt_map.header.stamp = pt.header.stamp;
      std::unique_lock<std::mutex> lck(mutex_);
      if (global_path_.poses.empty())
      {
        map_base_.SetX(pt.pose.position.x);
        map_base_.SetY(pt.pose.position.y);
        map_base_.SetZ(pt.pose.position.z);
        map_base_confirmed_.exchange(true);

        pt_map.pose.position.x = 0;
        pt_map.pose.position.y = 0;
        pt_map.pose.position.z = 0;
        pt_map.pose.orientation = pt.pose.orientation;

        MapRangeSet(pt_map);
        global_path_.poses.push_back(pt_map);
        std::cout << std::setprecision(15) << "MAP_COLLECT >> pos_x = " << pt_map.pose.position.x << ", pos_y = " << pt_map.pose.position.y << ", pos_z = " << pt_map.pose.position.z << std::endl;
      }
      else
      {
        pt_map.pose.position.x = pt.pose.position.x - map_base_.GetX();
        pt_map.pose.position.y = pt.pose.position.y - map_base_.GetY();
        pt_map.pose.position.z = pt.pose.position.z - map_base_.GetZ();
        pt_map.pose.orientation = pt.pose.orientation;
        double dst = sqrt(pow(pt_map.pose.position.x - global_path_.poses.back().pose.position.x, 2.0) + pow(pt_map.pose.position.y - global_path_.poses.back().pose.position.y, 2.0) + pow(pt_map.pose.position.z - global_path_.poses.back().pose.position.z, 2.0));
        if (dst > 0.2)
        {          
          MapRangeSet(pt_map);
          global_path_.poses.push_back(pt_map);
          // TODO: delete later
          std::cout << std::setprecision(15) << "MAP_COLLECT >> pos_x = " << pt_map.pose.position.x << ", pos_y = " << pt_map.pose.position.y << ", pos_z = " << pt_map.pose.position.z << std::endl;
        }
      }
      global_path_.header.frame_id = "map_coord_base";
      return true;
    }

    nav_msgs::Path HDMap::GetGlobalPath() const
    {
      std::unique_lock<std::mutex> lck(mutex_);
      return global_path_;
    }

    void HDMap::Clear()
    {
      map_range_inited_.exchange(false);
      map_base_confirmed_.exchange(false);
      std::unique_lock<std::mutex> lck(mutex_);
      global_path_.poses.clear();
    }

    void HDMap::GenerateMap(Poco::AutoPtr<Poco::XML::Document> pMapDoc)
    {
      nav_msgs::Path gpath;
      {
        std::unique_lock<std::mutex> lck(mutex_);
        gpath = global_path_;
      }

      // Poco::AutoPtr<Poco::XML::Document> pMapDoc = new Poco::XML::Document;
      Poco::AutoPtr<Poco::XML::ProcessingInstruction> pMapProcIns = pMapDoc->createProcessingInstruction("xml", "version='1.0' encoding='UTF-8'");
      Poco::AutoPtr<Poco::XML::Comment> pMapComment = pMapDoc->createComment("The HDMap for SelfDriving Vehicles.");
      Poco::AutoPtr<Poco::XML::Element> pMapRoot = pMapDoc->createElement("map_Root");

      //TODO:...
      // common::graph::Graph<road::LaneRefPoint, road::LaneCenter> graph;
      // std::map<int, std::shared_ptr<road::IRoad>> roads;
      // {
      //   std::unique_lock<std::mutex> lck(mutex_);
      // }

      //Map Name
      Poco::AutoPtr<Poco::XML::Element> pMapNameChild = pMapDoc->createElement("map_info");
      pMapNameChild->setAttribute("map_name", map_name_);
      pMapRoot->appendChild(pMapNameChild);

      //Map Base
      Poco::AutoPtr<Poco::XML::Element> pMapBaseChild = pMapDoc->createElement("map_base");
      pMapBaseChild->setAttribute("map_base_confirmed", std::to_string(map_base_confirmed_.load()));
      pMapBaseChild->setAttribute("map_base_x", std::to_string(map_base_.GetX()));
      pMapBaseChild->setAttribute("map_base_y", std::to_string(map_base_.GetY()));
      pMapBaseChild->setAttribute("map_base_z", std::to_string(map_base_.GetZ()));
      pMapRoot->appendChild(pMapBaseChild);

      //Map Range
      Poco::AutoPtr<Poco::XML::Element> pMapRangeChild = pMapDoc->createElement("map_range");
      pMapRangeChild->setAttribute("map_rang_inited", std::to_string(map_range_inited_.load()));
      pMapRangeChild->setAttribute("map_rangex_min", std::to_string(map_rangex_min_.load()));
      pMapRangeChild->setAttribute("map_rangex_max", std::to_string(map_rangex_max_.load()));
      pMapRangeChild->setAttribute("map_rangey_min", std::to_string(map_rangey_min_.load()));
      pMapRangeChild->setAttribute("map_rangey_max", std::to_string(map_rangey_max_.load()));
      pMapRangeChild->setAttribute("map_rangez_min", std::to_string(map_rangez_min_.load()));
      pMapRangeChild->setAttribute("map_rangez_max", std::to_string(map_rangez_max_.load()));
      pMapRoot->appendChild(pMapRangeChild);

      //Map global path
      Poco::AutoPtr<Poco::XML::Element> pMapGPathChild = pMapDoc->createElement("map_global_path");
      pMapGPathChild->setAttribute("id", std::to_string(0));
      pMapGPathChild->setAttribute("map_frame_id", global_path_.header.frame_id);
      for (int ind_gpath = 0; ind_gpath < gpath.poses.size(); ind_gpath++)
      {
        Poco::AutoPtr<Poco::XML::Element> pMapGrandChild = pMapDoc->createElement("map_global_path_point");
        pMapGrandChild->setAttribute("pos_x", std::to_string(gpath.poses[ind_gpath].pose.position.x));
        pMapGrandChild->setAttribute("pos_y", std::to_string(gpath.poses[ind_gpath].pose.position.y));
        pMapGrandChild->setAttribute("pos_z", std::to_string(gpath.poses[ind_gpath].pose.position.z));
        pMapGrandChild->setAttribute("quat_x", std::to_string(gpath.poses[ind_gpath].pose.orientation.x));
        pMapGrandChild->setAttribute("quat_y", std::to_string(gpath.poses[ind_gpath].pose.orientation.y));
        pMapGrandChild->setAttribute("quat_z", std::to_string(gpath.poses[ind_gpath].pose.orientation.z));
        pMapGrandChild->setAttribute("quat_w", std::to_string(gpath.poses[ind_gpath].pose.orientation.w));
        pMapGPathChild->appendChild(pMapGrandChild);
      }
      pMapRoot->appendChild(pMapGPathChild);
      pMapDoc->appendChild(pMapProcIns);
      pMapDoc->appendChild(pMapComment);
      pMapDoc->appendChild(pMapRoot);
    }

    void HDMap::ParseMap(Poco::AutoPtr<Poco::XML::Document> pMapDoc)
    {
      Poco::XML::NodeIterator map_it(pMapDoc, Poco::XML::NodeFilter::SHOW_ALL);
      Poco::XML::Node *pMapNode = map_it.nextNode();

      {
        std::unique_lock<std::mutex> lck(mutex_);
        global_path_.poses.clear();
      }

      std::string map_name = "";
      bool confirmed = false;
      bool inited = false;
      double base_x, base_y, base_z, rangex_min, rangex_max, rangey_min, rangey_max, rangez_min, rangez_max;
      while (pMapNode)
      {
        //Read map data
        std::string node_name = pMapNode->nodeName();
        if (node_name == "map_info")
        {
          Poco::XML::NamedNodeMap *map = pMapNode->attributes();
          if (map)
          {
            for (int i = 0; i < map->length(); i++)
            {
              Poco::XML::Node *attr = map->item(i);
              std::string sAttrName = attr->nodeName();
              if (sAttrName == "map_name")
              {
                map_name = attr->nodeValue().c_str();
              }
              else
              {
              }
            }
          }
        }
        else if (node_name == "map_base")
        {
          Poco::XML::NamedNodeMap *map = pMapNode->attributes();
          if (map)
          {
            for (int i = 0; i < map->length(); i++)
            {
              Poco::XML::Node *attr = map->item(i);
              std::string sAttrName = attr->nodeName();
              if (sAttrName == "map_base_confirmed")
              {
                int confirmed_int = atoi(attr->nodeValue().c_str());
                confirmed = confirmed_int != 0 ? true : false;
              }
              else if (sAttrName == "map_base_x")
                base_x = atof(attr->nodeValue().c_str());
              else if (sAttrName == "map_base_y")
                base_y = atof(attr->nodeValue().c_str());
              else if (sAttrName == "map_base_z")
                base_z = atof(attr->nodeValue().c_str());
              else
              {
              }
            }
          }
        }
        else if (node_name == "map_range")
        {
          Poco::XML::NamedNodeMap *map = pMapNode->attributes();
          if (map)
          {
            for (int i = 0; i < map->length(); i++)
            {
              Poco::XML::Node *attr = map->item(i);
              std::string sAttrName = attr->nodeName();
              if (sAttrName == "map_rang_inited")
              {
                int inited_int = atoi(attr->nodeValue().c_str());
                inited = inited_int != 0 ? true : false;
              }
              else if (sAttrName == "map_rangex_min")
                rangex_min = atof(attr->nodeValue().c_str());
              else if (sAttrName == "map_rangex_max")
                rangex_max = atof(attr->nodeValue().c_str());
              else if (sAttrName == "map_rangey_min")
                rangey_min = atof(attr->nodeValue().c_str());
              else if (sAttrName == "map_rangey_max")
                rangey_max = atof(attr->nodeValue().c_str());
              else if (sAttrName == "map_rangez_min")
                rangez_min = atof(attr->nodeValue().c_str());
              else if (sAttrName == "map_rangez_max")
                rangez_max = atof(attr->nodeValue().c_str());
              else
              {
              }
            }
          }
        }
        else if (node_name == "map_global_path")
        {
          Poco::XML::NamedNodeMap *map = pMapNode->attributes();
          if(map)
          {
            for (int i = 0; i < map->length(); i++)
            {
              Poco::XML::Node *attr = map->item(i);
              std::string sAttrName = attr->nodeName();
              if(sAttrName == "map_frame_id")
              {
                std::unique_lock<std::mutex> lck(mutex_);
                global_path_.header.frame_id = attr->nodeValue().c_str();
              }
            }
          }
        }
        else if (node_name == "map_global_path_point")
        {
          geometry_msgs::PoseStamped pt;
          Poco::XML::NamedNodeMap *map = pMapNode->attributes();
          if (map)
          {
            for (int i = 0; i < map->length(); i++)
            {
              Poco::XML::Node *attr = map->item(i);
              std::string sAttrName = attr->nodeName();
              if (sAttrName == "pos_x")
                pt.pose.position.x = atof(attr->nodeValue().c_str());
              else if (sAttrName == "pos_y")
                pt.pose.position.y = atof(attr->nodeValue().c_str());
              else if (sAttrName == "pos_z")
                pt.pose.position.z = atof(attr->nodeValue().c_str());
              else if (sAttrName == "quat_x")
                pt.pose.orientation.x = atof(attr->nodeValue().c_str());
              else if (sAttrName == "quat_y")
                pt.pose.orientation.y = atof(attr->nodeValue().c_str());
              else if (sAttrName == "quat_z")
                pt.pose.orientation.z = atof(attr->nodeValue().c_str());
              else if (sAttrName == "quat_w")
                pt.pose.orientation.w = atof(attr->nodeValue().c_str());
              else
              {
              }
            }
            std::unique_lock<std::mutex> mutex_;
            global_path_.poses.push_back(pt);
          }
        }
        pMapNode = map_it.nextNode();
      }

      map_name_ = map_name;
      map_base_confirmed_.exchange(confirmed);
      map_base_.SetX(base_x);
      map_base_.SetY(base_y);
      map_base_.SetZ(base_z);
      map_range_inited_.exchange(inited);
      map_rangex_min_.exchange(rangex_min);
      map_rangex_max_.exchange(rangex_max);
      map_rangey_max_.exchange(rangey_max);
      map_rangey_min_.exchange(rangey_min);
      map_rangez_max_.exchange(rangez_max);
      map_rangez_min_.exchange(rangez_min);
    }

    void HDMap::MapRangeSet(const geometry_msgs::PoseStamped &pt)
    {
      if (map_range_inited_.load())
      {
        if (pt.pose.position.x < map_rangex_min_.load())
          map_rangex_min_.exchange(pt.pose.position.x);
        if (pt.pose.position.x > map_rangex_max_.load())
          map_rangex_max_.exchange(pt.pose.position.x);
        if (pt.pose.position.y < map_rangey_min_.load())
          map_rangey_min_.exchange(pt.pose.position.y);
        if (pt.pose.position.y > map_rangey_max_.load())
          map_rangey_max_.exchange(pt.pose.position.y);
        if (pt.pose.position.z < map_rangez_min_.load())
          map_rangez_min_.exchange(pt.pose.position.z);
        if (pt.pose.position.z > map_rangez_max_.load())
          map_rangez_max_.exchange(pt.pose.position.z);
      }
      else
      {
        //Init Map Range
        map_rangex_min_.exchange(pt.pose.position.x);
        map_rangex_max_.exchange(pt.pose.position.x);
        map_rangey_min_.exchange(pt.pose.position.y);
        map_rangey_max_.exchange(pt.pose.position.y);
        map_rangez_min_.exchange(pt.pose.position.z);
        map_rangez_max_.exchange(pt.pose.position.z);
        map_range_inited_.exchange(true);
      }
    }
    // for ()

    //TODO:other road elements, lifei, 2020/08/31
    // }

    // void HDMap::AddLaneLineRefPoint(road::LaneLineRefPoint p)
    // {
    //   uint16_t id = 0;
    //   if (map_lane_line_ref_points_.empty() == false)
    //   {
    //     auto itr = map_lane_line_ref_points_.end();
    //     itr--;
    //     id = itr->second.GetId() + 1;
    //   }
    //   map_lane_line_ref_points_[id] = p;
    // }
    // void HDMap::DeleteLaneLineRefPoint(uint16_t id)
    // {
    //   if (map_lane_line_ref_points_.find(id) != map_lane_line_ref_points_.end())
    //     map_lane_line_ref_points_.erase(id);
    // }
    // void HDMap::AddLaneRefPoint(road::LaneRefPoint p)
    // {
    //   uint16_t id = 0;
    //   if (map_lane_ref_points_.empty() == false)
    //   {
    //     auto itr = map_lane_ref_points_.end();
    //     itr--;
    //     id = itr->second.GetId() + 1;
    //   }
    //   map_lane_ref_points_[id] = p;
    //   map_graph_.InsertVertex(id, std::shared_ptr<road::LaneRefPoint>(&map_lane_ref_points_[id]));
    // }
    // void HDMap::AddLaneRefPoint(const road::LaneLineRefPoint &p1,
    //                             const road::LaneLineRefPoint &p2)
    // {
    //   road::LaneRefPoint p;
    //   p.SetX((p1.GetX() + p2.GetX()) / 2);
    //   p.SetY((p1.GetY() + p2.GetY()) / 2);
    //   p.SetZ((p1.GetZ() + p2.GetZ()) / 2);
    //   p.SetS(0);
    //   p.SetDirecr((p1.GetDirect() + p2.GetDirect()) / 2);
    //   AddLaneRefPoint(p);
    // }
    // void HDMap::DeleteLaneRefPoint(uint16_t id)
    // {
    //   if (map_lane_ref_points_.find(id) != map_lane_ref_points_.end())
    //     map_lane_ref_points_.erase(id);
    //   map_graph_.DeleteVertex(id);
    // }
    // std::shared_ptr<road::LaneLine> HDMap::GenerateLaneLine(const road::LaneLineRefPoint &src,
    //                                                         const road::LaneLineRefPoint &dst,
    //                                                         enum common::geometry::CurvType type)
    // {
    // }
    // std::shared_ptr<road::LaneCenter> HDMap::GenerateLaneCenter(const road::LaneRefPoint &src,
    //                                                             const std::shared_ptr<road::LaneLine> &left,
    //                                                             const std::shared_ptr<road::LaneLine> &right,
    //                                                             enum common::geometry::CurvType type)
    // {
    // }
    // std::shared_ptr<road::LaneCenter> HDMap::LinkLaneRefPoint(const road::LaneRefPoint &src,
    //                                                           const road::LaneRefPoint &dst)
    // {
    // }

  } // namespace map
} // namespace nirvana
