#pragma once
#include <memory>
#include <set>

namespace nirvana
{
  namespace map
  {
    namespace road
    {
      class IRoad
      {
      public:
        IRoad() = default;
        virtual ~IRoad() = default;
        virtual int GetId() const = 0;
        virtual void SetId(int id) = 0;
        virtual void InsertLaneLine(int lane_line_id) = 0;
        virtual void InsertLane(int lane_id) = 0;
        virtual std::set<int> GetLaneLineIds() const = 0;
        virtual std::set<int> GetLaneIds() const = 0;
        virtual void DeleteLaneLine(int lane_line_id) = 0;
        virtual void DeleteLane(int lane_id) = 0;
        virtual void Clear() = 0;
      };
    } // namespace road
  }   // namespace map
} // namespace nirvana