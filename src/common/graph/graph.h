#pragma once

#include <string>
#include <memory>
#include <map>

namespace nirvana
{
  namespace common
  {
    namespace graph
    {
      template <typename VertexT, typename EdgeT>
      class Edge;

      //Vertex=====================================================================
      template <typename VertexT, typename EdgeT>
      class Vertex
      {
        friend class Edge<VertexT, EdgeT>;

      public:
        Vertex(uint16_t id = 0, const std::shared_ptr<VertexT> &data = nullptr);
        virtual ~Vertex() {}

        uint16_t GetId() const;
        std::shared_ptr<VertexT> GetData() const;
        std::shared_ptr<Edge<VertexT, EdgeT>> GetFirstInEdge() const;
        std::shared_ptr<Edge<VertexT, EdgeT>> GetFirstOutEdge() const;
        std::shared_ptr<Edge<VertexT, EdgeT>> LinkTo(const std::shared_ptr<Vertex<VertexT, EdgeT>> &vex,
                                                     const std::shared_ptr<EdgeT> &ed_data = nullptr);
        bool IsLinkedWith(const std::shared_ptr<Vertex> &vex);
        void DeleteVertex();

      private:
        uint16_t id_;
        std::shared_ptr<VertexT> data_;
        std::shared_ptr<Edge<VertexT, EdgeT>> first_in_edge_;
        std::shared_ptr<Edge<VertexT, EdgeT>> first_out_edge_;
      };

      template <typename VertexT, typename EdgeT>
      Vertex<VertexT, EdgeT>::Vertex(uint16_t id = 0, const std::shared_ptr<VertexT> &data = nullptr)
      {
        id_ = id;
        data_ = data;
      }

      template <typename VertexT, typename EdgeT>
      uint16_t Vertex<VertexT, EdgeT>::GetId() const
      {
        return id_;
      }

      template <typename VertexT, typename EdgeT>
      std::shared_ptr<VertexT> Vertex<VertexT, EdgeT>::GetData() const
      {
        return data_;
      }

      template <typename VertexT, typename EdgeT>
      std::shared_ptr<Edge<VertexT, EdgeT>> Vertex<VertexT, EdgeT>::GetFirstInEdge() const
      {
        return first_in_edge_;
      }

      template <typename VertexT, typename EdgeT>
      std::shared_ptr<Edge<VertexT, EdgeT>> Vertex<VertexT, EdgeT>::GetFirstOutEdge() const
      {
        return first_out_edge_;
      }

      template <typename VertexT, typename EdgeT>
      std::shared_ptr<Edge<VertexT, EdgeT>> Vertex<VertexT, EdgeT>::LinkTo(const std::shared_ptr<Vertex<VertexT, EdgeT>> &vex,
                                                                           const std::shared_ptr<EdgeT> &ed_data = nullptr)
      {
        std::shared_ptr<Edge<VertexT, EdgeT>> ed = std::shared_ptr<Edge<VertexT, EdgeT>>(new Edge<VertexT, EdgeT>(this, vex, ed_data));
        ed->next_edge_dst_ = vex->first_in_edge_;
        vex->first_in_edge_ = ed;
        ed->next_edge_src_ = this->first_out_edge_;
        this->first_out_edge_ = ed;
        return ed;
      }

      template <typename VertexT, typename EdgeT>
      bool Vertex<VertexT, EdgeT>::IsLinkedWith(const std::shared_ptr<Vertex> &vex)
      {
        for (std::shared_ptr<Edge<VertexT, EdgeT>> i = first_out_edge_; i != nullptr; i = i->next_edge_src_)
          if (vex == i->GetVertexDst())
            return true;
        return false;
      }

      template <typename VertexT, typename EdgeT>
      void Vertex<VertexT, EdgeT>::DeleteVertex()
      {
        std::shared_ptr<Edge<VertexT, EdgeT>> next_edge;
        for (std::shared_ptr<Edge<VertexT, EdgeT>> i = first_in_edge_; i != nullptr; i = next_edge)
        {
          next_edge = i->next_edge_dst_;
          i->DeleteEdge();
        }

        for (std::shared_ptr<Edge<VertexT, EdgeT>> i = first_out_edge_; i != nullptr; i = next_edge)
        {
          next_edge = i->next_edge_src_;
          i->DeleteEdge();
        }

        delete this;
      }

      //Edge==============================================================================
      template <typename VertexT, typename EdgeT>
      class Edge
      {
        friend class Vertex<VertexT, EdgeT>;

      public:
        Edge() {}
        Edge(const std::shared_ptr<Vertex<VertexT, EdgeT>> &src,
             const std::shared_ptr<Vertex<VertexT, EdgeT>> &dst,
             const std::shared_ptr<EdgeT> &ed_data = nullptr);
        virtual ~Edge() {}

        void SetSrcAndDst(const std::shared_ptr<Vertex<VertexT, EdgeT>> &src,
                          const std::shared_ptr<Vertex<VertexT, EdgeT>> &dst);
        void DeleteEdge();
        std::shared_ptr<Vertex<VertexT, EdgeT>> GetVertexSrc() const;
        std::shared_ptr<Vertex<VertexT, EdgeT>> GetVertexDst() const;
        std::shared_ptr<Edge> GetNextEdgeSrc() const;
        std::shared_ptr<Edge> GetNextEdgeDst() const;
        void SetEdge(const std::shared_ptr<EdgeT> &ed);
        std::shared_ptr<EdgeT> GetEdge() const;

      private:
        std::shared_ptr<Vertex<VertexT, EdgeT>> vertex_src_;
        std::shared_ptr<Vertex<VertexT, EdgeT>> vertex_dst_;
        std::shared_ptr<Edge> next_edge_src_;
        std::shared_ptr<Edge> next_edge_dst_;
        std::shared_ptr<EdgeT> edge_data_;
      };

      template <typename VertexT, typename EdgeT>
      Edge<VertexT, EdgeT>::Edge(const std::shared_ptr<Vertex<VertexT, EdgeT>> &src,
                                 const std::shared_ptr<Vertex<VertexT, EdgeT>> &dst,
                                 const std::shared_ptr<EdgeT> &ed_data = nullptr)
          : next_edge_src_(nullptr), next_edge_dst_(nullptr), edge_data_(ed_data)
      {
        SetSrcAndDst(src, dst);
      }

      template <typename VertexT, typename EdgeT>
      void Edge<VertexT, EdgeT>::SetSrcAndDst(const std::shared_ptr<Vertex<VertexT, EdgeT>> &src,
                                              const std::shared_ptr<Vertex<VertexT, EdgeT>> &dst)
      {
        vertex_src_ = src;
        vertex_dst_ = dst;
      }

      template <typename VertexT, typename EdgeT>
      void Edge<VertexT, EdgeT>::DeleteEdge()
      {
        std::shared_ptr<Vertex<VertexT, EdgeT>> s = this->vertex_src_;
        if (s->first_out_edge_->vertex_dst_ == vertex_dst_)
          s->first_out_edge_ = s->first_out_edge_->next_edge_src_;
        else
        {
          for (std::shared_ptr<Edge<VertexT, EdgeT>> i = s->first_out_edge_; i->next_edge_src_ != nullptr; i = i->next_edge_src_)
          {
            if (i->next_edge_src_ == vertex_dst_)
            {
              i->next_edge_src_ = i->next_edge_src_->next_edge_src_;
              break;
            }
          }
        }

        std::shared_ptr<Vertex<VertexT, EdgeT>> d = this->vertex_dst_;
        if (d->first_in_edge_->vertex_src_ == vertex_src_)
          d->first_in_edge_ = d->first_in_edge_->next_edge_dst_;
        else
        {
          for (std::shared_ptr<Edge<VertexT, EdgeT>> i = s->first_in_edge_; i->next_edge_dst_ != nullptr; i = i->next_edge_dst_)
          {
            if (i->next_edge_dst_ == vertex_src_)
            {
              i->next_edgedst_ = i->next_edge_dst_->next_edge_dst_;
              break;
            }
          }
        }
        delete this;
      }

      template <typename VertexT, typename EdgeT>
      std::shared_ptr<Vertex<VertexT, EdgeT>> Edge<VertexT, EdgeT>::GetVertexSrc() const
      {
        return vertex_src_;
      }

      template <typename VertexT, typename EdgeT>
      std::shared_ptr<Vertex<VertexT, EdgeT>> Edge<VertexT, EdgeT>::GetVertexDst() const
      {
        return vertex_dst_;
      }

      template <typename VertexT, typename EdgeT>
      std::shared_ptr<Edge<VertexT, EdgeT>> Edge<VertexT, EdgeT>::GetNextEdgeSrc() const
      {
        return next_edge_src_;
      }

      template <typename VertexT, typename EdgeT>
      std::shared_ptr<Edge<VertexT, EdgeT>> Edge<VertexT, EdgeT>::GetNextEdgeDst() const
      {
        return next_edge_dst_;
      }

      template <typename VertexT, typename EdgeT>
      void Edge<VertexT, EdgeT>::SetEdge(const std::shared_ptr<EdgeT> &ed)
      {
        edge_data_ = ed;
      }

      template <typename VertexT, typename EdgeT>
      std::shared_ptr<EdgeT> Edge<VertexT, EdgeT>::GetEdge() const
      {
        return edge_data_;
      }

      //Graph========================================================================================
      template <typename VertexT, typename EdgeT>
      class Graph
      {
      public:
        Graph() {}
        virtual ~Graph() {}

        std::shared_ptr<Vertex<VertexT, EdgeT>> InsertVertex(uint16_t id = 0, std::shared_ptr<VertexT> data = nullptr);
        bool LinkVertex(const std::shared_ptr<Vertex<VertexT, EdgeT>> &src,
                        const std::shared_ptr<Vertex<VertexT, EdgeT>> &dst,
                        const std::shared_ptr<EdgeT> &ed_data = nullptr);
        bool IsVertexLinked(const std::shared_ptr<Vertex<VertexT, EdgeT>> &src,
                            const std::shared_ptr<Vertex<VertexT, EdgeT>> &dst);
        std::shared_ptr<Vertex<VertexT, EdgeT>> GetVertex(uint16_t id);
        void DeleteVertex(uint16_t id);

        void Clear();

      private:
        std::map<uint16_t, std::shared_ptr<Vertex<VertexT, EdgeT>>> vertex_map_;
      };

      template <typename VertexT, typename EdgeT>
      std::shared_ptr<Vertex<VertexT, EdgeT>> Graph<VertexT, EdgeT>::InsertVertex(uint16_t id = 0, std::shared_ptr<VertexT> data = nullptr)
      {
        auto itr = vertex_map_.find(id);
        if (itr != vertex_map_.end()) //Vertex already exist
          return nullptr;
        vertex_map_[id] = std::shared_ptr<Vertex<VertexT, EdgeT>>(new Vertex<VertexT, EdgeT>(id, data));
        return vertex_map_[id];
      }

      template <typename VertexT, typename EdgeT>
      bool Graph<VertexT, EdgeT>::LinkVertex(const std::shared_ptr<Vertex<VertexT, EdgeT>> &src,
                                             const std::shared_ptr<Vertex<VertexT, EdgeT>> &dst,
                                             const std::shared_ptr<EdgeT> &ed_data = nullptr)
      {
        if ((src == nullptr) || (dst == nullptr))
          return false;
        auto itr = vertex_map_.find(src->GetId());
        auto itr1 = vertex_map_.find(dst->GetId());
        if ((itr == vertex_map_.end()) || (itr1 == vertex_map_.end()))
          return false;
        src->LinkTo(dst, ed_data);
        return true;
      }

      template <typename VertexT, typename EdgeT>
      bool Graph<VertexT, EdgeT>::IsVertexLinked(const std::shared_ptr<Vertex<VertexT, EdgeT>> &src,
                                                 const std::shared_ptr<Vertex<VertexT, EdgeT>> &dst)
      {
        if ((src == nullptr) || (dst == nullptr))
          return false;
        auto itr = vertex_map_.find(src->GetId());
        auto itr1 = vertex_map_.find(dst->GetId());
        if ((itr == vertex_map_.end()) || (itr1 == vertex_map_.end()))
          return false;
        return src->IsLinkedWith(dst);
      }

      template <typename VertexT, typename EdgeT>
      std::shared_ptr<Vertex<VertexT, EdgeT>> Graph<VertexT, EdgeT>::GetVertex(uint16_t id)
      {
        auto itr = vertex_map_.find(id);
        return itr == vertex_map_.end() ? nullptr : itr->second;
      }

      template <typename VertexT, typename EdgeT>
      void Graph<VertexT, EdgeT>::DeleteVertex(uint16_t id)
      {
        std::shared_ptr<Vertex<VertexT, EdgeT>> vertx = GetVertex(id);
        if (vertx != nullptr)
          vertx->DeleteVertex();
      }

      template <typename VertexT, typename EdgeT>
      void Graph<VertexT, EdgeT>::Clear()
      {
        for (auto itr = vertex_map_.begin(); itr != vertex_map_.end(); itr++)
          itr->second->DeleteVertex();
      }

    } // namespace graph
  }   // namespace common
} // namespace nirvana