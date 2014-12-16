/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * \file
 *
 * Implementation of shortest_path.h
 *
 * \author Bhaskara Marthi
 */

#include "shortest_path_result.h"
#include <occupancy_grid_utils/shortest_path.h>
#include <vector>
#include <functional>
#include <ros/assert.h>
#include <queue>

namespace occupancy_grid_utils
{

namespace nm=nav_msgs;
namespace gm=geometry_msgs;

using std::vector;
using boost::optional;
using std::max;

TerminationCondition::TerminationCondition ()
{}

TerminationCondition::TerminationCondition (const double max_distance,
                                            const bool use_cells) :
  max_distance_(max_distance), use_cells_(use_cells)
{
  if (use_cells_)
    ROS_WARN ("Deprecated usage of version of TerminationCondition that uses"
              " distance in cells rather than meters.  This API will change"
              " in a hard-to-debug way in future!");
}

TerminationCondition::TerminationCondition (const Cells& goals) :
  goals_(goals)
{
}

TerminationCondition::TerminationCondition (const Cells& goals, const double max_distance,
                                            const bool use_cells) :
  max_distance_(max_distance), goals_(goals), use_cells_(use_cells)
{
  if (use_cells_)
    ROS_WARN ("Deprecated usage of version of TerminationCondition that uses"
              " distance in cells rather than meters.  This API will change"
              " in a hard-to-debug way in future!");
}


typedef vector<Cell> Path;
typedef boost::shared_ptr<nm::OccupancyGrid> GridPtr;

boost::optional<double> distanceTo(ResultPtr res, const Cell& dest)
{
  boost::optional<double> d = res->potential[cellIndex(res->info, dest)];
  if (d)
    return *d * res->info.resolution;
  else
    return d;
}


boost::optional<Path> extractPath(ResultPtr res, const Cell& dest)
{
  boost::optional<Path> p;
  index_t current=cellIndex(res->info, dest);
  if (!res->back_pointers[current])
    return p; // No path exists

  index_t max=res->back_pointers.size();
  index_t n=0;
  p = Path();

  do {
    if (n++ > max) {
      ROS_FATAL("Cycle in extractPath");
      ROS_BREAK();
    }
    p->push_back(indexCell(res->info, current));
    ROS_ASSERT_MSG(res->back_pointers[current], "Back pointer not found for %u", current);
    current=*(res->back_pointers[current]);
  } while (current!=res->src_ind);
  p->push_back(indexCell(res->info, current));
  reverse(p->begin(), p->end());
  return p;
}



struct QueueItem
{
  index_t ind;
  double potential;
  QueueItem (const index_t ind, const double potential) :
    ind(ind), potential(potential) {}
};

bool operator< (const QueueItem& i1, const QueueItem& i2)
{
  return i1.potential > i2.potential;
}


ResultPtr singleSourceShortestPaths (const nm::OccupancyGrid& g, const Cell& src,
                                     const bool manhattan)
{
  return singleSourceShortestPaths(g, src, TerminationCondition(), manhattan);
}

ResultPtr singleSourceShortestPaths (const nm::OccupancyGrid& g, const Cell& src,
                                     const TerminationCondition& t,
                                     const bool manhattan)
{
  verifyDataSize(g);
  ShortestPathResult* res = new ShortestPathResult();
  ResultPtr result(res);
  res->info = g.info;
  res->src_ind = cellIndex(g.info, src);
  res->back_pointers.resize(g.data.size());
  res->potential.resize(g.data.size());

  std::priority_queue<QueueItem> q;
  q.push(QueueItem(res->src_ind, 0.0));
  res->potential[res->src_ind] = 0.0;
  ROS_DEBUG_NAMED ("shortest_path", "Computing single source shortest paths from %d, %d", src.x, src.y);

  Cells remaining_goals;
  if (t.goals_)
    remaining_goals = *t.goals_;

  while (!q.empty()) {
    const QueueItem i=q.top();
    q.pop();

    const Cell cell = indexCell(g.info, i.ind);
    const double dist = i.potential;
    const double dist_in_meters = dist*g.info.resolution;
    if (t.max_distance_ &&
        (*t.max_distance_ < (t.use_cells_ ? dist : dist_in_meters)))
        break;
    if (t.goals_) {
      remaining_goals.erase(cell);
      if (remaining_goals.empty())
        break;
    }

    for (int dx=-1; dx<=1; dx++) {
      for (int dy=-1; dy<=1; dy++) {
        if ((dx==0) && (dy==0))
          continue;
        if ((dx!=0) && (dy!=0) && manhattan)
          continue;
        const double d = ((dx==0) || (dy==0)) ? 1 : sqrt(2);
        const Cell neighbor(cell.x+dx, cell.y+dy);
        if (withinBounds(g.info, neighbor)) {
          const index_t ind=cellIndex(g.info, neighbor);
          const double new_dist = dist+d;
          if (g.data[ind]==UNOCCUPIED &&
              (!res->potential[ind] || *(res->potential[ind]) > new_dist) &&
              (!t.max_distance_ ||
               (t.use_cells_ && *t.max_distance_ >= new_dist) ||
               (!t.use_cells_ && *t.max_distance_ >= new_dist*g.info.resolution)))
          {
            ROS_DEBUG_NAMED ("shortest_path_propagation", "Adding cell %d, %d, at distance %.2f",
                             neighbor.x, neighbor.y, new_dist);
            res->potential[ind] = new_dist;
            res->back_pointers[ind] = i.ind;
            q.push(QueueItem(ind, new_dist));
          }
        }
      }
    }
  }
  ROS_DEBUG_NAMED ("shortest_path", "Done computing single source shortest paths from %d, %d", src.x, src.y);
  return result;
}

ResultPtr singleSourceShortestPathsToUnknown (const nm::OccupancyGrid& g, const Cell& src,
                                     const TerminationCondition& t,
                                     const bool manhattan)
{
  verifyDataSize(g);
  ShortestPathResult* res = new ShortestPathResult();
  ResultPtr result(res);
  res->info = g.info;
  res->src_ind = cellIndex(g.info, src);
  res->back_pointers.resize(g.data.size());
  res->potential.resize(g.data.size());

  std::priority_queue<QueueItem> q;
  q.push(QueueItem(res->src_ind, 0.0));
  res->potential[res->src_ind] = 0.0;
  ROS_DEBUG_NAMED ("shortest_path", "Computing single source shortest paths from %d, %d", src.x, src.y);

  Cells remaining_goals;
  if (t.goals_)
    remaining_goals = *t.goals_;

  while (!q.empty()) {
    const QueueItem i=q.top();
    q.pop();

    const Cell cell = indexCell(g.info, i.ind);
    const double dist = i.potential;
    const double dist_in_meters = dist*g.info.resolution;
    if (t.max_distance_ &&
        (*t.max_distance_ < (t.use_cells_ ? dist : dist_in_meters)))
        break;
    if (t.goals_) {
      remaining_goals.erase(cell);
      if (remaining_goals.empty())
        break;
    }

    for (int dx=-1; dx<=1; dx++) {
      for (int dy=-1; dy<=1; dy++) {
        if ((dx==0) && (dy==0))
          continue;
        if ((dx!=0) && (dy!=0) && manhattan)
          continue;
        const double d = ((dx==0) || (dy==0)) ? 1 : sqrt(2);
        const Cell neighbor(cell.x+dx, cell.y+dy);
        if (withinBounds(g.info, neighbor)) {
          const index_t ind=cellIndex(g.info, neighbor);
          const double new_dist = dist+d;
          /*if (g.data[ind] == UNKNOWN)
          {
              return g.data[ind];
          }*/
          if (g.data[ind]==UNOCCUPIED &&
              (!res->potential[ind] || *(res->potential[ind]) > new_dist) &&
              (!t.max_distance_ ||
               (t.use_cells_ && *t.max_distance_ >= new_dist) ||
               (!t.use_cells_ && *t.max_distance_ >= new_dist*g.info.resolution)))
          {
            ROS_DEBUG_NAMED ("shortest_path_propagation", "Adding cell %d, %d, at distance %.2f",
                             neighbor.x, neighbor.y, new_dist);
            res->potential[ind] = new_dist;
            res->back_pointers[ind] = i.ind;
            q.push(QueueItem(ind, new_dist));
          }
        }
      }
    }
  }
  ROS_DEBUG_NAMED ("shortest_path", "Done computing single source shortest paths from %d, %d", src.x, src.y);
  return result;
}

inline
bool myGt (const signed char x, const signed char y)
{
  return (x==-1 && y==0) || x>y;
}

inline long distance2(int x, int y)
{
    return x*x+y*y;
}

struct InflationQueueItem
{
    Cell center;
    Cell cell;
    long dist2;

    InflationQueueItem (const Cell center, const Cell cell)
        : center(center)
        , cell(cell)
        , dist2(distance2(cell.x-center.x, cell.y-center.y))
    {}

    bool operator>(const InflationQueueItem& rhs) const
    {
        if (dist2 == rhs.dist2)
        {
            if (cell == rhs.cell)
            {
                if (center == rhs.center)
                    return true;
                else
                    return center < rhs.center;
            }
            else
                return cell < rhs.cell;
        }
        else
            return dist2 > rhs.dist2;
    }
};

nm::OccupancyGrid inflateObstacles (const nm::OccupancyGrid& g,
                                    boost::function<signed char (float)> distanceToOccupancyFn,
                                    const bool allow_unknown)
{
    std::vector<bool> seen(g.info.height*g.info.width);
    std::priority_queue<InflationQueueItem,
                        std::vector<InflationQueueItem>,
                        std::greater<InflationQueueItem> > queue;
    nm::OccupancyGrid g2(g);
    //for (int i=0; i<seen.size(); i++)
    //{
    //    if (g2.data[i] == OCCUPIED)
    //        g2.data[i] = 101;
    //}

    // Add initial obstacles
    for (coord_t x=0; x<(int)g.info.width; x++)
    {
        for (coord_t y=0; y<(int)g.info.height; y++)
        {
            const Cell cell(x,y);
            const index_t ind = cellIndex(g.info, cell);
            const signed char val=g.data[ind];
            if ((allow_unknown && val>=1) || (!allow_unknown && val!=0))
                queue.push(InflationQueueItem(cell, cell));
        }
    }

    while (!queue.empty())
    {
        const InflationQueueItem& q = queue.top();
        queue.pop();
        const index_t ind = cellIndex(g.info, q.cell);
        if (seen[ind])
            continue;
        seen[ind] = true;

        signed char occupancy = distanceToOccupancyFn(sqrt(q.dist2)*g.info.resolution);
        if (occupancy != UNKNOWN)
        {
            g2.data[ind] = std::max(g2.data[ind], occupancy);

            for (int i=0; i<4; i++)
            {
                const angles::StraightAngle& offset = angles::StraightAngle::rotations[i];
                const Cell c2(q.cell.x+offset.x(), q.cell.y+offset.y());
                //ROS_INFO("lol %d %d %ld", q.center.x, q.cell.y, InflationQueueItem(q.center, c2).dist2);

                if (withinBounds(g.info, c2))
                {
                    const index_t ind2 = cellIndex(g.info, c2);
                    if (!seen[ind2] && g.data[ind2] != UNKNOWN)
                        queue.push(InflationQueueItem(q.center, c2));
                }
            }
        }
    }
    return g2;
}

/************************************************************
 * A*
 ***********************************************************/

inline float manhattanHeuristic (const Cell& c1, const Cell& c2)
{
    return std::abs(c1.x-c2.x) + std::abs(c1.y-c2.y);
}

angles::StraightAngle getDirection(const Cell& from, const Cell& to)
{
    return angles::StraightAngle::getClosest(to.x-from.x, to.y-from.y);
}

boost::optional<Cell> closestFree(const nm::OccupancyGrid& g, const Cell& from, float maxRadius)
{
    maxRadius = ceil(maxRadius/g.info.resolution);
    boost::optional<Cell> bestCell;
    float bestDist = maxRadius*maxRadius+1;
    for (int x=-maxRadius; x <= maxRadius; x++)
    {
        for (int y=-maxRadius; y <= maxRadius; y++)
        {
            float dist = x*x+y*y;
            Cell cell(from.x+x,from.y+y);
            if (bestDist > dist &&
                withinBounds(g.info, cell) &&
                g.data[cellIndex(g.info, cell)] < OCCUPIED)
            {
                bestDist = dist;
                bestCell = cell;
            }
        }
    }
    return bestCell;
}

bool containsWall(const nm::OccupancyGrid& g, const Cell& center, float radius)
{
    radius = ceil(radius/g.info.resolution);
    for (int x=-radius; x <= radius; x++)
    {
        for (int y=-radius; y <= radius; y++)
        {
            float dist = x*x+y*y;
            Cell cell(center.x+x,center.y+y);
            if (radius*radius+1 > dist &&
                withinBounds(g.info, cell) &&
                g.data[cellIndex(g.info, cell)] == 101)
            {
                return true;
            }
        }
    }
    return false;
}

typedef std::pair<Path, double> AStarResult;
struct PQItem
{
    index_t parent_ind;
    index_t ind;
    angles::StraightAngle dir;

    float g_cost;
    float h_cost;
    float wall_cost;
    float acc_wall_cost;
    int segments;

    PQItem (index_t parent_ind, index_t ind, angles::StraightAngle dir)
        : parent_ind(parent_ind)
        , ind(ind)
        , dir(dir)
        , g_cost(0)
        , h_cost(0)
        , wall_cost(0)
        , acc_wall_cost(0)
        , segments(0)
    {}

    PQItem (index_t parent_ind, index_t ind, angles::StraightAngle dir, float g_cost, float h_cost, float wall_cost)
        : parent_ind(parent_ind)
        , ind(ind)
        , dir(dir)
        , g_cost(g_cost)
        , h_cost(h_cost)
        , wall_cost(wall_cost)
        , acc_wall_cost(0)
        , segments(0)
    {}

    float cost() const
    {
        return g_cost + h_cost;// + (wall_cost + acc_wall_cost)/(segments+1);
    }

    bool operator>(const PQItem& o) const
    {
        return cost() > o.cost();
    }
};

optional<AStarResult> shortestPathAStar(const nm::OccupancyGrid& g,
                                        const Cell& src,
                                        const Cell& dest,
                                        const bool searchForClosestUnknown,
                                        const angles::StraightAngle& srcDir)
{
    std::priority_queue<PQItem,
            std::vector<PQItem>,
            std::greater<PQItem> > queue;
    std::vector<bool> seen(g.info.height*g.info.width); // Default initialized to all false
    index_t dest_ind = cellIndex(g.info, dest);
    const index_t src_ind = cellIndex(g.info, src);
    queue.push(PQItem(src_ind, src_ind, srcDir, 0, g.info.resolution*manhattanHeuristic(src, dest), 0));
    std::map<index_t, index_t> parent;

    optional<AStarResult> res;

    while (!queue.empty())
    {
        const PQItem currItem = queue.top();
        queue.pop();
        const Cell curr = indexCell(g.info, currItem.ind);

        if (seen[currItem.ind])
            continue;
        seen[currItem.ind] = true;
        parent[currItem.ind] = currItem.parent_ind;

        if ((searchForClosestUnknown && g.data[currItem.ind] == UNKNOWN) ||
           (!searchForClosestUnknown && currItem.ind == dest_ind))
        {
            dest_ind = currItem.ind;
            res = AStarResult();
            res->second = currItem.g_cost;
            break;
        }

        for (int i=0; i<4; i++)
        {
            const angles::StraightAngle& offset = angles::StraightAngle::rotations[i];
            const Cell next(curr.x+offset.x(), curr.y+offset.y());

            if (withinBounds(g.info, next))
            {
                const index_t ind = cellIndex(g.info, next);
                PQItem nextItem(currItem.ind, ind, getDirection(curr, next));
                if (g.data[ind] < OCCUPIED)// && !seen[ind])
                {
                    nextItem.g_cost = currItem.g_cost + 1;
                    if (!searchForClosestUnknown)
                        nextItem.h_cost = manhattanHeuristic(next, dest);
                    nextItem.acc_wall_cost = currItem.acc_wall_cost;
                    nextItem.segments = currItem.segments;
                    float occupancy = g.data[ind];

                    if (currItem.dir == nextItem.dir)
                    {
                        nextItem.wall_cost = std::max(currItem.wall_cost, occupancy);
                    }
                    else
                    {
                        nextItem.g_cost += 5;
                        //if (containsWall(g, Cell(curr.x+offset.x()*0.15/g.info.resolution,
                        //                         curr.y+offset.y()*0.15/g.info.resolution), 0.1))
                        //    nextItem.g_cost += 1;
                        //else
                        //    nextItem.g_cost += 20;
                        nextItem.wall_cost = occupancy;
                        nextItem.acc_wall_cost += currItem.wall_cost;
                        nextItem.segments++;
                    }

                    queue.push(nextItem);
                }
                //ROS_DEBUG_STREAM_COND_NAMED (g.data[ind]!=UNOCCUPIED, "shortest_path_internal",
                //                             "  Skipping cell " << indexCell(g.info, ind) <<
                //                             " with cost " << (unsigned) g.data[ind]);
            }
        }
    }

    // Extract path if found
    if (res)
    {
        vector<index_t> path;
        path.push_back(dest_ind);
        const index_t src_ind = cellIndex(g.info, src);
        while (true)
        {
            index_t last = *(--path.end());
            if (last == src_ind)
                break;
            std::map<index_t, index_t>::const_iterator it = parent.find(last);
            ROS_ASSERT (it!=parent.end());
            index_t parent = it->second;
            ROS_ASSERT (parent!=last);
            path.push_back(parent);
        }
        for (int i=path.size()-1; i>=0; i--)
            res->first.push_back(indexCell(g.info, path[i]));
    }

    ROS_DEBUG_STREAM_NAMED ("shortest_path", "Computed shortest path.  Found = " << (bool) res.is_initialized());
    return res;
}



/************************************************************
 * Ros message conversion
 ************************************************************/

/// \brief Convert a shortest path result from a ros message
ResultPtr shortestPathResultFromMessage (const NavigationFunction& msg)
{
  ShortestPathResult* res = new ShortestPathResult();
  ResultPtr retval(res); // retval is a pointer to const, so we modify via res
  res->info = msg.info;
  res->src_ind = msg.source;

  const index_t num_cells = msg.valid.size();
  ROS_ASSERT (num_cells == msg.back_pointers.size());
  ROS_ASSERT (num_cells == msg.potential.size());
  res->back_pointers.resize(num_cells);
  res->potential.resize(num_cells);

  for (index_t i=0; i<num_cells; i++) {
    if (i==msg.source) {
      // Sanity check
      ROS_ASSERT (msg.back_pointers[i] == 123456);
      ROS_ASSERT (msg.potential[i] == -1.0);
      res->potential[i] = 0.0;
    }
    else if (msg.valid[i]) {
      res->back_pointers[i] = msg.back_pointers[i];
      res->potential[i] = msg.potential[i];
    }
    else {
      // Sanity check
      ROS_ASSERT (msg.back_pointers[i] == 234567);
      ROS_ASSERT (msg.potential[i] == -2.0);
    }
  }

  return retval;
}

/// \brief Convert a shortest path result to a ros message
NavigationFunction shortestPathResultToMessage (ResultPtr res)
{
  NavigationFunction msg;
  msg.info = res->info;
  msg.source = res->src_ind;

  const index_t num_cells=res->back_pointers.size();
  ROS_ASSERT (num_cells == res->potential.size());
  msg.valid.resize(num_cells);
  msg.back_pointers.resize(num_cells);
  msg.potential.resize(num_cells);

  for (index_t i=0; i<num_cells; i++) {
    if (i==msg.source) {
      msg.valid[i] = false;
      msg.potential[i] = -1.0;
      msg.back_pointers[i] = 123456;
    }
    else if (res->potential[i]) {
      msg.valid[i] = true;
      msg.potential[i] = *res->potential[i];
      msg.back_pointers[i] = *res->back_pointers[i];
    }
    else {
      msg.valid[i] = false;
      msg.potential[i] = -2.0;
      msg.back_pointers[i] = 234567;
    }
  }

  return msg;
}



} // namespace occupancy_grid_utils
