#pragma once
#define WIDTH 31
#define HEIGHT 31

#include "cells.hpp"

class TLevit
{
public:
  const int MAX_DIST = 1000000000;

  using IntSquare = std::array<int, WIDTH * HEIGHT>;

  void ProcessVertex (int to, int cur, const TCells& cells, IntSquare& id, std::deque<int>& q, int avoid)
  {
    if (cells.Cells[to] == avoid) {
      return;
    }
    if (Dist[to] <= Dist[cur] + 1) {
      return;
    }
    Dist[to] = Dist[cur] + 1;
    if (id[to] == 0) {
      q.push_back (to);
    } else if (id[to] == 1) {
      q.push_front (to);
    }
    Prev[to] = cur;
    id[to] = 1;
  }

  void Apply (const TCells& cells, int x, int y, int avoid)
  {
    IntSquare id;
    std::deque<int> q;
    const int origin = x + y * WIDTH;

    Dist.fill (MAX_DIST);
    Prev.fill (-1);
    id.fill (0);
    q.push_back (origin);
    Dist[q.back ()] = 0;

    while (!q.empty ()) {
      const int v = q.front ();
      q.pop_front ();

      id[v] = 1;
      const int w = WIDTH;
      const int h = HEIGHT;
      const int x = v % w;
      const int y = v / w;
      if (x > 0) {
        ProcessVertex (v - 1, v, cells, id, q, avoid);
      }
      if (y > 0) {
        ProcessVertex (v - w, v, cells, id, q, avoid);
      }
      if (x < w - 1) {
        ProcessVertex (v + 1, v, cells, id, q, avoid);
      }
      if (y < h - 1) {
        ProcessVertex (v + w, v, cells, id, q, avoid);
      }
    }
  }

  IntSquare Dist;
  IntSquare Prev;
};
