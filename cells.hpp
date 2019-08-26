#pragma once

#include <deque>
#include <iomanip>
#include <ostream>
#include <vector>

class TCells
{
public:
  TCells () = default;
  TCells (const TCells&) = default;
  TCells (TCells&&) = default;
  TCells& operator= (const TCells&) = default;
  TCells& operator= (TCells&&) = default;

  void Reset ()
  {
    Cells.fill(0);
  }

  void Clear ()
  {
    Cells.fill(0);
  }

  char& At (int pos)
  {
    return Cells[pos];
  }

  char& At (int x, int y)
  {
    return Cells[x + WIDTH * y];
  }

  char& At (std::pair<int, int> pt)
  {
    return At (pt.first, pt.second);
  }

  char At (int pos) const
  {
    return Cells[pos];
  }

  char At (int x, int y) const
  {
    return Cells[x + WIDTH * y];
  }

  char At (std::pair<int, int> pt) const
  {
    return At (pt.first, pt.second);
  }

  bool Valid (int pos) const
  {
    return pos >= 0 && pos <= (int)Cells.size ();
  }

  bool Valid (int x, int y) const
  {
    return x >= 0 && y >= 0 && x < WIDTH && y < HEIGHT;
  }

  bool Valid (std::pair<int, int> pt) const
  {
    return Valid (pt.first, pt.second);
  }

  void DebugPrint (std::ostream& stream) const
  {
    for (size_t i = 0; i < Cells.size (); ++i) {
      stream << std::setw (2) << std::hex << std::setfill ('0') << int(Cells[i]) << ' ';
      if (i % WIDTH == WIDTH - 1) {
        stream << "\n";
      }
    }
  }

  std::vector<bool> FillMask (int id, int tail) const
  {
    std::vector<bool> mask;
    mask.assign (WIDTH * HEIGHT, true);
    std::deque<int> border;

#define PUSH(pt)                \
  {                             \
    const auto c = Cells[pt];   \
    if (c != id && c != tail) { \
      border.push_back (pt);    \
    }                           \
  }

    for (int x = 0; x < WIDTH; ++x) {
      PUSH (x);
      PUSH (x + WIDTH * (HEIGHT - 1));
    }
    for (int y = 1; y < HEIGHT - 1; ++y) {
      PUSH (0 + y * WIDTH);
      PUSH (WIDTH - 1 + y * WIDTH);
    }

    while (!border.empty ()) {
      const int current = border.front ();
      border.pop_front ();

      mask[current] = false;
      const int x = current % WIDTH;
      const int y = current / WIDTH;
      if (x > 0) {
        const int next = current - 1;
        if (mask[next]) {
          PUSH (next);
        }
      }
      if (x < WIDTH - 1) {
        const int next = current + 1;
        if (mask[next]) {
          PUSH (next);
        }
      }
      if (y > 0) {
        const int next = current - WIDTH;
        if (mask[next]) {
          PUSH (next);
        }
      }
      if (y < HEIGHT - 1) {
        const int next = current + WIDTH;
        if (mask[next]) {
          PUSH (next);
        }
      }
    }
    return mask;
  }

  int Fill (int id, int tail)
  {
    auto mask = FillMask (id, tail);

    int numChanged = 0;
    for (size_t i = 0; i < Cells.size (); ++i) {
      if (mask[i]) {
        if (Cells[i] != id) {
          ++numChanged;
        }
        Cells[i] = id;
      }
    }
    return numChanged;
  }

  std::vector<int> GetPointNeighbours (int pt) const
  {
    const int x = pt % WIDTH;
    const int y = pt / WIDTH;
    std::vector<int> v;
    if (x > 0) {
      v.push_back (pt - 1);
    }
    if (x < WIDTH - 1) {
      v.push_back (pt + 1);
    }
    if (y > 0) {
      v.push_back (pt - WIDTH);
    }
    if (y < HEIGHT - 1) {
      v.push_back (pt + WIDTH);
    }
    return v;
  }

  bool HaveOwnedNeighbours (int pt, int owner) const
  {
    const int x = pt % WIDTH;
    const int y = pt / WIDTH;
    if (x > 0 && At (pt - 1) == owner) {
      return true;
    }
    if (x < WIDTH - 1 && At (pt + 1) == owner) {
      return true;
    }
    if (y > 0 && At (pt - WIDTH) == owner) {
      return true;
    }
    if (y < HEIGHT - 1 && At (pt + WIDTH) == owner) {
      return true;
    }
    return false;
  }

  bool HaveNotOwnedNeighbours (int pt, int owner) const
  {
    const int x = pt % WIDTH;
    const int y = pt / WIDTH;
    if (x > 0 && At (pt - 1) != owner) {
      return true;
    }
    if (x < WIDTH - 1 && At (pt + 1) != owner) {
      return true;
    }
    if (y > 0 && At (pt - WIDTH) != owner) {
      return true;
    }
    if (y < HEIGHT - 1 && At (pt + WIDTH) != owner) {
      return true;
    }
    return false;
  }

  std::array<char, WIDTH*HEIGHT> Cells;
};
