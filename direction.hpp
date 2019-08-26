#pragma once

#include <exception>
#include <iostream>

class TDirection
{
public:
  TDirection (int dx, int dy)
  : Dx (dx)
  , Dy (dy)
  {
  }
  TDirection ()
  : TDirection (0, 0)
  {
  }

  TDirection (const std::string& s)
  : Dx (0)
  , Dy (0)
  {
    if (s == "right") {
      Dx = 1;
    } else if (s == "left") {
      Dx = -1;
    } else if (s == "up") {
      Dy = 1;
    } else if (s == "down") {
      Dy = -1;
    } else if (s.empty ()) {
      // pass
    } else {
      std::cerr << "Unknown direction: '" << s << "'" << std::endl;
      throw std::exception{};
    }
  }

  bool Compatible (int dx, int dy)
  {
    if (Dx && Dx * dx < 0) {
      return false;
    }
    if (Dy && Dy * dy < 0) {
      return false;
    }
    return true;
  }

  std::string ToString () const
  {
    if (Dx > 0) {
      return "right";
    } else if (Dx < 0) {
      return "left";
    } else if (Dy > 0) {
      return "up";
    } else if (Dy < 0) {
      return "down";
    }
    return "";
  }

  int Dx, Dy;
};

