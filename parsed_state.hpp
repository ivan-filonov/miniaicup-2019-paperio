#pragma once

#include <iostream>

#include "nlohmann/json.hpp"


static const int C_EMPTY = 0;
static const int C_SELF = 7;
static const int C_TAIL_OFS = 8;
static const int C_SELF_TAIL = C_SELF + C_TAIL_OFS;
static const int C_NITRO = C_SELF_TAIL + 2;
static const int C_SLOW = C_SELF_TAIL + 3;
static const int C_SAW = C_SELF_TAIL + 4;

class TParsedState
{
public:
  using TJson = nlohmann::json;

  bool Parse (const std::string& stateStr)
  {
    auto jstate = TJson::parse (stateStr);
    const std::string type = jstate["type"];
    if (type == "tick") {
      ParseTick (jstate["params"]);
    } else {
      CurrentTick = 0;
      if (type == "start_game") {
        ParseStart (jstate["params"]);
      } else if (type == "end_game") {
        //
      }
    }

    return CurrentTick > 0;
  }

  void PrintKeys (const std::string& what, const TJson& json)
  {
    std::cerr << what;
    for (const auto& it : json.items ()) {
      std::cerr << it.key () << "\t";
    }
    std::cerr << "\n";
  }

  void ParseTick (const TJson& jparams)
  {
    CurrentTick = jparams["tick_num"];
    for (auto& player : Players) {
      player.Alive = false;
    }
    for (const auto& it : jparams["players"].items ()) {
      const int id = (it.key () == "i") ? C_SELF : std::stoi (it.key ());
      ParsePlayer (id, it.value ());
    }
    for (const auto& jbonus : jparams["bonuses"]) {
      auto& bonus = Bonuses.emplace_back ();
      bonus.Point.Parse (jbonus["position"]);
      Point2Cell (bonus.Point, bonus.Cell);
      bonus.Type = jbonus["type"];
      if (bonus.Type == "n") {
        bonus.TypeCode = C_NITRO;
      } else if (bonus.Type == "s") {
        bonus.TypeCode = C_SLOW;
      } else if (bonus.Type == "saw") {
        bonus.TypeCode = C_SAW;
      } else {
        std::cerr << jbonus << "\n";
      }
    }
  }

  void ParsePlayer (int id, const TJson& jplayer)
  {
    auto& player = Players.at (id);
    player.Alive = true;
    player.Score = jplayer["score"];
    for (const auto& jbonus : jplayer["bonuses"]) {
      const int ticks = jbonus["ticks"];
      const std::string type = jbonus["type"];
      (void)ticks;
      (void)type;
    }
    player.Point.Parse (jplayer["position"]);
    const auto& jdir = jplayer["direction"];
    if (jdir.is_string ()) {
      player.Direction = jdir;
    } else {
      player.Direction.clear ();
    }
    ParseCoords (jplayer["lines"], player.Lines, player.LineCells);
    ParseCoords (jplayer["territory"], player.Territory, player.TerrCells);
    Point2Cell (player.Point, player.Cell);
  }

  void ParseStart (const TJson& jparams)
  {
    CellSize = jparams["width"];
    BaseSpeed = jparams["speed"];
    FieldWidth = jparams["x_cells_count"];
    FieldHeight = jparams["y_cells_count"];
    for (auto& player : Players) {
      player.Lines.reserve (FieldWidth * FieldHeight);
      player.Territory.reserve (FieldWidth * FieldHeight);
      player.LineCells.reserve (FieldWidth * FieldHeight);
      player.TerrCells.reserve (FieldWidth * FieldHeight);
    }
  }

  struct TCoords {
    int X;
    int Y;

    void Parse (const TJson& jcoords)
    {
      auto it = jcoords.cbegin ();
      X = *it++;
      Y = *it;
    }
  };

  void Point2Cell (const TCoords& pt, TCoords& cell)
  {
    cell.X = (pt.X - CellSize / 2) / CellSize;
    cell.Y = (pt.Y - CellSize / 2) / CellSize;
  }

  void ParseCoords (const TJson& jpoints, std::vector<TCoords>& points, std::vector<TCoords>& cells)
  {
    points.clear ();
    for (const auto& jpoint : jpoints) {
      points.emplace_back ().Parse (jpoint);
    }
    cells.resize (points.size ());
    for (size_t i = 0; i < points.size (); ++i) {
      Point2Cell (points[i], cells[i]);
    }
  }

  struct TBonus {
    TCoords Point;
    TCoords Cell;
    std::string Type;
    char TypeCode; // 17=n 18=s 19=saw
  };

  struct TPlayer {
    int Score;
    std::string Direction;
    TCoords Point;
    TCoords Cell;
    std::vector<TCoords> Lines;
    std::vector<TCoords> Territory;
    std::vector<TCoords> LineCells;
    std::vector<TCoords> TerrCells;
    bool Alive;
  };

  int CurrentTick;
  int CellSize;
  int BaseSpeed;
  int FieldWidth;
  int FieldHeight;

  std::vector<TBonus> Bonuses;
  std::array<TPlayer, 8> Players;
};
