#if 0
#pragma GCC optimize("O3", "unroll-loops", "omit-frame-pointer", "inline") // Optimization flags
#pragma GCC option("arch=native", "tune=native", "no-zeroupper")           // Enable AVX
#pragma GCC target("avx")                                                  // Enable AVX
#endif

#define AICUP_ENVIRONMENT 0

#define WIDTH 31
#define HEIGHT 31

#include "cells.hpp"
#include "direction.hpp"
#include "levit.hpp"
#include "parsed_state.hpp"

#include <unistd.h>

#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <set>

#include "nlohmann/json.hpp"

using TJson = nlohmann::json;
using namespace std::chrono_literals;

static auto G_RNG = std::mt19937 (std::chrono::high_resolution_clock::now ().time_since_epoch ().count ());

static const auto PID = std::to_string (::getpid ()) + " ";

static const std::string G_PLAN_NAME_EMERGENCY = "emergency";
static const std::string G_PLAN_NAME_RANDOM = "random";
static const std::string G_PLAN_NAME_RANDOM_RECT = "random_rect";
static const std::string G_PLAN_NAME_FIELD = "field";

class TPlayerState
{
public:
  int MaxTick = 2499;
  int BaseSpeed;
  int CellSize;
  int X, Y, Pt;
  int PrevX, PrevY, PrevPt;
  TCells Cells;
  std::unordered_map<int, TLevit> Levits;
  std::array<int, WIDTH * HEIGHT> SafetyDist;
  std::array<int, WIDTH * HEIGHT> SafetyEnemy;
  int TailSafety;
  std::deque<int> Plan;
  TDirection Direction;
  std::vector<int> BorderPoints;
  int CurrentTick;

  //{{ copied from parsed state
  std::unordered_map<int, std::vector<int>> Territories;
  std::unordered_map<int, std::vector<int>> Tails;
  std::unordered_map<int, std::pair<int, int>> PlayersPt;
  std::unordered_map<int, std::pair<int, int>> PlayersPrevPt;
  std::array<bool, 8> IsAlive;
  //}}

  void FillBorderPoints ()
  {
    BorderPoints.clear ();
    const auto& terr = Territories[C_SELF];
    for (int pt : terr) {
      if (Cells.HaveNotOwnedNeighbours (pt, C_SELF)) {
        BorderPoints.push_back (pt);
      }
    }
  }

  void Reset (const TParsedState& state)
  {
    Cells.Reset ();
    SafetyDist.fill (WIDTH + HEIGHT + 1);
    SafetyEnemy.fill (0);
    X = Y = Pt = -1;
    PrevX = PrevY = PrevPt = -1;
    CellSize = state.CellSize;
    BaseSpeed = state.BaseSpeed;
  }

  void ImportParsed (const TParsedState& state)
  {
    Cells.Clear ();
    Territories.clear ();
    Tails.clear ();
    PlayersPrevPt = std::move (PlayersPt);
    PlayersPt.clear ();
    IsAlive.fill (false);
    for (char id = 1; id < 8; ++id) {
      IsAlive[id] = state.Players[id].Alive;
      if (!IsAlive[id]) {
        continue;
      }
      PlayersPt[id] = {state.Players[id].Cell.X, state.Players[id].Cell.Y};
    }
    if (PlayersPrevPt.empty ()) {
      PlayersPrevPt = PlayersPt;
    }
    for (char id = 1; id < 8; ++id) {
      if (!IsAlive[id]) {
        continue;
      }
      auto& terr = Territories[id];
      for (const auto& pt : state.Players[id].TerrCells) {
        Cells.At (pt.X, pt.Y) = id;
        terr.push_back (pt.X + WIDTH * pt.Y);
      }
    }
    for (char id = 1; id < 8; ++id) {
      if (!IsAlive[id]) {
        continue;
      }
      auto& tail = Tails[id];
      for (const auto& pt : state.Players[id].LineCells) {
        Cells.At (pt.X, pt.Y) = id + 8;
        tail.push_back (pt.X + WIDTH * pt.Y);
      }
    }
    for (const auto& bonus : state.Bonuses) {
      Cells.At (bonus.Cell.X, bonus.Cell.Y) = bonus.TypeCode;
    }
    PrevX = X;
    PrevY = Y;
    PrevPt = Pt;
    X = state.Players[C_SELF].Cell.X;
    Y = state.Players[C_SELF].Cell.Y;
    Pt = X + WIDTH * Y;
    if (PrevPt < 0) {
      PrevX = X;
      PrevY = Y;
      PrevPt = Pt;
    }
    Direction = {state.Players[C_SELF].Direction};
    FillBorderPoints ();
    CurrentTick = state.CurrentTick;
  }

  void UpdateDistances (const TParsedState& state)
  {
    if (Levits.empty ()) {
      for (size_t playerId = 1; playerId < state.Players.size (); ++playerId) {
        Levits[playerId];
      }
    }
    std::vector<int> justDied;
    auto copiedCells = Cells;
    for (auto& [playerId, levit] : Levits) {
      auto& player = state.Players[playerId];
      if (player.Alive) {
        const auto pt = PlayersPrevPt[playerId];
        const auto temp = copiedCells.At (pt);
        copiedCells.At (pt) = playerId + C_TAIL_OFS;
        levit.Apply (copiedCells, player.Cell.X, player.Cell.Y, playerId + C_TAIL_OFS);
        copiedCells.At (pt) = temp;
      } else {
        justDied.push_back (playerId);
      }
    }
    for (auto id : justDied) {
      Levits.erase (id);
    }
  }

  bool AllowedDirection (int dx, int dy)
  {
    dx = dx > 0 ? 1 : dx < 0 ? -1 : 0;
    dy = dy > 0 ? 1 : dy < 0 ? -1 : 0;
    if (dx && dx == -Direction.Dx) {
      return false;
    }
    if (dy && dy == -Direction.Dy) {
      return false;
    }
    return true;
  }

  void UpdateSafety ()
  {
    SafetyDist.fill (WIDTH + HEIGHT + 1);
    SafetyEnemy.fill (C_SELF);
    for (auto& [playerId, levit] : Levits) {
      if (playerId == C_SELF || !IsAlive[playerId]) {
        continue;
      }
      const auto& edist = levit.Dist;
      for (size_t i = 0; i < WIDTH * HEIGHT; ++i) {
        if (edist[i] < SafetyDist[i]) {
          SafetyDist[i] = edist[i];
          SafetyEnemy[i] = playerId;
        }
      }
    }

    TailSafety = WIDTH + HEIGHT + 1;
    auto& tail = Tails[C_SELF];
    for (int pt : tail) {
      TailSafety = std::min (TailSafety, SafetyDist[pt]);
    }
  }

  mutable std::unordered_map<int, int> CachedSafety;
  int CurrentSafety () const
  {
    auto it = CachedSafety.find (CurrentTick);
    if (it != CachedSafety.end ()) {
      return it->second;
    }

    int safety = std::min (TailSafety, SafetyDist[Pt]);
    const int ticksLeft = MaxTick - CurrentTick + 1;
    if (ticksLeft > -1) { //если меньше 0 - у нас бага и лучше не высовываться
      safety = std::min (safety, ticksLeft * BaseSpeed / CellSize);
    }

    CachedSafety[CurrentTick] = safety;
    return safety;
  }

  template <typename T> int Safety (const T& points) const
  {
    int safety = CurrentSafety ();
    for (int pt : points) {
      safety = std::min (safety, SafetyDist[pt]);
    }
    return safety;
  }
};

class TPlan
{
public:
  virtual ~TPlan () = default;

  virtual const std::string& Name () const = 0;

  virtual int Next (TPlayerState&)
  {
    return PopNextPoint ();
  }

  int PopNextPoint ()
  {
    if (Points.empty ()) {
      return -1;
    }
    const int pt = Points.front ();
    Points.pop_front ();
    return pt;
  }

  int Safety (TPlayerState& player) const
  {
    return player.Safety (Points);
  }

  void DumpPoints (TPlayerState& player) const
  {
    std::cerr << PID << "start at " << player.X << ", " << player.Y << " (prev was " << player.PrevX
              << ", " << player.PrevY << ")\npoints:";
    for (int pt : Points) {
      std::cerr << (pt % WIDTH) << "," << (pt / WIDTH) << " ";
    }
    std::cerr << std::endl;
  }

  bool Interrupted = false;
  float ExpectedProfit = 0;
  std::deque<int> Points;
};

class TPlanFactory
{
public:
  virtual ~TPlanFactory () = default;
  virtual std::unique_ptr<TPlan> BuildPlan (TPlayerState&) = 0;
  virtual const std::string& Name () const = 0;

  std::deque<int> BuildPathToPoint (int dst, const TLevit& levit) const
  {
    std::deque<int> plan;
    const auto& vprev = levit.Prev;
    int pt = dst;
    while (pt != -1) {
      plan.push_front (pt);
      pt = vprev[pt];
    }
    return plan;
  }

  std::deque<int> BuildPathToPoint (int dst, const TPlayerState& player)
  {
    const auto& levits = player.Levits.at (C_SELF);
    std::deque<int> plan = BuildPathToPoint (dst, levits);
    if (plan.front () == player.Pt) {
      plan.pop_front ();
    } else {
      const auto& vprev = levits.Prev;
      std::cerr << PID << __PRETTY_FUNCTION__ << "src=" << player.X << ", " << player.Y
                << "(prev=" << vprev[player.Pt] << ")"
                << " dst=" << dst % WIDTH << ", " << dst / WIDTH << "(prev=" << vprev[dst] << ")"
                << "\nplan:";
      for (int pt : plan) {
        std::cerr << " " << pt % WIDTH << "," << pt / WIDTH << "(prev=" << vprev[pt] << ")";
      }
      std::cerr << "\nfield:\n";
      player.Cells.DebugPrint (std::cerr);
      std::cerr << std::endl;
    }
    return plan;
  }
};

// базовая стратегия - нарезать прямоугольники
#define DEBUG_ESTIMATE
class TRandomRectPlanFactory : public virtual TPlanFactory
{
public:
  class TPlanImpl : public virtual TPlan
  {
  public:
    int Next (TPlayerState& player) // override
    {
      if (!Points.empty ()) {
        const int pt = Points.back ();
        if (player.Cells.At (pt) != C_SELF) {
          std::cerr << PID << Name () << " plan aborted: point " << pt << " lost to enemy\n";
          Points.clear ();
        }
      }
      return PopNextPoint ();
    }

    const std::string& Name () const
    {
      return G_PLAN_NAME_RANDOM_RECT;
    }
  };

  std::vector<std::pair<int, int>> InitCorners (int x1, int y1, bool swap, TPlayerState& player) const
  {
    const int x0 = player.X;
    const int y0 = player.Y;
    if (x0 == x1 || y0 == y1) {
      return {};
    }
    std::vector<std::pair<int, int>> corners;
    corners.emplace_back (x0, y1);
    corners.emplace_back (x1, y1);
    corners.emplace_back (x1, y0);
    if (swap) {
      std::swap (corners[0], corners[2]);
    }
    corners.emplace_back (x0, y0);
    return corners;
  }

  std::vector<std::pair<int, int>>
  CornersToPath (int x0, int y0, const std::vector<std::pair<int, int>>& corners) const
  {
    std::vector<std::pair<int, int>> path;
    int x = x0;
    int y = y0;
    for (const auto [dstx, dsty] : corners) {
      const int dx = dstx == x ? 0 : dstx > x ? 1 : -1;
      const int dy = dsty == y ? 0 : dsty > y ? 1 : -1;
      assert (std::abs (dx) + std::abs (dy) == 1);
      for (;;) {
        x += dx;
        y += dy;
        path.emplace_back (x, y);
        if (x == dstx && y == dsty) {
          break;
        }
      }
    }
    return path;
  }

  void CutPath (std::vector<std::pair<int, int>>& path, TPlayerState& player) const
  {
    if (path.empty ()) {
      return;
    }

    const auto [firstX, firstY] = path.front ();
    // двигаться на PrevX, PrevY нельзя
    if (firstX == player.PrevX && firstY == player.PrevY) {
      path.clear ();
      return;
    }

    const auto& cells = player.Cells;
    bool wasOutside = false;
    for (size_t i = 0; i < path.size (); ++i) {
      const auto [x, y] = path[i];
      assert (x >= 0);
      assert (x < WIDTH);
      assert (y >= 0);
      assert (y < HEIGHT);
      const int pt = x + y * WIDTH;
      const char c = cells.At (pt);
      if (c == C_SELF_TAIL) {
        // пересекать свой хвост нельзя
        path.clear ();
        break;
      } else if (c != C_SELF) {
        wasOutside = true;
      } else /*(c == C_SELF)*/ {
        // продолжать хвост за свою территорию не будем
        if (wasOutside) {
          path.resize (i + 1);
          break;
        }
      }

      if (i < 3 || c == C_SELF) {
        continue;
      }

      // мы не на своей территории и путь уже хотя бы 3 точки
      auto neighbours = cells.GetPointNeighbours (pt);
      int chosenNb = -1;
      for (int nb : neighbours) {
        if (cells.At (nb) != C_SELF) {
          continue;
        }
        const int nbx = nb % WIDTH;
        const int nby = nb / WIDTH;
        if (nbx == path[i - 1].first && nby == path[i - 1].second) {
          continue;
        }
        chosenNb = nb;
        break;
      }
      if (chosenNb != -1) {
        path.resize (i + 1);
        path.emplace_back (chosenNb % WIDTH, chosenNb / WIDTH);
        break;
      }
    }
    if (!wasOutside) {
      path.clear ();
    }
  }

#ifdef DEBUG_ESTIMATE
  mutable std::string ProfitExplained;
#endif // defined DEBUG_ESTIMATE
  float EstimatePathProfit (const std::vector<std::pair<int, int>>& path, TPlayerState& player, float ownCostBase) const
  {
#ifdef DEBUG_ESTIMATE
    std::stringstream ss;
#endif // defined DEBUG_ESTIMATE
    int counts[256];
    std::fill (std::begin (counts), std::end (counts), 0);

    auto copiedCells = player.Cells;
    const auto& currentCells = player.Cells;
    auto [minx, miny] = path[0];
    auto [maxx, maxy] = path[0];
    for (auto [x, y] : path) {
      if (copiedCells.At (x, y) == C_SELF) {
        counts[C_SELF] += 1;
      } else {
        copiedCells.At (x, y) = C_SELF_TAIL;
      }
      minx = std::min (x, minx);
      miny = std::min (y, miny);
      maxx = std::max (x, maxx);
      maxy = std::max (y, maxy);
    }
    if (counts[C_SELF] == (int)path.size ()) {
      return 0;
    }
#ifdef DEBUG_ESTIMATE
    ss << "len=" << path.size () << " #C_SELF=" << counts[C_SELF];
#endif // defined DEBUG_ESTIMATE
    const auto mask = copiedCells.FillMask (C_SELF, C_SELF_TAIL);
    int totalChanged = 0;
    for (size_t pt = 0; pt < mask.size (); ++pt) {
      if (mask[pt] && currentCells.At (pt) != C_SELF) {
        counts[int(currentCells.At (pt))] += 1;
        ++totalChanged;
      }
    }
    if (!totalChanged) {
      return 0;
    }

#ifdef DEBUG_ESTIMATE
    ss << " {";
    for (int i = 0; i < 256; ++i) {
      if (counts[i]) {
        ss << (i ? " #" : "#") << i << ":" << counts[i];
      }
    }
    ss << "}";
#endif // defined DEBUG_ESTIMATE
    float profit = WEIGHT_FREE * counts[0];
    for (int i = 1; i <= 6; ++i) {
      profit += WEIGHT_ENEMY * counts[i];
      profit += WEIGHT_ENEMY_TAIL * counts[i + C_TAIL_OFS];
    }
    profit += counts[C_SELF_TAIL] * WEIGHT_FREE;
    assert (ownCostBase < 0);
    profit += (counts[C_SELF] - 1) * ownCostBase;
    profit += counts[C_SLOW] * WEIGHT_SLOW + counts[C_NITRO] * WEIGHT_FAST + counts[C_SAW] * WEIGHT_SAW;

    float sizePenalty = std::pow ((float)path.size () + player.Tails[C_SELF].size (), .15f);
#if 0
    float ratio = (float)std::max (maxx - minx, 1) / std::max (maxy - miny, 1);
    if (ratio < 1) {
      ratio = 1. / ratio;
    }
    if (ratio > 2) {
      sizePenalty *= std::pow (ratio, .2f);
    }
#endif

    const auto [endpointX, endpointY] = path.back ();
    const float endpointWeight = EstimateEndpointProfit (endpointX, endpointY, player);

#ifdef DEBUG_ESTIMATE
    ss << "\tendpointWeight=" << endpointWeight;
#endif // defined DEBUG_ESTIMATE
    profit += endpointWeight;

#ifdef DEBUG_ESTIMATE
    ss << "\traw_profi=" << profit
#if 0
      << " ratio=" << ratio
#endif
       << " sizePenalty=" << sizePenalty << "\n";
#endif // defined DEBUG_ESTIMATE
    if (profit > 0) {
      profit /= sizePenalty;
    } else {
      profit *= sizePenalty;
    }

#ifdef DEBUG_ESTIMATE
    ProfitExplained = ss.str ();
#endif // defined DEBUG_ESTIMATE
    return profit;
  }

  float EstimateEndpointProfit (const int ex, const int ey, TPlayerState& player) const
  {
    const auto& cells = player.Cells;
    const auto& sdist = player.SafetyDist;
    float endpointWeight = 0;
    for (int x = ex - RANDOM_RECT_PLAN_ENDPOINT_WEIGHT_SCAN_RANGE, xmax = ex + RANDOM_RECT_PLAN_ENDPOINT_WEIGHT_SCAN_RANGE;
         x < xmax; ++x) {
      for (int y = ey - RANDOM_RECT_PLAN_ENDPOINT_WEIGHT_SCAN_RANGE, ymax = ey + RANDOM_RECT_PLAN_ENDPOINT_WEIGHT_SCAN_RANGE;
           y < ymax; ++y) {
        if (x < 0 || y < 0 || x >= WIDTH || y >= HEIGHT) {
          endpointWeight -= 2;
          continue;
        }
        const auto c = cells.At (x, y);
        float pw = 0;
        if (c == 0) {
          pw = 1;
        } else if (c == C_SELF || c == C_SELF_TAIL || c == C_SLOW) {
          pw = -1;
        } else if (c >= 1 && c <= 6) {
          pw = 1;
        } else if (c >= 1 + C_TAIL_OFS && c <= 6 + C_TAIL_OFS) {
          pw = 1.5;
        } else if (c == C_NITRO) {
          pw = 2.5;
        } else if (c == C_SAW) {
          pw = 5;
        }
        endpointWeight += pw * sdist[x + WIDTH * y];
      }
    }
    return endpointWeight * ENDPOINT_WEIGHT_CORRECTION_DISTRIBUTION (G_RNG) /
           std::pow (RANDOM_RECT_PLAN_ENDPOINT_WEIGHT_SCAN_RANGE, 2);
  }

  void TryGeneratePath (int x1, int y1, bool swap, float ownCost, TPlayerState& player)
  {
    auto corners = InitCorners (x1, y1, swap, player);
    auto path = CornersToPath (player.X, player.Y, corners);
    CutPath (path, player);
    if (path.empty () || PathSafety (path, player) <= (int)path.size () + 1) {
      return;
    }
    Path = std::move (path);
    EstimatedProfit = EstimatePathProfit (Path, player, ownCost);
  }

  int PathSafety (const std::vector<std::pair<int, int>>& path, TPlayerState& player) const
  {
    int safety = player.CurrentSafety ();
    const int width = WIDTH;
    auto& sdist = player.SafetyDist;
    for (auto [x, y] : path) {
      const int pt = x + y * width;
      if (sdist[pt] < safety) {
        safety = sdist[pt];
      }
    }
    return safety;
  }

  std::vector<std::pair<int, int>> Path;
  float EstimatedProfit;

  bool ChoosePath (TPlayerState& player)
  {
    const auto started = std::chrono::steady_clock::now ();
    const auto maxTime = started + MAX_CHOOSE_PATH_TIME;
    std::vector<std::pair<int, int>> path;
    float bestEstimatedProfit = PROFIT_BIAS;
    int count = 0;
    const float ownCost = OWN_COST_DIST (G_RNG);
    std::uniform_int_distribution<int> ptDist (0, WIDTH * HEIGHT - 1);
    std::vector<bool> mask;
    mask.assign (WIDTH * HEIGHT, true);
    for (int numFree = (int)mask.size (); numFree > 0 && maxTime > std::chrono::steady_clock::now (); --numFree) {
      int pt = ptDist (G_RNG);
      while (!mask[pt]) {
        pt = (pt + 1) % mask.size ();
      }
      mask[pt] = false;

      const int x = pt % WIDTH;
      const int y = pt / WIDTH;
      for (int swap = 0; swap < 2; ++swap) {
        EstimatedProfit = PROFIT_BIAS;
        Path.clear ();
        TryGeneratePath (x, y, !!swap, ownCost, player);
        ++count;
        if (bestEstimatedProfit >= EstimatedProfit) {
          continue;
        }
        std::cerr << PID << __PRETTY_FUNCTION__ << " new path from " << player.X << "," << player.Y << ":\t";
        for (size_t i = 0; i < Path.size (); ++i) {
          std::cerr << (i ? " " : "") << Path[i].first << "," << Path[i].second;
        }
        std::cerr << " profit=" << EstimatedProfit << " was " << bestEstimatedProfit
                  << " safety=" << PathSafety (Path, player) << " (current "
                  << player.SafetyDist[player.Pt] << ") len=" << Path.size ()
                  << "\n"
#ifdef DEBUG_ESTIMATE
                     "\tprofit explained: "
                  << ProfitExplained
#endif
          ;

        bestEstimatedProfit = EstimatedProfit;
        path = std::move (Path);
      }
    }
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::steady_clock::now () - started)
                .count ();
    std::cerr << PID << __PRETTY_FUNCTION__ << " done " << count << " attempts in " << dt << "ms\n";

    if (bestEstimatedProfit == PROFIT_BIAS) {
      Path.clear ();
      return false;
    }

    EstimatedProfit = bestEstimatedProfit;
    Path = std::move (path);

    return true;
  }

  std::unique_ptr<TPlan> BuildPlan (TPlayerState& player) // override
  {
    if (!player.Cells.HaveNotOwnedNeighbours (player.Pt, C_SELF) &&
        std::uniform_real_distribution<float> (0, 1) (G_RNG) < .75) {
      return {};
    }

    auto plan = std::make_unique<TPlanImpl> ();
    if (ChoosePath (player)) {
      plan->ExpectedProfit = EstimatedProfit;
      for (auto [x, y] : Path) {
        plan->Points.push_back (x + y * WIDTH);
      }
      if (player.Cells.At (plan->Points.back ()) != C_SELF) {
        const int pt = plan->Points.back ();
        const int x = pt % WIDTH;
        const int y = pt / WIDTH;
        std::cerr << PID << "ERROR: " << __PRETTY_FUNCTION__ << " plan ends in " << x << ", " << y
                  << " which is not in our territory!\n";
        return {};
      }
      return plan;
    }
    return {};
  }

  const std::string& Name () const
  {
    return G_PLAN_NAME_RANDOM_RECT;
  }

  const std::chrono::milliseconds MAX_CHOOSE_PATH_TIME = 480ms;
  const int RANDOM_RECT_PLAN_ENDPOINT_WEIGHT_SCAN_RANGE = 16;

  const float PROFIT_BIAS = 0;

  const float WEIGHT_FREE = 1;
  const float WEIGHT_ENEMY = 5;
  const float WEIGHT_ENEMY_TAIL = 200;
  const float WEIGHT_SLOW = -100;
  const float WEIGHT_FAST = 100;
  const float WEIGHT_SAW = 500;

  mutable std::uniform_real_distribution<float> ENDPOINT_WEIGHT_CORRECTION_DISTRIBUTION{.005, .01};
  mutable std::uniform_real_distribution<float> OWN_COST_DIST{-2.75, -3};
};

// паникуем - пытаемся попасть куда-нибудь на границе своих владений
class TEmergencyPlanFactory : public virtual TPlanFactory
{
public:
  class TPlanImpl : public virtual TPlan
  {
  public:
    const std::string& Name () const
    {
      return G_PLAN_NAME_EMERGENCY;
    }
  };

  void CutOwnedTail (std::deque<int>& points, const TCells& cells) const
  {
    while (points.size () > 1 && cells.At (points.back ()) == C_SELF) {
      const int back = points.back ();
      points.pop_back ();
      if (cells.At (points.back ()) == C_SELF) {
        continue;
      } else {
        points.push_back (back);
        break;
      }
    }
  }

  std::vector<int> GetOuterBorderPoints (TPlayerState& player)
  {
    const auto& cells = player.Cells;
    const auto& border = player.BorderPoints;
    std::vector<int> outer;
    std::vector<bool> mask;
    mask.assign (cells.Cells.size (), false);
    for (int pt : border) {
      auto neighbours = cells.GetPointNeighbours (pt);
      for (int nb : neighbours) {
        if (cells.At (nb) != C_SELF && cells.At (nb) != C_SELF_TAIL) {
          mask[nb] = true;
        }
      }
    }
    for (int i = 0; i < (int)mask.size (); ++i) {
      if (mask[i]) {
        outer.push_back (i);
      }
    }
    return outer;
  }

  int GetEmergencyPointWeight (int x1, int y1, const TPlayerState& player)
  {
    const auto& cells = player.Cells;
    const auto& sdist = player.SafetyDist;
    const auto width = WIDTH;
    float weight = 0;
    for (int y = y1 - EMERGENCY_PLAN_POINT_WEIGHT_RANGE; y < y1 + EMERGENCY_PLAN_POINT_WEIGHT_RANGE; ++y) {
      for (int x = x1 - EMERGENCY_PLAN_POINT_WEIGHT_RANGE; x < x1 + EMERGENCY_PLAN_POINT_WEIGHT_RANGE; ++x) {
        if (x < 0 || y < 0 || x >= WIDTH || y >= HEIGHT) {
          weight -= 2;
          continue;
        }
        const auto c = cells.At (x, y);
        float pw = 0;
        if (c == C_SELF || c == C_SELF_TAIL || c == C_SLOW) {
          pw = -1;
        } else if (c >= 1 && c <= 6) {
          pw = 1;
        } else if (c >= 1 + C_TAIL_OFS && c <= 6 + C_TAIL_OFS) {
          pw = 1.5;
        } else if (c == C_NITRO) {
          pw = 2.5;
        } else if (c == C_SAW) {
          pw = 5;
        }
        weight += pw > 0 ? pw * sdist[x + width * y] : pw;
      }
    }
    return weight * std::uniform_real_distribution<float> (0.05, 0.2) (G_RNG);
  }

  std::pair<std::pair<int, float>, std::deque<int>>
  BuildPlanForDestinations (const TPlayerState& player, const int* dstPoints, size_t dstCount)
  {
    auto& cells = player.Cells;
    const int currPt = player.Pt;
    const bool startedOutside = (cells.At (currPt) != C_SELF) || !ALLOW_EMERGENCY_TO_OUTER;

    std::pair<int, float> bestProfit = {0, -1000};
    std::deque<int> bestPlan;

    auto manhattan = [w = WIDTH](int pta, int ptb) {
      using std::abs;
      return abs (pta % w - ptb % w) + abs (pta / w - ptb / w);
    };

    for (size_t idx = 0; idx < dstCount; ++idx) {
      const int dst = dstPoints[idx];
      auto points = BuildPathToPoint (dst, player);
      if (points.front () == player.PrevPt) {
        continue;
      }
      if (startedOutside) {
        CutOwnedTail (points, cells);
      }
      if (points.empty ()) {
        continue;
      }

      const int ept = points.back ();
      const int ex = ept % WIDTH;
      const int ey = ept / WIDTH;
      const int safety = player.Safety (points);
      float weight = GetEmergencyPointWeight (ex, ey, player);
      if (weight > 0) {
        weight /= points.size () * manhattan (currPt, ept);
      } else {
        weight *= points.size () * manhattan (currPt, ept);
      }

      const auto profit = std::make_pair (safety, weight);
      if (profit > bestProfit) {
        bestProfit = profit;
        bestPlan = std::move (points);
      }
    }
    return {bestProfit, bestPlan};
  }

  const bool ALLOW_EMERGENCY_TO_OUTER = true;
  std::unique_ptr<TPlan> BuildPlan (TPlayerState& player) // override
  {
    auto& cells = player.Cells;
    const int currPt = player.Pt;
    const bool startedOutside = (cells.At (currPt) != C_SELF) || !ALLOW_EMERGENCY_TO_OUTER;

    std::cerr << __PRETTY_FUNCTION__ << " startedOutside=" << std::boolalpha << startedOutside << "\n";
    std::vector<int> dstPoints;
    if (startedOutside) {
      for (int bpt : player.BorderPoints) {
        if (cells.HaveNotOwnedNeighbours (bpt, C_SELF)) {
          dstPoints.push_back (bpt);
        }
      }
    } else {
      dstPoints = GetOuterBorderPoints (player);
    }

    auto manhattan = [](int pta, int ptb, int w) {
      using std::abs;
      return abs (pta % w - ptb % w) + abs (pta / w - ptb / w);
    };
    std::sort (dstPoints.begin (), dstPoints.end (), [&manhattan, currPt, w = WIDTH](int a, int b) {
      return manhattan (a, currPt, w) < manhattan (b, currPt, w);
    });

    const size_t mid = dstPoints.size () / 3;
    auto [bestProfit, bestPlan] = BuildPlanForDestinations (player, &dstPoints[0], mid);
    if (bestPlan.empty ()) {
      auto [prof, plan] = BuildPlanForDestinations (player, &dstPoints[mid], dstPoints.size () - mid);
      bestProfit = prof;
      bestPlan = plan;
    }

    if (bestPlan.empty ()) {
      return {};
    }

    auto plan = std::make_unique<TPlanImpl> ();
    plan->Points = std::move (bestPlan);
    plan->ExpectedProfit = bestProfit.second;
    return plan;
  }

  const std::string& Name () const
  {
    return G_PLAN_NAME_EMERGENCY;
  }

  const int EMERGENCY_PLAN_POINT_WEIGHT_RANGE = 16;
};

class TFieldPlanFactory : public virtual TPlanFactory
{
public:
  class TPlanImpl : public virtual TPlan
  {
  public:
    const std::string& Name () const
    {
      return G_PLAN_NAME_FIELD;
    }
  };

  const int SCAN_RANGE = 24;

  const float F_MIN = -std::numeric_limits<float>::max ();
  const float F_OUTSIDE = -.01;

  float EstimatePointScore (int pt, TPlayerState& player)
  {
    const auto& cells = player.Cells;
    const char cell = cells.At (pt);
    if (cell == C_SELF_TAIL) {
      return F_MIN;
    }
    const float ptSafety = std::min (player.CurrentSafety () - 1, player.SafetyDist[pt]);
    if (ptSafety < 2) {
      return F_MIN;
    }
    const int width = WIDTH;
    const int ptx = pt % width;
    const int pty = pt / width;
    if (cells.At (pt) != C_SELF) {
      // сможем ли мы вернуться, если пойдем в данную точку
      auto cellsCopy = cells;
      cellsCopy.At (player.Pt) = C_SELF_TAIL;
      TLevit lev;
      lev.Apply (cellsCopy, ptx, pty, C_SELF_TAIL);
      bool canReturn = false;
      const auto& terr = player.Territories[C_SELF];
      for (int tpt : terr) {
        if (lev.Dist[tpt] < ptSafety && lev.Prev[tpt] != -1 && lev.Prev[tpt] != player.Pt) {
          canReturn = true;
          break;
        }
      }
      if (!canReturn) {
        return F_MIN;
      }
    }
    float score = 1000 / ptSafety;
    if (cells.At (pt) == C_SELF && cells.At (player.Pt) != C_SELF) {
      score += 100;
    }
    for (int y = pty - SCAN_RANGE; y <= pty + SCAN_RANGE; ++y) {
      for (int x = pty - SCAN_RANGE; x <= pty + SCAN_RANGE; ++x) {
        const int mdist = std::abs (x - ptx) + std::abs (y - pty) + 1;
        auto fdist = [mdist](float v) { return v / std::pow (mdist, 1.f / 2); };

        if (!cells.Valid (x, y)) {
          score += fdist (F_OUTSIDE);
          continue;
        }

        float cellBaseScore = 0;
        const auto cell = cells.At (x, y);
        if (cell == 0) {
          cellBaseScore = 1;
        } else if (cell >= 1 && cell <= 6) {
          cellBaseScore = 5;
        } else if (cell >= 1 + C_TAIL_OFS && cell <= 6 + C_TAIL_OFS) {
          cellBaseScore = 50;
        } else if (cell == C_SELF) {
          cellBaseScore = -150;
        } else if (cell == C_SELF_TAIL) {
          cellBaseScore = -2000;
        } else if (cell == C_SAW) {
          cellBaseScore = 500;
        } else if (cell == C_NITRO) {
          cellBaseScore = 50;
        } else if (cell == C_SLOW) {
          cellBaseScore = -50;
        }
        score += fdist (cellBaseScore);
      }
    }

    std::uniform_real_distribution<float> dist (0.95, 1);
    return score * dist (G_RNG) + ptSafety * 200;
  }

  std::unique_ptr<TPlan> BuildPlan (TPlayerState& player) // override
  {
    auto nb = player.Cells.GetPointNeighbours (player.Pt);
    std::shuffle (nb.begin (), nb.end (), std::mt19937 (G_RNG ()));
    float score = F_MIN;
    int best = -1;
    for (int pt : nb) {
      if (pt == player.PrevPt) {
        continue;
      }
      const float ps = EstimatePointScore (pt, player);
      if (ps > score) {
        score = ps;
        best = pt;
      }
    }
    if (best == -1) {
      return {};
    }
    auto plan = std::make_unique<TPlanImpl> ();
    plan->Points.push_back (best);
    return plan;
  }

  const std::string& Name () const
  {
    return G_PLAN_NAME_FIELD;
  }
};

// последний шанс хоть как-то пошевелиться напоследок
class TRandomPlanFactory : public virtual TPlanFactory
{
public:
  class TPlanImpl : public virtual TPlan
  {
  public:
    const std::string& Name () const
    {
      return G_PLAN_NAME_RANDOM;
    }
  };

  std::unique_ptr<TPlan> BuildPlan (TPlayerState& player) // override
  {
    std::cerr << PID << __PRETTY_FUNCTION__ << "\n";
    auto plan = std::make_unique<TPlanImpl> ();
    auto neighbours = player.Cells.GetPointNeighbours (player.Pt);
    assert (!neighbours.empty ());
    std::cerr << PID << neighbours.size () << " neighbours before cleanup\n";
    int dst = neighbours.front ();
    for (auto it = neighbours.begin (); it != neighbours.end ();) {
      if (player.Cells.At (*it) == C_SELF_TAIL || *it == player.PrevPt) {
        it = neighbours.erase (it);
      } else {
        ++it;
      }
    }
    std::cerr << PID << neighbours.size () << " neighbours after cleanup\n";
    if (!neighbours.empty ()) {
      std::uniform_int_distribution<int> dist (0, neighbours.size () - 1);
      dst = neighbours[dist (G_RNG)];
    }
    std::cerr << PID << "random transition: " << std::dec << player.Pt << " -> " << dst << "\n";
    plan->Points.push_back (dst);
    return plan;
  }

  const std::string& Name () const
  {
    return G_PLAN_NAME_RANDOM;
  }
};

class TPlayer : public TPlayerState
{
public:
  std::unique_ptr<TPlanFactory> RectPlanFactory;
  std::unique_ptr<TPlanFactory> EmergencyPlanFactory;
  std::unique_ptr<TPlanFactory> RandomPlanFactory;
  std::unique_ptr<TPlanFactory> FieldPlanFactory;
  std::unique_ptr<TPlan> ActivePlan;

  void InitPlanFactories ()
  {
    RectPlanFactory = std::make_unique<TRandomRectPlanFactory> ();
    EmergencyPlanFactory = std::make_unique<TEmergencyPlanFactory> ();
    RandomPlanFactory = std::make_unique<TRandomPlanFactory> ();
    FieldPlanFactory = std::make_unique<TFieldPlanFactory> ();
  }

  std::string LastMovePlanName;
  int GetNextPoint ()
  {
    std::cerr << PID << __PRETTY_FUNCTION__;
    if (!ActivePlan) {
      std::cerr << ", no active plan\n";
      return -1;
    }
    std::cerr << ", active plan: " << ActivePlan->Name ();
    const int point = ActivePlan->Next (*this);
    std::cerr << "\treturn " << point % WIDTH << "," << point / WIDTH << "\n";
    if (-1 == point) {
      std::cerr << PID << "FINISHED plan: " << ActivePlan->Name () << "\n";
      ActivePlan.reset ();
    } else {
      LastMovePlanName = ActivePlan->Name ();
    }
    return point;
  }

  void RandomRectPlanUpdate ()
  {
    if (!ActivePlan || ActivePlan->Name () != G_PLAN_NAME_RANDOM_RECT) {
      return;
    }
    auto replacement = RectPlanFactory->BuildPlan (*this);
    if (replacement && replacement->ExpectedProfit > ActivePlan->ExpectedProfit) {
      std::cerr << PID << "INFO: randomly updated plan, profit change "
                << ActivePlan->ExpectedProfit << " -> " << replacement->ExpectedProfit << "\n";
      ActivePlan = std::move (replacement);
    }
  }

  void UpdateFromState (const TParsedState& state)
  {
    ImportParsed (state);
    UpdateDistances (state);
    UpdateSafety ();
  }

  std::string Think (const TParsedState& state)
  {
    UpdateFromState (state);
    return ChooseAction ();
  }

  std::string ChooseAction ()
  {
    RandomRectPlanUpdate ();

    int point = GetNextPoint ();
    if (-1 == point) {
      assert (!ActivePlan);

      std::uniform_real_distribution<float> dist (0, 1);
      if (!ActivePlan && dist (G_RNG) < 0.5) {
        ActivePlan = FieldPlanFactory->BuildPlan (*this);
      }
      if (!ActivePlan) {
        ActivePlan = RectPlanFactory->BuildPlan (*this);
      }
      if (!ActivePlan) {
        ActivePlan = EmergencyPlanFactory->BuildPlan (*this);
      }
      if (!ActivePlan) {
        ActivePlan = RandomPlanFactory->BuildPlan (*this);
      }
      assert (ActivePlan);

      std::cerr << PID << "new plan: " << ActivePlan->Name () << "\n";
      point = GetNextPoint ();
    }
    assert (point != -1);

    const int dx = point % WIDTH - X;
    const int dy = point / WIDTH - Y;
    if (std::abs (dx) + std::abs (dy) != 1) {
      std::cerr << PID << "FATAL: invalid move planned: prev=" << PrevX << "," << PrevY
                << " curr=" << X << "," << Y << " planned=" << point % WIDTH << "," << point / WIDTH
                << " (delta=" << dx << "," << dy << ")" << std::endl;
    }
    // assert (std::abs (dx) + std::abs (dy) == 1);
    return TDirection{dx, dy}.ToString ();
  }
};

class TApp
{
public:
  void Init (int argc, char** argv)
  {
    ::srand (std::time (nullptr));
    for (int i = 1; i < argc; ++i) {
      if (!::strcmp (argv[i], "--silent")) {
        Silent = true;
      } else if (!::strcmp (argv[i], "--test")) {
        TestMode = true;
      }
    }
  }

  void Run ()
  {
    if (TestMode) {
      RunTests ();
    } else {
      RunGame ();
    }
  }

  void RunTests ()
  {
    TCells cells;
    cells.Reset ();

    for (int x = 0; x < 31; ++x) {
      for (int y = 0; y < 31; ++y) {
        if (x > 5 && y > 5 && x < 20 && y < 8) {
          cells.At (x, y) = 7;
        }
        if (x > 5 && y > 25 && x < 20 && y < 28) {
          cells.At (x, y) = 7;
        }
        if (x == 8 && y >= 8 && y <= 25) {
          cells.At (x, y) = 7;
        }
        if (x == 18 && y >= 8 && y <= 25) {
          cells.At (x, y) = C_SELF_TAIL;
        }
      }
    }

    cells.Fill (7, C_SELF_TAIL);
    cells.DebugPrint (std::cout);

    TLevit lev;
    int x0 = 3, y0 = 15 + 3;
    lev.Apply (cells, x0, y0, 7);

    int x1 = 15, y1 = 29;
    int dst = x1 + y1 * WIDTH;
    while (dst >= 0) {
      cells.At (dst) = 0x55;
      dst = lev.Prev[dst];
    }
    cells.At (x0, y0) += 0x22;
    cells.At (x1, y1) += 0x22;
    cells.DebugPrint (std::cout);
  }

  void RunGame ()
  {
    std::string stateStr;
    TJson command;
    TParsedState parsedState;
    TPlayer player;

    std::ofstream devNull{"/dev/null"};
    if (Silent) {
      std::cerr.rdbuf (devNull.rdbuf ());
    }

#if AICUP_ENVIRONMENT
    std::ostringstream newCerr;
    std::cerr.rdbuf (newCerr.rdbuf ());
#endif

    player.InitPlanFactories ();
    while (true) {
      std::getline (std::cin, stateStr);
      // std::cerr << stateStr << std::endl;
      if (stateStr.empty ()) {
        break;
      }
      if (parsedState.Parse (stateStr)) {
        const auto beforeThink = std::chrono::high_resolution_clock::now ();
        command["command"] = player.Think (parsedState);
        const auto afterThink = std::chrono::high_resolution_clock::now ();
        TJson jdebug;
        jdebug["times"]["think"] =
          std::chrono::duration_cast<std::chrono::microseconds> (afterThink - beforeThink).count ();
        jdebug["self"]["x"] = player.X;
        jdebug["self"]["y"] = player.Y;
        jdebug["plan"] = player.LastMovePlanName;
        jdebug["safety"] = player.CurrentSafety ();

#if AICUP_ENVIRONMENT
        // std::cerr << jdebug.dump () << std::endl;
        // command["debug"] = newCerr.str ();
        command["debug"] = jdebug.dump ();
        newCerr.clear ();
#else
        command["debug"] = jdebug.dump ();

        std::cerr << command.dump () << std::endl;
#endif
        std::cout << command.dump () << std::endl;
      } else {
        std::cerr << stateStr << std::endl;
        player.Reset (parsedState);
      }
    }
  }

  bool Silent = false;
  bool TestMode = false;
};

int main (int argc, char** argv)
{
  TApp app;
  app.Init (argc, argv);
  app.Run ();
  return 0;
}
