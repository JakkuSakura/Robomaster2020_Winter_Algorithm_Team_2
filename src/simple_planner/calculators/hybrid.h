#if !defined(HYBRID_H)
#define HYBRID_H
#include <bits/stdc++.h>
#include "../graph.h"
namespace Hybrid
{
inline bool random(double p)
{
  return 1.0 * rand() / RAND_MAX < p;
}
const int STEP_VALUE = 100;
struct State
{
  std::bitset<64> visited;
  float x = 0, y = 0;
  float orientation = 0;
  float g = 0;
  std::vector<int> path;
  friend bool operator<(const State &lhs, const State &rhs)
  {
    return -lhs.g + lhs.path.size() * STEP_VALUE < -rhs.g + rhs.path.size() * STEP_VALUE;
  }
};

class Graph
{
  size_t soft_max_generation_population = 5000;
  size_t absolute_max_generation_population = 6000;
  double probability_in_between = .30;
  double probability_when_worse = .01;
  const int *id1_, *id2_;

protected:
  float dist(float x1, float y1, float x2, float y2)
  {
    return sqrt(p2(x1 - x2) + p2(y1 - y2));
  }

  float distance_in_degree(float alpha, float beta)
  {
    float phi = abs(beta - alpha);
    while (phi > 360)
      phi -= 360;
    float distance = phi > 180 ? 360 - phi : phi;
    return distance;
  }
  float p2(float x)
  {
    return x * x;
  }

public:
  Graph(const int *id1, const int *id2)
  {
    id1_ = id1;
    id2_ = id2;
  }
  float consumed_time(std::vector<int> result)
  {
    float g = 0;
    float nowx = 0, nowy = 0;
    float orientation = 0;
    for (int point : result)
    {
      float midx, midy;
      lookup(id1_, id2_, point, midx, midy);

      float endx, endy;
      lookup(id1_, id2_, pair(point), endx, endy);

      float mid_orientation = atan2f(midy - nowy, midx - nowx) * 180 / M_PI;
      float next_orientation = atan2f(endy - midy, endx - midx) * 180 / M_PI;

      g += 10 * dist(midx, midy, nowx, nowy) + distance_in_degree(orientation, mid_orientation) / 180.0 * 40 + distance_in_degree(mid_orientation, next_orientation) / 180.0 * 40;
      nowx = endx, nowy = endy, orientation = next_orientation;
    }
    return g;
  }
  std::vector<std::vector<int>> initial_generation()
  {
    std::multiset<State> current_generation;
    current_generation.insert(State());
    std::multiset<State> next_generation;

    for (size_t step = 0; step < 36; step++)
    {
      while (current_generation.size())
      {
        const State s = *current_generation.begin();
        current_generation.erase(current_generation.begin());

        int cnt = 0;
        for (size_t j = 0; j < 36; j++)
        {
          int i = j;
          if (s.visited[i % 36])
            continue;

          ++cnt;
          State s2;
          s2.visited = s.visited;
          s2.visited[i % 36] = 1;
          float midx, midy;
          lookup(id1_, id2_, i, midx, midy);       // original location
          lookup(id1_, id2_, pair(i), s2.x, s2.y); // teleport

          float degree = atan2f(midy - s.y, midx - s.x) * 180 / M_PI;
          s2.orientation = atan2f(s2.y - midy, s2.x - midx) * 180 / M_PI;

          s2.g = s.g + 10 * dist(midx, midy, s.x, s.y) + distance_in_degree(s.orientation, degree) / 180.0 * 40 + distance_in_degree(degree, s2.orientation) / 180.0 * 40;
          s2.path = s.path;
          s2.path.push_back(i);
          if (next_generation.size() < soft_max_generation_population)
          {
            next_generation.insert(s2);
          }
          else
          {
            auto best = next_generation.begin();
            auto worst = next_generation.rbegin();
            if (s2.g < best->g)
            {
              next_generation.insert(s2);
            }
            else if (s2.g < worst->g)
            {
              if (random(probability_in_between))
              {
                next_generation.insert(s2);
              }
            }
            else // worse
            {
              if (random(probability_when_worse))
              {
                next_generation.insert(s2);
              }
            }
          }
          while (next_generation.size() > absolute_max_generation_population)
          {
            next_generation.erase(--next_generation.end());
          }

        } // end for
      }   // end while
      current_generation.swap(next_generation);
    } // end for
    std::vector<std::vector<int>> results;
    for (auto &&state : current_generation)
    {
      results.push_back(state.path);
    }
    return results;
  }
};
} // namespace Hybrid

std::vector<int> calculate_path(const int *mat1, const int *mat2)
{
  using namespace Hybrid;
  srand(time(0));
  Graph graph(mat1, mat2);
  auto results = graph.initial_generation();
  // TODO adapt hybrid algorithm for stage two
  return results[0];
}
#define DEBUG_DATA_SHOWING_ENABLED
void show_debug_data(const int *mat1, const int *mat2, std::vector<int> result)
{
  using namespace Hybrid;

  std::cout << "Result: ";
  for (int i = 0; i < 36; i++)
  {
    std::cout << result[i] << " ";
  }
  std::cout << std::endl;
  Graph graph(mat1, mat2);
  std::cout << "Time: " << graph.consumed_time(result) << std::endl;
}
#endif // HYBRID_H
