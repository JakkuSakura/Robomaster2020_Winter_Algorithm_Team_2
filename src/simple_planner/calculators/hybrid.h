#if !defined(HYBRID_H)
#define HYBRID_H
#include <bits/stdc++.h>
#include "../graph.h"
#include "../thread_pool.h"
// #include "../safe_ptr.h"
namespace Hybrid
{
#include "a_star.h"

inline bool random(double p)
{
  return 1.0 * rand() / RAND_MAX < p;
}
using A_Star::State;
typedef std::vector<int> solution;
#define show_line_num() //std::cerr << __LINE__ << std::endl
#define show_vector(x) show_vector_2(x, __LINE__, #x)
template <typename T>
inline void show_vector_2(T x, int line_number, const char *s)
{
  std::cerr << s << " at line " << line_number << " has " << x.size() << " children: ";
  for (auto &&i : x)
  {
    std::cerr << i << ", ";
  }
  std::cerr << std::endl;
}
class Graph
{
  size_t max_species_population = 50;
  size_t elite_protection_number_per_species = 3;

  float same_speices_distance_threshold = 370;

  int generation_number = 10;

  double inherit_probability = 1;
  double swap_probability = 1;
  double reverse_probability = 1;
  double shuffle_probability = 1;
  double transform_probabilty = 1;
  double crossover_within_species_probability = 1;
  double crossover_over_species_probability = 1;

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
  std::vector<float> fitness(const std::vector<solution> &speices)
  {
    assert(speices.size() > 0);
    std::vector<float> scores;
    std::vector<float> fits;
    float average_time = 0;
    for (auto &&so : speices)
    {
      float time = consumed_time(so);
      scores.push_back(time);
      average_time += time;
    }
    average_time /= speices.size();

    for (size_t i = 0; i < speices.size(); i++)
    {
      float fit = average_time / scores[i];
      fits.push_back(fit);
    }

    std::vector<float> fits2 = fits;
    float non_elite_average_fitness = 0;
    sort(fits2.begin(), fits2.end(), std::greater<float>());
    for (size_t i = elite_protection_number_per_species; i < fits2.size(); i++)
    {
      non_elite_average_fitness += fits2[i];
    }
    non_elite_average_fitness /= speices.size();

    float projected_non_elite_species_population = std::max(0.01f, non_elite_average_fitness * (speices.size() - elite_protection_number_per_species));

    float adj = (max_species_population - elite_protection_number_per_species) / projected_non_elite_species_population;

    // ind * adj = adjusted_expect_survial_probability
    for (size_t i = 0; i < fits.size(); i++)
    {
      if (i < elite_protection_number_per_species)
        fits[i] = 1;
      else
        fits[i] *= adj;
    }

    return fits;
  }
  float consumed_time(const solution &result)
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
  solution to_solution(const std::vector<int> &order)
  {
    solution sol(order.size());
    for (size_t i = 0; i < order.size(); ++i)
    {
      sol[order[i]] = i;
    }
    return sol;
  }
  std::vector<int> to_order(const solution &a)
  {
    std::vector<int> ord(a.size());
    for (size_t i = 0; i < a.size(); ++i)
    {
      assert(a[i] >= 0 && a[i] < (int)a.size());

      ord[a[i]] = i;
    }
    return ord;
  }

  float genetic_distance(const solution &a, const solution &b)
  {
    assert(a.size() == b.size());
    std::vector<int> a2 = to_order(a), b2 = to_order(b);

    float dist = 0;
    for (size_t i = 0; i < a.size(); i++)
    {
      dist += abs(a2[i] - b2[i]);
    }

    return dist;
  }
  std::vector<solution> initial_generation()
  {
    A_Star::Graph graph(id1_, id2_);
    auto result = graph.calc(State());
    std::vector<solution> results;
    // results.push_back(result.path);
    for (size_t i = 0; i < 100; i++)
    {
      solution res;
      for (size_t i = 0; i < 36; i++)
      {
        res.push_back(i);
      }
      random_shuffle(++res.begin(), res.end(), [](int i) { return rand() % i; });
      results.push_back(res);
    }
    return results;
  }
  std::vector<std::vector<solution>> divide_species(std::vector<std::vector<solution>> &population, std::vector<solution> solutions)
  {
    for (size_t i = 0; i < solutions.size(); ++i)
    {
      bool found = false;
      for (std::vector<solution> &speices : population)
      {
        if (speices.size())
        {
          float distance = genetic_distance(solutions[i], speices[0]);
          if (distance < same_speices_distance_threshold)
          {
            speices.push_back(solutions[i]);
            found = true;

            break;
          }
        }
      }
      if (!found)
      {

        std::vector<solution> species;
        species.push_back(solutions[i]);
        population.push_back(species);
      }
    }
    return population;
  }
  std::vector<solution> crossover(const solution &s1, const solution &s2)
  {
    // if(s1.size() != s2.size())
    //   fprintf(stderr, "%lu vs %lu\n", s1.size(), s2.size());
    assert(s1.size() == s2.size());
    std::vector<solution> solutions;
    { // PMX
      int a = rand() % s1.size(), b = rand() % s1.size();
      if (a > b)
        std::swap(a, b);

      solution child1(s1.size(), -1), child2(s2.size(), -1);

      for (int i = a; i <= b; ++i)
      {
        child1[i] = s2[i];
        child2[i] = s1[i];
      }
      auto oper = [&](const solution &parent, solution &child) {
        for (int i = 0; i < (int)s1.size(); ++i)
        {
          if (a <= i && i <= b)
            continue;
          int now = parent[i];
          solution::const_iterator iter;
          while ((iter = std::find(child.begin(), child.end(), now), iter != child.end()))
          {
            int index = iter - child.begin();
            now = parent[index];
          }
          child[i] = now;
        }
      };
      oper(s1, child1);
      oper(s2, child2);
      solutions.push_back(child1);
      solutions.push_back(child2);
    }

    return solutions;
  }

  solution evolve(const std::vector<solution> gen)
  {

    std::vector<std::vector<solution>> population, next_population;
    divide_species(population, gen);

    std::cout << "Generation " << 0 << " ended up with " << gen.size() << " solutions in " << population.size() << " species, and the best solution comsumes " << consumed_time(pick_best(population)) << " units" << std::endl;

    ThreadPool pool(4);
    for (int i = 1; i <= generation_number; ++i)
    {
      std::vector<std::future<std::vector<solution>>> next_gen_pool;
      // new solutions
      for (const std::vector<solution> &species : population)
      {
        auto work = [&]() {
          std::vector<solution> next_gen;

          for (const solution &solu : species)
          {
            assert(solu.size() > 0);
            // inherit
            if (random(inherit_probability))
            {
              next_gen.push_back(solu);
            }

            // mutations
            if (random(swap_probability))
            {
              solution s = solu;
              int a = rand() % s.size(), b = rand() % s.size();
              std::swap(s[a], s[b]);
              assert(s.size() > 0);
              next_gen.push_back(s);
            }

            if (random(reverse_probability))
            {
              solution s = solu;
              int a = rand() % s.size(), b = rand() % s.size();
              if (a > b)
                std::swap(a, b);
              std::reverse(s.begin() + a, s.begin() + (b + 1));
              assert(s.size() > 0);
              next_gen.push_back(s);
            }

            if (random(shuffle_probability))
            {
              solution s = solu;
              int a = rand() % s.size(), b = rand() % s.size();
              if (a > b)
                std::swap(a, b);
              std::random_shuffle(s.begin() + a, s.begin() + (b + 1), [](int x) { return rand() % x; });
              assert(s.size() > 0);
              next_gen.push_back(s);
            }

            // if (random(transform_probabilty))
            // {
            // int a = rand() % solu.size(), b = rand() % solu.size(), c = rand() % solu.size();
            // not gonna implement it yet
            // }

            // crossover
            if (random(crossover_within_species_probability))
            {
              assert(species.size() > 0);
              solution s2 = species[rand() % species.size()];
              auto &&children = crossover(solu, s2);
              for (auto &&child : children)
              {
                assert(child.size());
                next_gen.push_back(child);
              }
            }

            if (random(crossover_over_species_probability))
            {
              assert(population.size() > 0);
              std::vector<solution> &species2 = population[rand() % population.size()];
              assert(species2.size() > 0);
              const solution &s2 = species2[rand() % species2.size()];
              assert(s2.size() > 0);
              auto &&children = crossover(solu, s2);
              for (auto &&child : children)
              {
                assert(child.size() > 0);
                next_gen.push_back(child);
              }
            }
          }
          return next_gen;
        };

        next_gen_pool.push_back(pool.enqueue(work));
      }

      for (auto &&future : next_gen_pool)
        divide_species(next_population, future.get());

      population.clear();

      for (auto &&species : next_population)
      {
        std::vector<solution> spe;
        auto fits = fitness(species);

        for (size_t i = 0; i < species.size(); i++)
        {
          if (random(fits[i]))
          {
            spe.push_back(species[i]);
          }
        }
        if (spe.size())
          population.push_back(spe);
      }
      next_population.clear();
      assert(population.size() > 0);

      int count = 0;
      for (auto &&species : population)
        count += species.size();
      std::cout << "Generation " << i << " ended up with " << count << " solutions in " << population.size() << " species, and the best solution comsumes " << consumed_time(pick_best(population)) << " units" << std::endl;
    }

    return pick_best(population);
  }
  solution pick_best(std::vector<std::vector<solution>> population)
  {
    solution best = population[0][0];
    for (auto &&species : population)
      for (auto &&sol : species)
        if (consumed_time(sol) < consumed_time(best))
          best = sol;
    return best;
  }
};
} // namespace Hybrid

Hybrid::solution calculate_path(const int *mat1, const int *mat2)
{
  using namespace Hybrid;
  srand(time(0));
  Graph graph(mat1, mat2);
  auto results = graph.initial_generation();
  solution best = graph.evolve(results);

  return best;
  // return results[rand() % results.size()];
}
#define DEBUG_DATA_SHOWING_ENABLED
void show_debug_data(const int *mat1, const int *mat2, const Hybrid::solution &result)
{
  using namespace Hybrid;

  std::cout << "Result: ";
  for (int i = 0; i < 36; i++)
  {
    std::cout << result[i] << " ";
  }
  std::cout << std::endl;
  Graph graph(mat1, mat2);
  std::cout << "Predicted consumed time: " << graph.consumed_time(result) << std::endl;
}
#endif // HYBRID_H
