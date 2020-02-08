#if !defined(HYBRID_H)
#define HYBRID_H
#define NDEBUG
#include <bits/stdc++.h>
#include "../graph.h"
#include "../thread_pool.h"

namespace Hybrid
{
#include "a_star.h"

inline bool random(double p)
{
  return 1.0 * rand() / RAND_MAX <= p;
}
using A_Star::State;
typedef std::vector<int> Solution;
#define show_line_num() std::cerr << __LINE__ << std::endl
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
class Species
{
  std::vector<Solution> solutions;

public:
  static int get_id()
  {
    static int id = 0;
    return ++id;
  }
  int species_id = -1;
  Species() {}
  Species(const std::vector<Solution> &s) : solutions(s)
  {
  }
  Species(const Species &s) : solutions(s.solutions), species_id(s.species_id)
  {
  }
  Species &operator=(const Species &s)
  {
    solutions = s.solutions;
    species_id = s.species_id;
    return *this;
  }
  void push_back(const Solution &s)
  {
    solutions.push_back(s);
  }
  void extend(const Species &s)
  {
    for (auto &&i : s)
    {
      solutions.push_back(i);
    }
  }
  size_t size() const
  {
    return solutions.size();
  }
  void erase(std::vector<Solution>::iterator begin, std::vector<Solution>::iterator end)
  {
    solutions.erase(begin, end);
  }
  void clear()
  {
    solutions.clear();
  }
  std::vector<Solution>::iterator begin()
  {
    return solutions.begin();
  }
  std::vector<Solution>::iterator end()
  {
    return solutions.end();
  }
  std::vector<Solution>::const_iterator begin() const
  {
    return solutions.begin();
  }
  std::vector<Solution>::const_iterator end() const
  {
    return solutions.end();
  }
  Solution &operator[](int index)
  {
    // return solutions.at(index);
    return solutions[index];
  }
  const Solution &operator[](int index) const
  {
    // return solutions.at(index);
    return solutions[index];
  }
};
typedef std::map<int, Species> Population;
class Graph
{
  size_t max_species_population = 10;
  size_t elite_protection_number_per_species = 2;

  float same_speices_distance_threshold = 10;

  int generation_number = 200;
  int thread_number = 4;

  double inherit_probability = 1;
  double swap_probability = 1;
  double reverse_probability = 1;
  double shuffle_probability = 1;
  double transform_probabilty = 1;
  double crossover_within_species_probability = 0.5;
  double crossover_over_species_probability = 0.01;

  const int *mat1, *mat2;

protected:
  float dist(float x1, float y1, float x2, float y2)
  {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
  }

  float distance_in_degree(float alpha, float beta)
  {
    float phi = abs(beta - alpha);
    while (phi > 360)
      phi -= 360;
    float distance = phi > 180 ? 360 - phi : phi;
    return distance;
  }

public:
  Graph(const int *id1, const int *id2)
  {
    mat1 = id1;
    mat2 = id2;
  }
  std::vector<float> fitness(const Species &speices)
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
        fits[i] = -1; // the fitest ones will survive later
      else
        fits[i] *= adj;
    }

    return fits;
  }
  float consumed_time(const Solution &result)
  {
    assert(result.size() == 36);
    if (result[0] != 0)
    {
      return 1e8;
    }

    float g = 0;
    float nowx = 0, nowy = 0;
    float orientation = 0;
    for (int point : result)
    {
      float midx, midy;
      lookup(mat1, mat2, point, midx, midy);

      float endx, endy;
      lookup(mat1, mat2, pair(point), endx, endy);

      float mid_orientation = atan2f(midy - nowy, midx - nowx) * 180 / M_PI;
      float next_orientation = atan2f(endy - midy, endx - midx) * 180 / M_PI;

      g += 10 * dist(midx, midy, nowx, nowy) + distance_in_degree(orientation, mid_orientation) / 180.0 * 40 + distance_in_degree(mid_orientation, next_orientation) / 180.0 * 40;
      nowx = endx, nowy = endy, orientation = next_orientation;
    }
    return g;
  }
  Solution to_solution(const std::vector<int> &order)
  {
    Solution sol(order.size());
    for (size_t i = 0; i < order.size(); ++i)
    {
      sol[order[i]] = i;
    }
    return sol;
  }
  std::vector<int> to_order(const Solution &a)
  {
    std::vector<int> ord(a.size());
    for (size_t i = 0; i < a.size(); ++i)
    {
      assert(a[i] >= 0 && a[i] < (int)a.size());

      ord[a[i]] = i;
    }
    return ord;
  }

  float genetic_distance(const Solution &a, const Solution &b)
  {
    assert(a.size() == b.size());
    std::vector<int> a2 = to_order(a), b2 = to_order(b);

    float dist = 0;
    for (size_t i = 0; i < a.size(); i++)
    {
      dist += abs(a2[i] - b2[i]);
    }

    return dist / a.size();
  }
  std::vector<Solution> initial_generation()
  {
    std::vector<Solution> results;
    // results.push_back(A_Star::Graph(mat1, mat2).calc(State()).path);
    for (size_t i = 0; i < 100; i++)
    {
      Solution res;
      for (size_t i = 0; i < 36; i++)
      {
        res.push_back(i);
      }
      random_shuffle(++res.begin(), res.end(), [](int i) { return rand() % i; });
      results.push_back(res);
    }
    return results;
  }
  Population &divide_species(Population &population, const Species &solutions)
  {
    assert(solutions.size() > 0);
    // assert(solutions.species_id <= 100);
    for (auto &&sol : solutions)
    {
      if (solutions.species_id >= 0 && population.find(solutions.species_id) != population.end())
      {
        float distance = genetic_distance(sol, population[solutions.species_id][0]);
        if (distance < same_speices_distance_threshold)
        {
          population[solutions.species_id].push_back(sol);
          continue;
        }
      }

      bool found = false;
      for (auto &&pair : population)
      {
        auto &&speices = pair.second;
        assert(speices.size() > 0);

        float distance = genetic_distance(sol, speices[0]);
        if (distance < same_speices_distance_threshold)
        {
          speices.push_back(sol);
          found = true;
          break;
        }
      }

      if (!found)
      {
        Species species;
        species.push_back(sol);
        species.species_id = Species::get_id();

        population[species.species_id] = species;
      }
    }

    return population;
  }
  std::vector<Solution> crossover(const Solution &s1, const Solution &s2)
  {
    // if(s1.size() != s2.size())
    //   fprintf(stderr, "%lu vs %lu\n", s1.size(), s2.size());
    assert(s1.size() == s2.size());
    std::vector<Solution> solutions;
    { // PMX
      int a = rand() % s1.size(), b = rand() % s1.size();
      if (a > b)
        std::swap(a, b);

      Solution child1(s1.size(), -1), child2(s2.size(), -1);

      for (int i = a; i <= b; ++i)
      {
        child1[i] = s2[i];
        child2[i] = s1[i];
      }
      auto oper = [&](const Solution &parent, Solution &child) {
        for (int i = 0; i < (int)s1.size(); ++i)
        {
          if (a <= i && i <= b)
            continue;
          int now = parent[i];
          Solution::const_iterator iter;
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
  int count_population(const Population &pop)
  {
    int count = 0;
    for (auto &&pair : pop)
      count += pair.second.size();
    return count;
  }
  void mutate(const Solution &solu, Species &next_gen)
  {
    // inherit
    if (random(inherit_probability))
    {
      next_gen.push_back(solu);
    }

    // mutations
    if (random(swap_probability))
    {
      Solution s = solu;
      int a = rand() % s.size(), b = rand() % s.size();
      std::swap(s[a], s[b]);
      assert(s.size() > 0);
      next_gen.push_back(s);
    }
    if (random(reverse_probability))
    {
      Solution s = solu;
      int a = rand() % s.size(), b = rand() % s.size();
      if (a > b)
        std::swap(a, b);
      std::reverse(s.begin() + a, s.begin() + (b + 1));
      assert(s.size() > 0);
      next_gen.push_back(s);
    }
    if (random(shuffle_probability))
    {
      Solution s = solu;
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
  }
  void crossover(const Solution &solu, const Species &now_species, Species &next_gen, const Population &population)
  {
    // crossover
    if (random(crossover_within_species_probability))
    {
      assert(now_species.size() > 0);
      Solution s2 = now_species[rand() % now_species.size()];
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
      auto item = population.begin();
      std::advance(item, rand() % population.size());
      const auto &species2 = item->second;
      assert(species2.size() > 0);
      const Solution &s2 = species2[rand() % species2.size()];
      assert(s2.size() > 0);
      auto &&children = crossover(solu, s2);
      for (auto &&child : children)
      {
        assert(child.size() > 0);
        next_gen.push_back(child);
      }
    }
  }
  Population select(const Population &population)
  {
    Population temp_pop = population;

    for (auto &&pair : temp_pop)
    {
      auto &&species = pair.second;
      assert(species.size() > 0);
      // population must be sorted to keep the elite alive
      if (species.size() > elite_protection_number_per_species)
        species.erase(species.begin() + elite_protection_number_per_species, species.end());
      assert(species.size() <= elite_protection_number_per_species);
    }

    assert(count_population(temp_pop) > 0);

    for (auto &&pair : population)
    {
      auto &&species = pair.second;
      assert(species.size() > 0);

      auto fits = fitness(species);

      for (size_t i = 0; i < species.size(); i++)
      {
        if (random(fits[i]))
        {
          temp_pop[species.species_id].push_back(species[i]);
        }
      }
    }

    return temp_pop;
  }
  Solution evolve(const Species &gen)
  {

    Population population;
    divide_species(population, gen);

    std::cerr << "Generation " << 0 << " ended up with " << gen.size() << " solutions in " << population.size() << " species, and the best Solution comsumes " << consumed_time(pick_best(population)) << " units" << std::endl;

    ThreadPool pool(thread_number);
    for (int i = 1; i <= generation_number; ++i)
    {
      std::vector<std::future<Species>> next_gen_pool;
      // new solutions
      for (const auto &pair : population)
      {
        auto work = [&]() {
          const Species &species = pair.second;
          assert(species.species_id >= 0);
          assert(species.size() > 0);
          Species next_gen;
          next_gen.species_id = species.species_id;

          for (const Solution &solu : species)
          {
            assert(species.size() > 0);
            assert(solu.size() > 0);
            mutate(solu, next_gen);
            crossover(solu, species, next_gen, population);
            assert(next_gen.size() > 0);
          }
          return next_gen;
        };

        next_gen_pool.push_back(pool.enqueue(work));
      }
      pool.join_all_tasks();

      for (size_t i = 0; i < next_gen_pool.size(); i++)
      {
        auto rest = next_gen_pool[i].get();
        assert(rest.size() > 0);
        divide_species(population, rest);
      }

      for (auto &&pair : population)
      {
        auto &species = pair.second;
        // FIXME add multi-thread support
        // pool.enqueue([&]() {
          std::sort(species.begin(), species.end(),
                    [&, this](const Solution &s1, const Solution &s2) {
                      return consumed_time(s1) < consumed_time(s2);
                    });
        // });
      }
      // pool.join_all_tasks();
      show_line_num();

      population = select(population);

      assert(population.size() > 0);

      std::cerr << "Generation " << i << " ended up with " << count_population(population) << " solutions in " << population.size() << " species, and the best Solution comsumes " << consumed_time(pick_best(population)) << " units" << std::endl;
    }

    return pick_best(population);
  }
  Solution pick_best(Population population)
  {
    assert(population.size() > 0);
    assert(population.begin()->second.size() > 0);
    Solution best = population.begin()->second[0];

    for (auto &&species : population)
      for (auto &&sol : species.second)
        if (consumed_time(sol) < consumed_time(best))
          best = sol;

    return best;
  }
};
} // namespace Hybrid
void show_debug_data(const int *mat1, const int *mat2, const Hybrid::Solution &result)
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
  std::cout << "Now species number id: " << Species::get_id() << std::endl;
}
struct AutoSrand
{
  AutoSrand()
  {
    srand(time(0));
  }
} auto_srand;
Hybrid::Solution calculate_path(const int *mat1, const int *mat2)
{
  using namespace Hybrid;

  Graph graph(mat1, mat2);
  auto results = graph.initial_generation();
  Solution best = graph.evolve(results);
  ::show_debug_data(mat1, mat2, best);
  return best;
  // return results[rand() % results.size()];
}

#endif // HYBRID_H
