#if !defined(HYBRID_H)
#define HYBRID_H

#include <bits/stdc++.h>
#include "../graph.h"
#include "../thread_pool.h"
#include "../debug_utils.h"

namespace Hybrid
{
#include "a_star.h"

inline bool random(double p)
{
  return 1.0 * rand() / RAND_MAX <= p;
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
    return solutions.at(index);
    // return solutions[index];
  }
  const Solution &operator[](int index) const
  {
    return solutions.at(index);
    // return solutions[index];
  }
  void sort(const int *mat1, const int *mat2)
  {
    std::sort(solutions.begin(), solutions.end(),
              [&, this](const Solution &s1, const Solution &s2) {
                return s1.consumed_time(mat1, mat2) < s2.consumed_time(mat1, mat2);
              });
  }
};

typedef std::map<int, Species> Population;
class Graph
{
  size_t max_species_population = 10;
  size_t elite_protection_number_per_species = 2;

  size_t initial_generation_population = 100;

  float same_speices_distance_threshold = 10;

  int generation_number = 10;
  int thread_number = 1;

  double inherit_probability = 0.03;
  double swap_probability = 1;
  double reverse_probability = 1;
  double shuffle_probability = 1;
  double transform_probabilty = 1;
  double crossover_within_species_probability = 0.03;
  double crossover_over_species_probability = 0.01;

  const int *mat1, *mat2;

public:
  float consumed_time(const Solution &result) const
  {
    return result.consumed_time(mat1, mat2);
  }

  Graph(const int *id1, const int *id2)
  {
    mat1 = id1;
    mat2 = id2;
  }
  std::vector<float> fitness(const Species &speices)
  {
    // assert(speices.size() > 0);
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
        fits[i] = -1; // the fittest ones will survive later
      else
        fits[i] *= adj;
    }

    return fits;
  }

  std::vector<Solution> initial_generation()
  {
    std::vector<Solution> results;
    results.push_back(A_Star::Graph(mat1, mat2).calc(A_Star::State()).path);
    for (size_t i = 1; i < initial_generation_population; i++)
    {
      Solution res;
      for (size_t i = 0; i < 36; i++)
      {
        res.set(i, i);
      }
      std::random_shuffle(res.begin() + 1, res.end());
      results.push_back(res);
    }
    return results;
  }
  Population &divide_species(Population &population, const Species &solutions)
  {
    // assert(solutions.size() > 0);

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
        // assert(speices.size() > 0);

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
  void crossover(const Solution &s1, const Solution &s2, Species &next_gen)
  {
    // assert(s1.size() == s2.size());
    { // PMX
      int a = rand() % s1.size(), b = rand() % s1.size();
      if (a > b)
        std::swap(a, b);

      Solution child1, child2;

      for (int i = a; i <= b; ++i)
      {
        child1.set(i, s2[i]);
        child2.set(i, s1[i]);
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
          child.set(i, now);
        }
      };
      oper(s1, child1);
      oper(s2, child2);
      next_gen.push_back(child1);
    }
    {
      std::vector<int> _GA = s1.unwrap();
      std::vector<int> _GB = s2.unwrap();
      std::vector<int> offspringGenome;
      std::set<int> keepTrack;

      size_t sizeGenome = _GA.size();
      // generate a random index in range [2, .. , sizeGenome - 2]
      unsigned randomIndex = (rand() % (sizeGenome - 3)) + 2;

      // init offspringGenome and keepTrack
      for (unsigned i = 0; i < randomIndex; ++i)
      {
        offspringGenome.push_back(_GA.at(i));
        keepTrack.insert(_GA.at(i));
      }

      // crosslinking

      for (const auto &gene : _GB)
      {
        bool found = keepTrack.find(gene) != keepTrack.end();
        if (!found)
        {
          offspringGenome.push_back(gene);
        }
      }

      Solution s = offspringGenome;
      next_gen.push_back(s);
    }
    {
      // CX
      Solution child;
      int p = 0;
      while (s1[0] != s2[p])
      {
        child.set(0, s1[p]);
        p = std::find(s1.begin(), s1.end(), s2[p]) - s1.begin();
      }
      for (size_t i = 0; i < child.size(); ++i)
        if (child[i] == -1)
          child.set(i, s2[i]);

      try
      {
        child.check();
        next_gen.push_back(child);
      }
      catch (const std::exception &e)
      {
        // discard illegal solutions
      }
    }
    {
      // OX1
      Solution s;
      size_t a = rand() % s1.size(), b = rand() % s1.size();
      for (size_t i = a; i <= b; ++i)
        s.set(i, s2[i]);
      int ptr = 0;
      for (size_t i = b + 1; i < s.size(); ++i)
      {
        while (ptr < 36 && s.has(s1[ptr]))
          ptr += 1;
        if (ptr < 36)
        {
          s.set(i, s1[ptr]);
          ptr += 1;
        }
      }
      for (size_t i = 0; i < a; ++i)
      {
        while (ptr < 36 && s.has(s1[ptr]))
          ptr += 1;
        if (ptr < 36)
        {
          s.set(i, s1[ptr]);
          ptr += 1;
        }
      }
      next_gen.push_back(s);
    }
    {
      // APX
      Solution s;
      int ptr = 0;
      for (size_t i = 0; i < s.size(); i++)
      {
          if (!s.has(s1[i]))
          {
            s.set(ptr, s1[i]);
            ptr += 1;
          }
          if (!s.has(s2[i])) {
            s.set(ptr, s2[i]);
            ptr += 1;
          }
      }
      next_gen.push_back(s);
    }
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
      std::swap(s.mut_ref(a), s.mut_ref(b));

      // assert(s.size() > 0);
      next_gen.push_back(s);
    }

    if (random(reverse_probability))
    {
      Solution s = solu;
      int a = rand() % s.size(), b = rand() % s.size();
      if (a > b)
        std::swap(a, b);
      std::reverse(s.begin() + a, s.begin() + (b + 1));
      // assert(s.size() > 0);
      next_gen.push_back(s);
    }

    if (random(shuffle_probability))
    {
      Solution s = solu;
      int a = rand() % s.size(), b = rand() % s.size();
      if (a > b)
        std::swap(a, b);

      std::random_shuffle(s.begin() + a, s.begin() + (b + 1));

      // assert(s.size() > 0);

      next_gen.push_back(s);
    }

    if (random(transform_probabilty))
    {
      int a[3] = {rand() % (int)solu.size(), rand() % (int)solu.size(), rand() % (int)solu.size()};
      std::sort(a, a + 3);

      if (a[1] != a[2])
      // otherwise this method cannot work
      {
        Solution s;
        int ptr = 0;
        for (int i = 0; i < a[0]; ++i)
          s.set(ptr++, solu[i]);

        for (int i = a[1] + 1; i < a[2]; ++i)
          s.set(ptr++, solu[i]);

        for (int i = a[0]; i <= a[1]; ++i)
          s.set(ptr++, solu[i]);

        for (int i = a[2]; i < (int)solu.size(); ++i)
          s.set(ptr++, solu[i]);

        next_gen.push_back(s);
      }
    }
  }
  void do_crossover(const Solution &solu, const Species &now_species, Species &next_gen, const Population &population)
  {
    // crossover
    if (random(crossover_within_species_probability))
    {
      // assert(now_species.size() > 0);
      Solution s2 = now_species[rand() % now_species.size()];
      crossover(solu, s2, next_gen);
    }

    if (random(crossover_over_species_probability))
    {
      // assert(population.size() > 0);
      auto item = population.begin();
      std::advance(item, rand() % population.size());
      const auto &species2 = item->second;
      // assert(species2.size() > 0);
      const Solution &s2 = species2[rand() % species2.size()];
      // assert(s2.size() > 0);
      crossover(solu, s2, next_gen);
    }
  }
  Population select(const Population &population)
  {
    Population temp_pop = population;

    for (auto &&pair : temp_pop)
    {
      auto &&species = pair.second;
      // assert(species.size() > 0);
      // population must be sorted to keep the elite alive
      if (species.size() > elite_protection_number_per_species)
        species.erase(species.begin() + elite_protection_number_per_species, species.end());
      // assert(species.size() <= elite_protection_number_per_species);
    }

    // assert(count_population(temp_pop) > 0);

    for (auto &&pair : population)
    {
      auto &&species = pair.second;
      // assert(species.size() > 0);

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

    std::cerr << "Generation " << 0 << " ended up with " << gen.size() << " solutions in " << population.size() << " species, and the best solution comsumes " << consumed_time(pick_best(population)) << " units" << std::endl;

    ThreadPool pool(thread_number);
    for (int i = 1; i <= generation_number; ++i)
    {
      std::vector<std::future<Species>> next_gen_pool;
      // new solutions
      for (const auto &pair : population)
      {
        const Species &species = pair.second;
        auto work = [&]() {
          Species next_gen;
          next_gen.species_id = species.species_id;

          for (const Solution &solu : species)
          {
            mutate(solu, next_gen);
            do_crossover(solu, species, next_gen, population);
          }

          return next_gen;
        };
        next_gen_pool.push_back(pool.enqueue(work));
      }
      {
        Population temp = population;
        for (size_t i = 0; i < next_gen_pool.size(); i++)
        {
          auto rest = next_gen_pool[i].get();
          divide_species(temp, rest);
        }
        population = temp;
      }
      {
        std::vector<std::future<void>> futures;
        for (auto &&pair : population)
        {
          auto &species = pair.second;
          futures.push_back(pool.enqueue([&]() {
            species.sort(mat1, mat2);
          }));
        }
        for (auto &&future : futures)
        {
          future.get();
        }
      }

      population = select(population);

      // assert(population.size() > 0);

      std::cerr << "Generation " << i << " ended up with " << count_population(population) << " solutions in " << population.size() << " species, and the best solution comsumes " << consumed_time(pick_best(population)) << " units" << std::endl;
    }

    return pick_best(population);
  }
  Solution pick_best(Population population)
  {
    // assert(population.size() > 0);
    // assert(population.begin()->second.size() > 0);
    Solution best = population.begin()->second[0];

    for (auto &&species : population)
      for (auto &&sol : species.second)
        if (consumed_time(sol) < consumed_time(best))
          best = sol;

    return best;
  }
}; // namespace Hybrid
} // namespace Hybrid
inline std::vector<int> calculate_path(const int *mat1, const int *mat2)
{
  using namespace Hybrid;

  Graph graph(mat1, mat2);
  auto results = graph.initial_generation();
  Solution best = graph.evolve(results);
  show_debug_data(mat1, mat2, best);
  return best.unwrap();
}

#endif // HYBRID_H
