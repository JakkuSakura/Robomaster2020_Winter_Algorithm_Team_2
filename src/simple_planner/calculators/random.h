#if !defined(RANDOM_PATH_H)
#define RANDOM_PATH_H
#include <vector>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <algorithm>
#include "../graph.h"
void show_debug_data(const int *mat1, const int *mat2, const std::vector<int> &result)
{

  std::cout << "Result: ";
  for (int i = 0; i < 36; i++)
  {
    std::cout << result[i] << " ";
  }
  std::cout << std::endl;
  std::cout << "Predicted consumed time: " << consumed_time(mat1, mat2, result) << std::endl;
}
std::vector<int> calculate_path(const int *mat1, const int *mat2)
{
    std::vector<int> path(36);
    for (size_t i = 0; i < 36; i++)
    {
        path[i] = i;
    }
    std::random_shuffle(++path.begin(), path.end());
    show_debug_data(mat1, mat2, path);
    return path;
}

#endif // RANDOM_PATH_H

