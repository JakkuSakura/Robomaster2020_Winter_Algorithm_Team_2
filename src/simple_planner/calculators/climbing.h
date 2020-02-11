#if !defined(CLIMBING_H)
#define CLIMBING_H

#include "../graph.h"

#include <iostream>
#include <algorithm>
#include <cstring>
#include <cstdio>
#include <cmath>
namespace Climbing
{
    

using namespace std;

double sx, sy, rx[40], ry[40], bx[40], by[40], g[40][40];
int test[40], ans[40];
bool vis[40];

double cal(double a, double b) { return (b - a) * (b - a); }

double cal_path(int to[])
{
    double len = sqrt(cal(sx, rx[to[1]]) + cal(sy, ry[to[1]])) + sqrt(cal(rx[to[1]], bx[to[1]]) + cal(ry[to[1]], by[to[1]]));
    for (int i = 2; i <= 36; i++)
        len += sqrt(cal(bx[to[i - 1]], rx[to[i]]) + cal(by[to[i - 1]], ry[to[i]])) + sqrt(cal(rx[to[i]], bx[to[i]]) + cal(ry[to[i]], by[to[i]]));
    return len;
}

inline std::vector<int> calculate_path(const int *mat1, const int *mat2)
{
    for (size_t i = 0; i < 36; i++)
    {
        lookup(mat1, mat2, i, rx[i], ry[i]);
        lookup(mat1, mat2, ::pair(i), bx[i], by[i]);
    }

    for (int i = 0; i < 36; i++)
        for (int j = 0; j < 36; j++)
            if (i != j)
                g[i][j] = sqrt((cal(bx[i], rx[j]) + cal(by[i], ry[j]))) + sqrt((cal(rx[j], bx[j]) + cal(ry[j], by[j]))); //构造出每个蓝色数字到其他蓝色数字的路程

    for (int i = 1; i <= 36; i++)
        ans[i] = i - 1;
    while (1)
    {
        bool change = 0;
        for (int i = 1; i <= 36; i++)
            for (int j = i + 1; j <= 36; j++)
                for (int k = 1; i + k - 1 < j; k++)
                    for (int l = 1; j + l - 1 <= 36; l++)
                    {
                        int x = 1;
                        for (; x < i; x++)
                            test[x] = ans[x];
                        for (int y = j; y <= j + l - 1; y++, x++)
                            test[x] = ans[y];
                        for (int y = i + k; y < j; y++, x++)
                            test[x] = ans[y];
                        for (int y = i; y <= i + k - 1; y++, x++)
                            test[x] = ans[y];
                        for (; x <= 36; x++)
                            test[x] = ans[x];
                        double f = cal_path(ans);
                        double fnew = cal_path(test);
                        double df = fnew - f;
                        if (df < 0)
                        {
                            memcpy(ans, test, sizeof(ans));
                            change = 1;
                        }
                    }
        if (!change)
            break;
    }
    printf("%lf\n", cal_path(ans));
    std::vector<int> vec;
    for (size_t i = 1; i <= 36; i++)
    {
        vec.push_back(ans[i]);
    }
    show_debug_data(mat1, mat2, vec);

    return vec;
}
} // namespace Climbing
inline std::vector<int> calculate_path(const int *mat1, const int *mat2)
{
    return Climbing::calculate_path(mat1, mat2);
}
#endif // CLIMBING_H
