#ifndef A_STAR
#define A_STAR
#include <bits/stdc++.h>
#include "../graph.h"
namespace A_Star
{
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
    State calc(State start)
    {
        std::priority_queue<State> que;
        que.push(start);
        State best;
        best.g = 1e6;
        for (size_t i = 0; i < 36; i++)
        {
            best.path.push_back(i);
        }

        int cnt = 0;
        while (que.size() > 0 && que.size() < 1000000)
        {
            const State s = que.top();
            que.pop();

            if (s.path.size() >= 36)
            {
                if (s.g < best.g)
                {
                    best = s;
                }
                if (++cnt > 500)
                    return best;
            }
            int rank[36];
            for (size_t i = 0; i < 36; i++)
            {
                rank[i] = i;
            }
            std::sort(rank, rank + 36, [&, this](int p1, int p2) {
                float x1, y1, x2, y2;
                lookup(id1_, id2_, p1, x1, y1);
                lookup(id1_, id2_, p2, x2, y2);
                return dist(x1, y1, s.x, s.y) < dist(x2, y2, s.x, s.y);
            });
            // static int count = 0;
            // if (++count < 100)
            // {
            //     std::cout << "(" << s.x << ", " << s.y << ") ranked ";
            //     for (size_t i = 0; i < 36; i++)
            //     {
            //         std::cout << rank[i] << " ";
            //     }
            //     std::cout << std::endl;
            // }

            int cnt = 0;
            for (size_t j = 0; j < 36 && cnt < 10; j++)
            {
                int i = rank[j];
                if (!s.visited[i % 36])
                {
                    ++cnt;
                    State s2;
                    s2.visited = s.visited;
                    s2.visited[i % 36] = 1;
                    float midx, midy;
                    lookup(id1_, id2_, i, midx, midy);       // original location
                    lookup(id1_, id2_, pair(i), s2.x, s2.y); // teleport

                    float degree = atan2f(midy - s.y, midx - s.x) * 180 / M_PI;
                    s2.orientation = atan2f(s2.y - midy, s2.x - midx) * 180 / M_PI;

                    s2.g = s.g + 10 * dist(midx, midy, s.x, s.y) + distance_in_degree(s.orientation, degree) / 180.0 * 40 + distance_in_degree(degree, s2.orientation) / 180.0 * 40 + 10.0 * rand() / RAND_MAX;
                    s2.path = s.path;
                    s2.path.push_back(i);
                    if (s2.g < s2.path.size() * STEP_VALUE)
                        que.push(s2);
                }
            }
        }
        return best;
    }
};
} // namespace A_Star


void show_debug_data(const int *mat1, const int *mat2, std::vector<int> result)
{
    using namespace A_Star;

    std::cout << "Result: ";
    for (int i = 0; i < 36; i++)
    {
        std::cout << result[i] << " ";
    }
    std::cout << std::endl;
    Graph graph(mat1, mat2);
    std::cout << "Time: " << graph.consumed_time(result) << std::endl;
}

std::vector<int> calculate_path(const int *mat1, const int *mat2)
{
    using namespace A_Star;
    srand(time(0));
    State s;
    Graph graph(mat1, mat2);
    State best = graph.calc(s);
    show_debug_data(mat1, mat2, best.path);
    return best.path;
}



#endif