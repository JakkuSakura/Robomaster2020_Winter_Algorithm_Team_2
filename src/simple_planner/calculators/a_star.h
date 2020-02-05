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
        while (que.size() > 0 && que.size() < 100000)
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
            for (size_t j = 0; j < 36 && cnt < 8; j++)
            {
                int i = rank[j];
                if (!s.visited[i % 36])
                {
                    ++cnt;
                    State s2;
                    s2.visited = s.visited;
                    s2.visited[i % 36] = 1;
                    float x1, y1;
                    lookup(id1_, id2_, i, x1, y1);           // original location
                    lookup(id1_, id2_, pair(i), s2.x, s2.y); // teleport
                    // TODO test this part
                    // float degree = atan2f(y1 - s.y, x1 - s.x) * 180 / M_PI;
                    // s2.orientation = atan2f(y1 - s2.y, x1 - s2.y) * 180 / M_PI;

                    s2.g = s.g + 20 * dist(x1, y1, s.x, s.y);
                    //  + distance_in_degree(s.orientation, degree) / 180.0 * 60 + distance_in_degree(degree, s2.orientation) / 180.0 * 60;
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

std::vector<int> calculate_path_a_star(const int *mat1, const int *mat2)
{
    using namespace A_Star;
    srand(time(0));
    State s;
    Graph graph(mat1, mat2);
    State best = graph.calc(s);
    return best.path;
}

#endif