#include <bits/stdc++.h>
struct State
{
    std::bitset<64> visited;
    float x = 0, y = 0;
    float g = 0;
    std::vector<int> path;
    friend bool operator<(const State &lhs, const State &rhs)
    {
        // TODO here
        return - lhs.g + lhs.path.size() * 10 < - rhs.g + rhs.path.size() * 10;
    }
};
class Graph
{
    int *id1_, *id2_;

public:
    Graph(int *id1, int *id2)
    {
        id1_ = id1;
        id2_ = id2;
    }
    float p2(float x)
    {
        return x * x;
    }
    float dist(float x1, float y1, float x2, float y2)
    {
        return sqrt(p2(x1 - x2) + p2(y1 - y2));
    }
    void calc(State start)
    {
        std::priority_queue<State> que;
        que.push(start);
        while (que.size())
        {
            const State &s = que.top();
            que.pop();

            if (que.size() > 1000)
                break;

            printf("Begin State (%f, %f) %d %f\nQueue size: %d\n", s.x, s.y, s.path.size(), s.g, que.size());
            if (s.path.size() >= 36)
            {
                break;
            }

            for (size_t i = 1; i <= 72; i++)
            {
                if (!s.visited[(i - 1) % 36 + 1])
                {
                    State s2;
                    s2.visited = s.visited;
                    s2.visited[(i - 1) % 36 + 1] = 1;
                    float x1, y1;
                    lookup((i - 1) % 36 + 1, i > 36 ? 1 : 2, x1, y1);     // original location
                    lookup((i - 1) % 36 + 1, i > 36 ? 2 : 1, s2.x, s2.y); // teleport
                    s2.g = s.g + dist(x1, y1, s.x, s.y);
                    s2.path = s.path;
                    s2.path.push_back(i);
                    que.push(s2);
                }
            }
            printf("End State (%f, %f) %d %f\nQueue size: %d\n", s.x, s.y, s.path.size(), s.g, que.size());
        }
    }

    // get the number of a point from row, col and type
    int get_id(int row, int col, int type)
    {
        if (type == 1)
        {
            return id1_[(row - 1) * 6 + col];
        }
        else // if (type == 2)
        {
            return id2_[(row - 1) * 6 + col];
        }
        // return 0;
    }

    // get the location from id and type
    void lookup(int id, int type, float &x, float &y)
    {
        int ans = 0;
        for (size_t i = 0; i < 36; i++)
        {
            if (type == 1)
            {
                if (id1_[i] == id)
                {
                    ans = i;
                    break;
                }
            }
            else
            {
                if (id2_[i] == id)
                {
                    ans = i;
                    break;
                }
            }
        }
        int row = ans / 6;
        int col = ans % 6;

        y = row + (type == 1 ? 0.25 : 0.75);
        x = col + (type == 1 ? 0.25 : 0.75);
    }
};
int mat[] = {
    1, 2, 3, 4, 5, 6,
    7, 8, 9, 10, 11, 12,
    13, 14, 15, 16, 17, 18,
    19, 20, 21, 22, 23, 24,
    25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36};
int mat2[] = {
    1, 2, 3, 4, 5, 6,
    7, 8, 9, 10, 11, 12,
    13, 14, 15, 16, 17, 18,
    19, 20, 21, 22, 23, 24,
    25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36};
int main(int argc, char const *argv[])
{
    Graph graph(mat, mat2);
    State s;
    graph.calc(s);
    return 0;
}
