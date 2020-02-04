#ifndef GRAPH_H
#define GRAPH_H
#include <vector>
typedef std::vector<int> (*calculate_path)(const int *, const int *);
int pair(int x)
{
    if (x < 36)
    {
        return x + 36;
    }
    else
    {
        return x - 36;
    }
}
// get the location from id and type
void lookup(const int *id1_, const int *id2_, int id, float &x, float &y)
{
    int type = id < 36 ? 1 : 2;
    int ans = 0;
    if (type == 1)
    {
        for (size_t i = 0; i < 36; i++)
        {
            if (id1_[i] == id)
            {
                ans = i;
                break;
            }
        }
    }
    else
    {
        for (size_t i = 0; i < 36; i++)
        {
            if (id2_[i] == id - 36)
            {
                ans = i;
                break;
            }
        }
    }

    int row = ans / 6;
    int col = ans % 6;

    y = row * 2 + type;
    x = col * 2 + type;
}
#endif