#ifndef GRAPH_H
#define GRAPH_H
#include <vector>
#include <cmath>
#include <iostream>
inline int pair(int x)
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
template<typename M1, typename M2, typename T>
inline void lookup(const M1 &id1_, const M2 &id2_, int id, T &x, T &y)
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

inline float dist(float x1, float y1, float x2, float y2)
{
    return hypot(x1 - x2, y1 - y2);
}
template<typename T>
inline float distance_in_radius(T alpha, T beta)
{
    T phi = std::abs(beta - alpha);
    while (phi > 2 * M_PI)
        phi -= 2 * M_PI;
    T distance = phi > M_PI ? 2 * M_PI - phi : phi;
    return distance;
}
template<typename T>
inline T distance_in_degree(T alpha, T beta)
{
    T phi = std::abs(beta - alpha);
    while (phi > 360)
        phi -= 360;
    T distance = phi > 180 ? 360 - phi : phi;
    return distance;
}

class Solution
{
    std::vector<int> v;
    mutable float cached_time = 0;

public:
    typedef std::vector<int>::const_iterator const_iterator;
    typedef std::vector<int>::iterator iterator;
    Solution()
    {
        v.resize(36);
        for (int i = 0; i < 36; ++i)
            v[i] = -1;
    }
    Solution &operator=(const Solution &s)
    {
        v = s.v;
        cached_time = s.cached_time;

        return *this;
    }
    Solution(const Solution &s) : v(s.v), cached_time(s.cached_time)
    {
    }
    Solution(const std::vector<int> &v) : v(v), cached_time(0)
    {
        // check();
    }
    bool has(int x) const
    {
        for (size_t i = 0; i < 36; i++)
        {
            if (v[i] == x)
                return true;
        }
        return false;
    }
    void set(int i, int val)
    {
        // v.at(i) = val;
        v[i] = val;
        cached_time = 0;
    }
    int &mut_ref(int index)
    {
        cached_time = 0;
        // return v.at(index);
        return v[index];
    }
    int operator[](int index) const
    {
        // return v.at(index);
        return v[index];
    }
    size_t size() const
    {
        return 36;
    }
    void check() const
    {
        if (v.size() != 36)
            throw std::invalid_argument("In check(), size of vector is wrong");
        int counts[36] = {};
        for (size_t i = 0; i < size(); ++i)
        {
            if (v[i] >= 36)
                throw std::invalid_argument("In check(), value for solution is too large");

            if (v[i] < -1)
                throw std::invalid_argument("In check(), value for solution is too small");

            if (v[i] == -1)
                throw std::invalid_argument("In check(), value for solution is incomplete");
            counts[v[i]] += 1;
        }
        for (size_t i = 0; i < size(); ++i)
            if (counts[i] != 1)
                throw std::invalid_argument("In check(), count of value for solution is not exactly one");
    }
    std::vector<int> to_order() const
    {
        std::vector<int> ord(size());
        for (size_t i = 0; i < size(); ++i)
        {
            ord[v[i]] = i;
        }
        return ord;
    }
    iterator begin()
    {
        cached_time = 0;
        return v.begin();
    }
    iterator end()
    {
        cached_time = 0;
        return v.end();
    }
    const_iterator begin() const
    {
        return v.begin();
    }
    const_iterator end() const
    {
        return v.end();
    }

    std::vector<int> unwrap() const
    {
        std::vector<int> vec = v;
        return vec;
    }
    float calc_consumed_time(const int *mat1, const int *mat2) const
    {
        // assert(v.size() == 36);
        if (v[0] != 0)
        {
            return 1e8;
        }

        int time = 0;
        float nowx = 0, nowy = 0;
        float orientation = 0;
        for (int point : v)
        {
            float midx, midy;
            lookup(mat1, mat2, point, midx, midy);

            float endx, endy;
            lookup(mat1, mat2, pair(point), endx, endy);

            float mid_orientation = atan2f(midy - nowy, midx - nowx) * 180 / M_PI;
            float next_orientation = atan2f(endy - midy, endx - midx) * 180 / M_PI;

            time += 10 * dist(midx, midy, nowx, nowy) + distance_in_degree(orientation, mid_orientation) / 180.0 * 40 + distance_in_degree(mid_orientation, next_orientation) / 180.0 * 40;
            nowx = endx, nowy = endy, orientation = next_orientation;
        }
        return time;
    }
    float consumed_time(const int *mat1, const int *mat2) const
    {
        if (cached_time)
            return cached_time;
        const Solution &so = *this;
        return cached_time = so.calc_consumed_time(mat1, mat2);
    }
};

inline void show_debug_data(const int *mat1, const int *mat2, const Solution &result)
{
    std::cout << "Map: ";
    for (int i = 0; i < 36; i++)
    {
        std::cout << mat2[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "Result: ";
    for (int i = 0; i < 36; i++)
    {
        std::cout << result[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "Predicted consumed time: " << result.consumed_time(mat1, mat2) << std::endl;
}

inline float genetic_distance(const Solution &a, const Solution &b)
{
    // assert(a.size() == b.size());
    std::vector<int> a2 = a.to_order(), b2 = b.to_order();

    float dist = 0;
    for (size_t i = 0; i < a.size(); i++)
    {
        dist += std::abs(a2[i] - b2[i]);
    }

    return dist / a.size();
}

#endif