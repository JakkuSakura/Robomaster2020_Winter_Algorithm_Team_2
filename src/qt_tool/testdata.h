#if !defined(TEST_DATA_H)
#define TEST_DATA_H
#include "../config.h"
#include <algorithm>
inline int const *get_mat1()
{
    static int mat1[] =
        {
            0, 1, 2, 3, 4, 5,
            6, 7, 8, 9, 10, 11,
            12, 13, 14, 15, 16, 17,
            18, 19, 20, 21, 22, 23,
            24, 25, 26, 27, 28, 29,
            30, 31, 32, 33, 34, 35};
    return mat1;
}

#ifndef FIXED_VALUES

#include <ctime>
#include <cstdlib>
namespace AutoSrand
{
struct auto_srand
{
    auto_srand()
    {
        srand(time(0));
    }
};

} // namespace AutoSrand

#endif

inline int const *get_mat2()
{
    static int mat2[] =
        {
            24, 20, 31, 7, 13, 16,
            27, 35, 1, 0, 5, 8,
            6, 12, 2, 14, 15, 11,
            22, 21, 18, 23, 4, 19,
            26, 25, 9, 30, 17, 34,
            33, 10, 3, 32, 28, 29};
#ifndef FIXED_VALUES
    static bool shuffled = false;
    if (!shuffled)
    {
        shuffled = true;
        std::random_shuffle(mat2, mat2 + 36);
    }
#endif
    return mat2;
}
#endif // TEST_DATA_H
