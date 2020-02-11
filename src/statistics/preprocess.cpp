#include <bits/stdc++.h>
#include "../simple_planner/graph.h"
#include "../config.h"
void consumed_time_middle_results(const int *mat1, const std::vector<int> &mat2, const std::vector<int> &v)
{
    // assert(v.size() == 36);
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

        dist(midx, midy, nowx, nowy);
        dist(midx, midy, endx, endy);

        distance_in_degree(orientation, mid_orientation);
        distance_in_degree(mid_orientation, next_orientation);

        nowx = endx, nowy = endy, orientation = next_orientation;
    }
}
struct playthrough
{
    std::vector<int> blue;
    std::vector<int> path;
    std::vector<double> mid_dist;
    std::vector<double> end_dist;
    std::vector<double> mid_angle;
    std::vector<double> end_angle;
    double actual_time;
};

int main()
{
    std::ifstream input(data_filename);
    std::string buf;
    std::vector<playthrough> playthroughs;
    playthrough play;
    for (size_t i = 0; i < 3; i++)
    {
        input >> buf;
        if(buf == "blue:"){
            play.blue.clear();
            for (size_t j = 0; j < 36; j++)
            {
                int x;
                input >> x;
                play.blue.push_back(x);
            }
        } else if (buf == "path:") {
            play.path.clear();
            for (size_t j = 0; j < 36; j++)
            {
                int x;
                input >> x;
                play.path.push_back(x);
            }
        } else if (isdigit(buf[0]))
        {
            input >> play.actual_time;
            playthroughs.push_back(play);
            std::cout << "pushed one" << std::endl;
        }
    }
    return 0;
}