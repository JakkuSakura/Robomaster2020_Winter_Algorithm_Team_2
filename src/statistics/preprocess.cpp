#include <bits/stdc++.h>
#include "../simple_planner/graph.h"
#include "../qt_tool/testdata.h"
#include "../config.h"
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
void consumed_time_middle_results(const int *mat1, const std::vector<int> &mat2, playthrough &play)
{
    // assert(v.size() == 36);
    float nowx = 0.5, nowy = 0.5;
    float orientation = 0;
    for (int point : play.path)
    {
        float midx, midy;
        lookup(mat1, mat2, point, midx, midy);

        float endx, endy;
        lookup(mat1, mat2, pair(point), endx, endy);

        float mid_orientation = atan2f(midy - nowy, midx - nowx) * 180 / M_PI;
        float next_orientation = atan2f(endy - midy, endx - midx) * 180 / M_PI;

        play.mid_dist.push_back(dist(midx, midy, nowx, nowy));
        play.end_dist.push_back(dist(midx, midy, endx, endy));

        play.mid_angle.push_back(distance_in_degree(orientation, mid_orientation));
        play.end_angle.push_back(distance_in_degree(mid_orientation, next_orientation));

        nowx = endx, nowy = endy, orientation = next_orientation;
    }
}
template<typename T>
void show_labels(const T &str, int x, std::ostream &out)
{
    for(int i = 0; i < x; ++i)
        out << str << i << " ";
}
template<typename T>
void show_vec(const T &v, std::ostream &out)
{
    for(auto &&x:v)
        out << x << " ";
}
int main()
{
    std::ifstream input(data_filename);
    std::string buf;
    std::vector<playthrough> playthroughs;
    playthrough play;
    while (input)
    {
        input >> buf;
        std::cout << "read a line" << std::endl;
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
    std::ofstream output("/home/jack/stat.txt");
    show_labels("blue", 36, output);
    show_labels("path", 36, output);
    show_labels("mid_d", 36, output);
    show_labels("end_d", 36, output);
    show_labels("mid_a", 36, output);
    show_labels("end_a", 36, output);
    output << std::endl;
    for(playthrough &play: playthroughs)
    {
        consumed_time_middle_results(get_mat1(), play.path, play);
        show_vec(play.blue, output);
        show_vec(play.path, output);
        show_vec(play.mid_dist, output);
        show_vec(play.end_dist, output);
        show_vec(play.mid_angle, output);
        show_vec(play.end_angle, output);
        output << std::endl;
    }

    return 0;
}
