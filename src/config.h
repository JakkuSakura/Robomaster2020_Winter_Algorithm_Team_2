#if !defined(CONFIG_H)
#define CONFIG_H

#define GENERATE_DATA
const char *data_filename = "/home/jack/run.dat";

// const float TIME_LIMIT = 250000; // secs
const float TIME_LIMIT = 15; // secs

// #define FIXED_VALUES

// GLOBAL PATH GENERATOR
// #include "./simple_planner/calculators/random.h"
// #include "./simple_planner/calculators/a_star.h"
// #include "./simple_planner/calculators/hybrid.h"
#include "./simple_planner/calculators/climbing.h"


#endif // CONFIG_H
