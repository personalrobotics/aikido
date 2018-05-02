#pragma once
// TODO: Convert to struct?
struct Params
{
    const int dof;
    const int dimensions;
    const double s_max;
    const double a_max;
    const double v_max;
};

constexpr Params param_1dof = {
    1,   // dof
    2,   // dimensions
    5.0,  // s_max
    1.0,  // a_max
    10   // v_max
};
constexpr Params param_2dof = {
    2,   // dof
    4,   // dimensions
    5.0,  // s_max
    1.0,  // a_max
    10   // v_max
};
constexpr Params param_3dof = {
    3,   // dof
    6,   // dimensions
    3.14,  // s_max
    2.0, // a_max
    10 // v_max
};
constexpr Params param_4dof = {
    4,   // dof
    8,   // dimensions
    5.0,  // s_max
    1.0,  // a_max
    10  // v_max
};
constexpr Params param_6dof = {
    6, // dof
    12, // dimensions
    1.57,  // s_max
    10.0, // a_max
    10 // v_max
};
constexpr Params param_8dof = {
    8, // dof
    16, // dimensions
    3.14,  // s_max
    1.0, // a_max
    10 // v_max
};
constexpr Params param_7dof = {
    7, // dof
    14, // dimensions
    3.14,  // s_max
    1.0, // a_max
    10 // v_max
};
constexpr Params param_12dof = {
    12, // dof
    24, // dimensions
    3.14,  // s_max
    1.0, // a_max
    10 // v_max
};

constexpr Params param = param_6dof;

