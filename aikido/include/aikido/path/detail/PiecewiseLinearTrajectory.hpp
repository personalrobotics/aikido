namespace aikido {
namespace path {

/// Custom comparator for Waypoint pairs
struct CompareWaypoints {
  double asTime(const PiecewiseLinearTrajectory::Waypoint &_wpt) const {
    return _wpt.first;
  }

  double asTime(const double &_t) const { return _t; }

  template <typename T1, typename T2>
  bool operator()(T1 const &_t1, T2 const &_t2) const {
    return asTime(_t1) < asTime(_t2);
  }
};
}
}
