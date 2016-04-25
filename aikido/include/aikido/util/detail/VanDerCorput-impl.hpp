namespace aikido {
namespace util {

class VanDerCorput::const_iterator
    : public boost::iterator_facade<VanDerCorput::const_iterator, const double,
                                    boost::forward_traversal_tag, const double>
{

public:
  double dereference() const { return curr.first; }

  void increment();
  bool equal(const VanDerCorput::const_iterator &other) const;

private:
  friend VanDerCorput;

  const_iterator(VanDerCorput *seq)
      : seq(seq)
      , n(-1)
      , final_iter(false)
  {
    assert(seq);
    increment();
  }

  VanDerCorput *seq;
  int n;
  bool final_iter;
  std::pair<double, double> curr;
};

} // namespace util
} // namespace aikido
