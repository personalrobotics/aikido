#ifndef AIKIDO_PLANNER_OMPL_BACKWARDCOMPATIBILITY_HPP_
#define AIKIDO_PLANNER_OMPL_BACKWARDCOMPATIBILITY_HPP_

#include <ompl/config.h>

#define OMPL_VERSION_AT_LEAST(x,y,z) \
  (OMPL_MAJOR_VERSION > x || (OMPL_MAJOR_VERSION >= x && \
  (OMPL_MINOR_VERSION > y || (OMPL_MINOR_VERSION >= y && \
  OMPL_PATCH_VERSION >= z))))

#define OMPL_MAJOR_MINOR_VERSION_AT_LEAST(x,y) \
  (OMPL_MAJOR_VERSION > x || (OMPL_MAJOR_VERSION >= x && \
  (OMPL_MINOR_VERSION > y || (OMPL_MINOR_VERSION >= y))))

#define OMPL_VERSION_AT_MOST(x,y,z) \
  (OMPL_MAJOR_VERSION < x || (OMPL_MAJOR_VERSION <= x && \
  (OMPL_MINOR_VERSION < y || (OMPL_MINOR_VERSION <= y && \
  OMPL_PATCH_VERSION <= z))))

#define OMPL_MAJOR_MINOR_VERSION_AT_MOST(x,y) \
  (OMPL_MAJOR_VERSION < x || (OMPL_MAJOR_VERSION <= x && \
  (OMPL_MINOR_VERSION < y || (OMPL_MINOR_VERSION <= y))))

namespace aikido {
namespace planner {
namespace ompl {

#if OMPL_VERSION_AT_LEAST(1,2,0)

#include <memory>

#define OMPL_PLACEHOLDER(ph) std::placeholders::ph

template <class T>
using ompl_shared_ptr = std::shared_ptr<T>;

template <class T>
using ompl_weak_ptr = std::weak_ptr<T>;

template <class T, class... Args>
ompl_shared_ptr<T> ompl_make_shared(Args&&... args)
{
  return std::make_shared<T>(std::forward<Args>(args)...);
}

template <class T, class U>
ompl_shared_ptr<T> ompl_dynamic_pointer_cast(const ompl_shared_ptr<U>& r)
{
  return std::dynamic_pointer_cast<T>(r);
}

template <class T, class U>
ompl_shared_ptr<T> ompl_static_pointer_cast(const ompl_shared_ptr<U>& r)
{
  return std::static_pointer_cast<T>(r);
}

template <class F, class... Args>
auto ompl_bind(F&& f, Args&&... args)
-> decltype(std::bind(std::forward<F>(f), std::forward<Args>(args)...))
{
  return std::bind(std::forward<F>(f), std::forward<Args>(args)...);
}

#else

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#define OMPL_PLACEHOLDER(ph) ph

template <class T>
using shared_ptr = boost::shared_ptr<T>;

template <class T>
using weak_ptr = boost::weak_ptr<T>;

template <class T, class... Args>
ompl_shared_ptr<T> ompl_make_shared(Args&&... args)
{
  return boost::make_shared<T>(boost::forward<Args>(args)...);
}

template <class T, class U>
ompl_shared_ptr<T> ompl_dynamic_pointer_cast(const ompl_shared_ptr<U>& r)
{
  return boost::dynamic_pointer_cast<T>(r);
}

template <class T, class U>
ompl_shared_ptr<T> ompl_static_pointer_cast(const ompl_shared_ptr<U>& r)
{
  return boost::static_pointer_cast<T>(r);
}

template <class F, class... Args>
auto ompl_bind(F&& f, Args&&... args)
-> decltype(boost::bind(boost::forward<F>(f), boost::forward<Args>(args)...))
{
  return boost::bind(boost::forward<F>(f), boost::forward<Args>(args)...);
}

#endif

} // namespace ompl
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OMPL_BACKWARDCOMPATIBILITY_HPP_
