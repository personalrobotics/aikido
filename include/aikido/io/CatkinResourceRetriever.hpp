#ifndef AIKIDO_IO_CATKINRESOURCERETRIEVER_HPP_
#define AIKIDO_IO_CATKINRESOURCERETRIEVER_HPP_

#include <string>
#include <unordered_map>
#include <vector>
#include <dart/common/ResourceRetriever.hpp>

namespace aikido {
namespace io {

/// Retreive resources specified by 'package://' URIs. This class resolves
/// a 'package://' URI to a 'file://' URI using the same logic as
/// `catkin.find_in_workspaces`, then resolves the resource using a delegate
/// \c ResourceRetriever.
class CatkinResourceRetriever : public virtual dart::common::ResourceRetriever
{
public:
  /// Constructs a resource retriever that delegates to a
  /// \c LocalResourceRetriever to resolve 'file://' URIs.
  CatkinResourceRetriever();

  /// Constructs a resource retriever that delegates to a
  /// \c LocalResourceRetriever to retrieve 'file://' URIs.
  ///
  /// \param _delegate resource retriever to retrieve 'file://' URIs
  explicit CatkinResourceRetriever(
      const dart::common::ResourceRetrieverPtr& _delegate);

  virtual ~CatkinResourceRetriever() = default;

  // Documentation inherited.
  bool exists(const dart::common::Uri& _uri) override;

  // Documentation inherited.
  dart::common::ResourcePtr retrieve(const dart::common::Uri& _uri) override;

private:
  struct Workspace
  {
    std::string mPath;
    std::unordered_map<std::string, std::string> mSourceMap;
  };

  std::vector<Workspace> getWorkspaces() const;
  dart::common::Uri resolvePackageUri(const dart::common::Uri& _uri) const;

  dart::common::ResourceRetrieverPtr mDelegate;
  std::vector<Workspace> mWorkspaces;
};

} // namespace io
} // namespace aikido

#endif // AIKIDO_IO_CATKINRESOURCERETRIEVER_HPP_
