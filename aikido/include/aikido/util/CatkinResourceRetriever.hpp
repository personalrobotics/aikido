#ifndef AIKIDO_UTIL_CATKINRESOURCERETRIEVER_H_
#define AIKIDO_UTIL_CATKINRESOURCERETRIEVER_H_
#include <string>
#include <unordered_map>
#include <vector>
#include <dart/common/ResourceRetriever.h>

namespace aikido {
namespace util {

class CatkinResourceRetriever : public virtual dart::common::ResourceRetriever {
public:
  CatkinResourceRetriever();
  explicit CatkinResourceRetriever(
    const dart::common::ResourceRetrieverPtr& _delegate);
  virtual ~CatkinResourceRetriever() = default;

  bool exists(const dart::common::Uri& _uri) override;

  dart::common::ResourcePtr retrieve(
    const dart::common::Uri& _uri) override;

private:
  struct Workspace {
    std::string mPath;
    std::unordered_map<std::string, std::string> mSourceMap;
  };

  static void searchForPackages(const std::string& _path,
    std::unordered_map<std::string, std::string>& _packageMap);
  static std::string getPackageNameFromXML(const std::string& _path);

  std::vector<Workspace> getWorkspaces() const;
  dart::common::Uri resolvePackageUri(const dart::common::Uri& _uri) const;

  dart::common::ResourceRetrieverPtr mDelegate;
  std::vector<Workspace> mWorkspaces;
};

} // namespace util
} // namespace aikido

#endif // ifndef AIKIDO_UTIL_CATKINRESOURCERETRIEVER_H_
