#include <aikido/common/CatkinResourceRetriever.hpp>

#include <fstream>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <dart/common/Console.hpp>
#include <dart/common/LocalResourceRetriever.hpp>
#include <dart/common/Uri.hpp>
#include <tinyxml2.h>

static const std::string CATKIN_MARKER(".catkin");

using dart::common::Uri;

namespace aikido {
namespace common {
namespace {

//==============================================================================
std::string getPackageNameFromXML(const std::string& _path)
{
  using tinyxml2::XMLHandle;
  using tinyxml2::XMLElement;

  tinyxml2::XMLDocument document;
  if (document.LoadFile(_path.c_str()))
  {
    dtwarn << "[CatkinResourceRetriever] Failed loading package.xml file '"
           << _path << "': " << document.GetErrorStr1() << "\n";
    return "";
  }

  XMLHandle root_handle(document.RootElement());
  XMLHandle name_handle = root_handle.FirstChildElement("name");
  XMLElement* name_element = name_handle.ToElement();

  if (!name_element)
  {
    dtwarn << "[CatkinResourceRetriever] Failed loading package.xml file '"
           << _path << "': File does not contain a <name> element.\n";
    return "";
  }

  if (!name_element->GetText())
  {
    dtwarn << "[CatkinResourceRetriever] Failed loading package.xml file '"
           << _path << "': <name> element is empty.\n";
    return "";
  }

  std::string package_name = name_element->GetText();
  boost::algorithm::trim(package_name);

  if (package_name.empty())
  {
    dtwarn << "[CatkinResourceRetriever] Failed loading package.xml file '"
           << _path << "': <name> element is empty.\n";
    return "";
  }

  return package_name;
}

//==============================================================================
void searchForPackages(
    const boost::filesystem::path& _packagePath,
    std::unordered_map<std::string, std::string>& _packageMap)
{
  using boost::filesystem::directory_iterator;
  using boost::filesystem::path;
  using boost::filesystem::file_status;
  using boost::filesystem::exists;

  // Ignore this directory if it contains a CATKIN_IGNORE file.
  const path catkin_ignore_path = _packagePath / "CATKIN_IGNORE";
  if (exists(catkin_ignore_path))
    return;

  // Try loading the package.xml file.
  const path package_xml_path = _packagePath / "package.xml";
  if (exists(package_xml_path))
  {
    const std::string package_name
        = getPackageNameFromXML(package_xml_path.string());
    if (!package_name.empty())
    {
      const auto result = _packageMap.insert(
          std::make_pair(package_name, _packagePath.string()));
      if (!result.second)
      {
        dtwarn << "[CatkinResourceRetriever] Found two package.xml"
                  " files for package '"
               << package_name << "': '" << result.first->second << "' and '"
               << _packagePath << "'.\n";
      }
      return; // Don't search for packages inside packages.
    }
  }

  // Recurse on subdirectories.
  directory_iterator it(_packagePath);
  directory_iterator end;

  while (it != end)
  {
    boost::system::error_code status_error;
    const file_status status = it->status(status_error);
    if (status_error)
    {
      dtwarn << "[CatkinResourceRetriever] Failed recursing into directory '"
             << it->path() << "'.\n";
      continue;
    }

    if (status.type() == boost::filesystem::directory_file)
      searchForPackages(it->path().string(), _packageMap);

    ++it;
  }
}

} // namespace

//==============================================================================
CatkinResourceRetriever::CatkinResourceRetriever()
  : CatkinResourceRetriever(
        std::make_shared<dart::common::LocalResourceRetriever>())
{
}

//==============================================================================
CatkinResourceRetriever::CatkinResourceRetriever(
    const dart::common::ResourceRetrieverPtr& _delegate)
  : mDelegate(_delegate), mWorkspaces(getWorkspaces())
{
  // Do nothing
}

//==============================================================================
bool CatkinResourceRetriever::exists(const Uri& _uri)
{
  const Uri resolvedUri = resolvePackageUri(_uri);
  if (resolvedUri.mPath)
    return mDelegate->exists(resolvedUri);
  else
    return false;
}

//==============================================================================
dart::common::ResourcePtr CatkinResourceRetriever::retrieve(const Uri& _uri)
{
  const Uri resolvedUri = resolvePackageUri(_uri);
  if (resolvedUri.mPath)
    return mDelegate->retrieve(resolvedUri);
  else
    return nullptr;
}

//==============================================================================
auto CatkinResourceRetriever::getWorkspaces() const -> std::vector<Workspace>
{
  using dart::common::ResourcePtr;
  using boost::filesystem::path;

  const char* cmake_prefix_path = std::getenv("CMAKE_PREFIX_PATH");
  if (!cmake_prefix_path)
  {
    dtwarn << "[CatkinResourceRetriever::getWorkspaces] The CMAKE_PREFIX_PATH"
              " environmental variable is not defined. Did you source"
              " 'setup.bash' in this shell?\n";
    return std::vector<Workspace>();
  }

  // Split CMAKE_PREFIX_PATH by the ':' path separator delimiter.
  std::vector<std::string> workspace_candidates;
  boost::split(workspace_candidates, cmake_prefix_path, boost::is_any_of(":"));

  // Filter out directories that are missing the Catkin marker file. If the
  // marker file exists, then we also read its contents to build a list of all
  // source directories.
  std::vector<Workspace> workspaces;

  for (const std::string& workspace_path : workspace_candidates)
  {
    if (workspace_path.empty())
      continue;

    const Uri workspace_uri = Uri::createFromPath(workspace_path + "/");
    const Uri marker_uri
        = Uri::createFromRelativeUri(workspace_uri, CATKIN_MARKER);

    // Skip non-Catkin workspaces by checking for the marker file. We do this
    // before reading the file because there is no way to differentiate between
    // "file does not exist" and other types of errors (e.g.  permission)
    // through the std::ifstream API.
    if (!mDelegate->exists(workspace_uri))
      continue;

    Workspace workspace;
    workspace.mPath = workspace_path;

    // Read the list of source packages (if any) from the marker file.
    const ResourcePtr marker_resource = mDelegate->retrieve(marker_uri);
    if (marker_resource)
    {
      const size_t filesize = marker_resource->getSize();

      std::string contents;
      contents.resize(filesize);
      marker_resource->read(&contents.front(), filesize, 1);

      // Split the string into a list of paths by the ';' delimiter. I'm not
      // sure why this doesn't use the standard path separator delimitor ':'.
      std::vector<std::string> source_paths;
      if (!contents.empty())
        boost::split(source_paths, contents, boost::is_any_of(";"));

      for (const std::string& source_path : source_paths)
        searchForPackages(source_path, workspace.mSourceMap);
    }
    else
    {
      dtwarn
          << "[CatkinResourceRetriever::getWorkspaces] Failed reading package"
             " source directories from the marker file '"
          << marker_uri.getFilesystemPath()
          << "'. Resources in the source"
             " space of this workspace will not resolve.\n";
    }

    workspaces.push_back(workspace);
  }

  return workspaces;
}

//==============================================================================
Uri CatkinResourceRetriever::resolvePackageUri(const Uri& _uri) const
{
  using boost::filesystem::path;

  if (!_uri.mScheme || *_uri.mScheme != "package")
    return Uri();

  if (!_uri.mAuthority)
  {
    dtwarn << "Failed extracting package name from URI '" << _uri.toString()
           << ".\n";
    return Uri();
  }

  const std::string& packageName = *_uri.mAuthority;
  std::string relativePath = _uri.mPath.get_value_or("");

  // Strip the leading "/", so this path will be interpreted as being relative
  // to the package directory.
  if (!relativePath.empty() && relativePath.front() == '/')
    relativePath = relativePath.substr(1);

  // Sequentially check each chained workspace.
  for (const Workspace& workspace : mWorkspaces)
  {
    // First check the 'devel' or 'install' space.
    const path develPath = path(workspace.mPath) / "share" / packageName;
    const Uri resourceDevelUri = Uri::createFromRelativeUri(
        Uri::createFromPath(develPath.string() + "/"), relativePath);

    if (mDelegate->exists(resourceDevelUri))
      return resourceDevelUri;

    // Next, check the source space.
    const auto it = workspace.mSourceMap.find(packageName);
    if (it != std::end(workspace.mSourceMap))
    {
      const Uri resourceSourceUri = Uri::createFromRelativeUri(
          Uri::createFromPath(it->second + "/"), relativePath);

      if (mDelegate->exists(resourceSourceUri))
        return resourceSourceUri;
    }
  }

  return Uri();
}

} // namespace common
} // namespace aikido
