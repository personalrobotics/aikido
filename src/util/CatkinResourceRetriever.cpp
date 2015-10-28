#include <fstream>
#include <iostream>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <dart/common/Console.h>
#include <dart/common/Uri.h>
#include <dart/common/LocalResourceRetriever.h>
#include <r3/util/CatkinResourceRetriever.h>

static const std::string CATKIN_MARKER(".catkin");

using dart::common::Uri;

namespace r3 {
namespace util {

CatkinResourceRetriever::CatkinResourceRetriever()
  : CatkinResourceRetriever(
      std::make_shared<dart::common::LocalResourceRetriever>())
{
}

CatkinResourceRetriever::CatkinResourceRetriever(
      const dart::common::ResourceRetrieverPtr& _delegate)
  : mDelegate(_delegate)
  , mWorkspaces(getWorkspaces())
{
}

bool CatkinResourceRetriever::exists(const Uri& _uri)
{
  const Uri resolvedUri = resolvePackageUri(_uri);
  if(resolvedUri.mPath)
    return mDelegate->exists(resolvedUri);
  else
    return false;
}

dart::common::ResourcePtr CatkinResourceRetriever::retrieve(const Uri& _uri)
{
  const Uri resolvedUri = resolvePackageUri(_uri);
  if(resolvedUri.mPath)
    return mDelegate->retrieve(resolvedUri);
  else
    return nullptr;
}

auto CatkinResourceRetriever::getWorkspaces() const -> std::vector<Workspace>
{
  using dart::common::ResourcePtr;
  using boost::filesystem::path;

  const char *cmake_prefix_path = std::getenv("CMAKE_PREFIX_PATH");
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
    const Uri marker_uri = Uri::createFromRelativeUri(
      workspace_uri, CATKIN_MARKER);

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
      {
        const size_t i = source_path.rfind('/');
        if (i == std::string::npos)
        {
          dtwarn << "[CatkinResourceRetriever::getWorkspaces] Unable to extract"
                 << " package name from the entry '" << source_path << "' in"
                 << " marker file '" << marker_uri.getFilesystemPath()
                 << "'. Resources in this directory will not resolve.\n";
          continue;
        }

        // TODO: We should actually extract the package name from the
        // package.xml file, not assume that it's the last component of the
        // directory.
        const std::string package_name = source_path.substr(i + 1);

        const auto result = workspace.mSourceMap.insert(
          std::make_pair(package_name, source_path));
        if (!result.second && result.first->second != source_path)
        {
          dtwarn << "[CatkinResourceRetriever::getWorkspaces] Found conflicting"
                 << " source paths for package '" << package_name << "': '"
                 << result.first->second << "' and '" << source_path << "'."
                    " Resources in this package may not resolve.\n";
        }
      }
    }
    else
    {
      dtwarn << "[CatkinResourceRetriever::getWorkspaces] Failed reading package"
                " source directories from the marker file '"
             << marker_uri.getFilesystemPath() << "'. Resources in the source"
                " space of this workspace will not resolve.\n";
    }

    workspaces.push_back(workspace);
  }

  return workspaces;
}

Uri CatkinResourceRetriever::resolvePackageUri(const Uri& _uri) const
{
  using boost::filesystem::path;

  if(!_uri.mScheme || *_uri.mScheme != "package")
    return "";

  if(!_uri.mAuthority)
  {
    dtwarn << "Failed extracting package name from URI '"
           << _uri.toString() << ".\n";
    return "";
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

} // namespace util
} // namespace r3
