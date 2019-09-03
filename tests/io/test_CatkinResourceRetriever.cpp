#include <cstdlib>
#include <gtest/gtest.h>
#include <aikido/io/CatkinResourceRetriever.hpp>

#define STR_EXPAND(tok) #tok
#define STR(tok) STR_EXPAND(tok)

using aikido::io::CatkinResourceRetriever;
using dart::common::ResourcePtr;
using dart::common::Uri;

static constexpr auto WORKSPACE_PATH = STR(AIKIDO_TEST_WORKSPACE_PATH);

/// gtest predicate for asserting that the content of a Resource equals a
/// string value. This predicate assumes that the ResourcePtr has its position
/// indicator at the beginning of the file. After calling this function, the
/// final location of the position indicator is undefined.
static testing::AssertionResult CompareResourceContents(
    const std::string& _expectedContent, const ResourcePtr& _resource)
{
  if (!_resource)
    return testing::AssertionFailure() << "Resource is nullptr";

  const std::size_t length = _resource->getSize();
  std::vector<char> content_buffer(length, '\0');
  const std::size_t read_length
      = _resource->read(content_buffer.data(), length, 1);
  if (read_length != 1)
    return testing::AssertionFailure() << "Failed reading Resource.";

  const std::string content(
      std::begin(content_buffer), std::end(content_buffer));
  if (content != _expectedContent)
    return testing::AssertionFailure()
           << "Got content \"" << content << "\", expected \""
           << _expectedContent << "\".";

  return testing::AssertionSuccess();
}

TEST(CatkinResourceRetrieverTests, ExistsInDevelOnly)
{
  setenv("CMAKE_PREFIX_PATH", WORKSPACE_PATH, 1);

  const Uri uri = Uri::getUri("package://my_package1/devel_only.txt");
  const std::string content = "my_package1_devel_only\n";

  CatkinResourceRetriever retriever;
  EXPECT_TRUE(retriever.exists(uri));
  EXPECT_TRUE(CompareResourceContents(content, retriever.retrieve(uri)));
}

TEST(CatkinResourceRetrieverTests, ExistsInSourceOnly)
{
  setenv("CMAKE_PREFIX_PATH", WORKSPACE_PATH, 1);

  const Uri uri = Uri::getUri("package://my_package1/source_only.txt");
  const std::string content = "my_package1_source_only\n";

  CatkinResourceRetriever retriever;
  EXPECT_TRUE(retriever.exists(uri));
  EXPECT_TRUE(CompareResourceContents(content, retriever.retrieve(uri)));
}

TEST(CatkinResourceRetrieverTests, ExistsInDevelAndSource)
{
  setenv("CMAKE_PREFIX_PATH", WORKSPACE_PATH, 1);

  const Uri uri = Uri::getUri("package://my_package1/devel_and_source.txt");
  const std::string content = "my_package1_devel_and_source:devel\n";

  CatkinResourceRetriever retriever;
  EXPECT_TRUE(retriever.exists(uri));
  EXPECT_TRUE(CompareResourceContents(content, retriever.retrieve(uri)));
}

TEST(CatkinResourceRetrieverTests, CatkinIgnore)
{
  setenv("CMAKE_PREFIX_PATH", WORKSPACE_PATH, 1);

  const Uri uri = Uri::getUri("package://my_package3/ignored_file.txt");

  CatkinResourceRetriever retriever;
  EXPECT_FALSE(retriever.exists(uri));
  EXPECT_EQ(nullptr, retriever.retrieve(uri));
}

TEST(CatkinResourceRetrieverTests, NestedPackage)
{
  setenv("CMAKE_PREFIX_PATH", WORKSPACE_PATH, 1);

  const Uri uri = Uri::getUri("package://nested_package/nested_file.txt");

  CatkinResourceRetriever retriever;
  EXPECT_FALSE(retriever.exists(uri));
  EXPECT_EQ(nullptr, retriever.retrieve(uri));
}

TEST(CatkinResourceRetrieverTests, FileDoesNotExist)
{
  setenv("CMAKE_PREFIX_PATH", WORKSPACE_PATH, 1);

  const Uri uri = Uri::getUri("package://my_package1/does_not_exist.txt");

  CatkinResourceRetriever retriever;
  EXPECT_FALSE(retriever.exists(uri));
  EXPECT_EQ(nullptr, retriever.retrieve(uri));
}

TEST(CatkinResourceRetrieverTests, PackageDoesNotExist)
{
  setenv("CMAKE_PREFIX_PATH", WORKSPACE_PATH, 1);

  const Uri uri = Uri::getUri("package://does_not_exist/devel_only.txt");

  CatkinResourceRetriever retriever;
  EXPECT_FALSE(retriever.exists(uri));
  EXPECT_EQ(nullptr, retriever.retrieve(uri));
}

TEST(CatkinResourceRetrieverTests, DoesNotUseDirectoryName)
{
  setenv("CMAKE_PREFIX_PATH", WORKSPACE_PATH, 1);

  const Uri uri = Uri::getUri("package://my_package2/other_package.txt");

  CatkinResourceRetriever retriever;
  EXPECT_FALSE(retriever.exists(uri));
  EXPECT_EQ(nullptr, retriever.retrieve(uri));
}

TEST(CatkinResourceRetrieverTests, UsesPackageName)
{
  setenv("CMAKE_PREFIX_PATH", WORKSPACE_PATH, 1);

  const Uri uri = Uri::getUri("package://my_package2_actual/other_package.txt");
  const std::string content = "other_package\n";

  CatkinResourceRetriever retriever;
  EXPECT_TRUE(retriever.exists(uri));
  EXPECT_TRUE(CompareResourceContents(content, retriever.retrieve(uri)));
}

TEST(CatkinResourceRetrieverTests, WorkspaceDoesNotExist)
{
  setenv("CMAKE_PREFIX_PATH", "", 1);

  const Uri uri = Uri::getUri("package://my_package1/does_not_exist.txt");

  CatkinResourceRetriever retriever;
  EXPECT_FALSE(retriever.exists(uri));
  EXPECT_EQ(nullptr, retriever.retrieve(uri));
}
