#include <cstdlib>
#include <gtest/gtest.h>
#include <aikido/util/CatkinResourceRetriever.hpp>

#define STR_EXPAND(tok) #tok
#define STR(tok) STR_EXPAND(tok)

using aikido::util::CatkinResourceRetriever;
using dart::common::Uri;
using dart::common::ResourcePtr;

static constexpr auto WORKSPACE_PATH = STR(AIKIDO_TEST_WORKSPACE_PATH);

static testing::AssertionResult CompareResourceContents(
  const std::string& _expectedContent, const ResourcePtr& _resource)
{
  if (!_resource)
    return testing::AssertionFailure() << "Resource is nullptr";

  const size_t length = _resource->getSize();
  std::vector<char> content_buffer(length, '\0');
  const size_t read_length = _resource->read(content_buffer.data(), length, 1);
  if (read_length != 1)
    return testing::AssertionFailure() << "Failed reading Resource.";

  const std::string content(
    std::begin(content_buffer), std::end(content_buffer));
  if (content != _expectedContent)
    return testing::AssertionFailure() << "Got content \"" << content
      << "\", expected \"" << _expectedContent << "\".";

  return testing::AssertionSuccess();
}

class CatkinResourceRetrieverTests : public testing::Test
{
  void SetUp() override
  {
  }
};

TEST_F(CatkinResourceRetrieverTests, ExistsInDevelOnly)
{
  setenv("CMAKE_PREFIX_PATH", WORKSPACE_PATH, 1);

  const Uri uri = Uri::getUri("package://my_package1/devel_only.txt");
  const std::string content = "my_package1_devel_only\n";

  CatkinResourceRetriever retriever;
  EXPECT_TRUE(retriever.exists(uri));
  EXPECT_TRUE(CompareResourceContents(content, retriever.retrieve(uri)));
}

TEST_F(CatkinResourceRetrieverTests, ExistsInSourceOnly)
{
  setenv("CMAKE_PREFIX_PATH", WORKSPACE_PATH, 1);

  const Uri uri = Uri::getUri("package://my_package1/source_only.txt");
  const std::string content = "my_package1_source_only\n";

  CatkinResourceRetriever retriever;
  EXPECT_TRUE(retriever.exists(uri));
  EXPECT_TRUE(CompareResourceContents(content, retriever.retrieve(uri)));
}

TEST_F(CatkinResourceRetrieverTests, ExistsInDevelAndSource)
{
  setenv("CMAKE_PREFIX_PATH", WORKSPACE_PATH, 1);

  const Uri uri = Uri::getUri("package://my_package1/devel_and_source.txt");
  const std::string content = "my_package1_devel_and_source:devel\n";

  CatkinResourceRetriever retriever;
  EXPECT_TRUE(retriever.exists(uri));
  EXPECT_TRUE(CompareResourceContents(content, retriever.retrieve(uri)));
}

TEST_F(CatkinResourceRetrieverTests, FileDoesNotExist)
{
  setenv("CMAKE_PREFIX_PATH", WORKSPACE_PATH, 1);

  const Uri uri = Uri::getUri("package://my_package1/does_not_exist.txt");

  CatkinResourceRetriever retriever;
  EXPECT_FALSE(retriever.exists(uri));
  EXPECT_EQ(nullptr, retriever.retrieve(uri));
}

TEST_F(CatkinResourceRetrieverTests, PackageDoesNotExist)
{
  setenv("CMAKE_PREFIX_PATH", WORKSPACE_PATH, 1);

  const Uri uri = Uri::getUri("package://my_package3/devel_only.txt");

  CatkinResourceRetriever retriever;
  EXPECT_FALSE(retriever.exists(uri));
  EXPECT_EQ(nullptr, retriever.retrieve(uri));
}

TEST_F(CatkinResourceRetrieverTests, DoesNotUseDirectoryName)
{
  setenv("CMAKE_PREFIX_PATH", WORKSPACE_PATH, 1);

  const Uri uri = Uri::getUri("package://my_package2/other_package.txt");

  CatkinResourceRetriever retriever;
  EXPECT_FALSE(retriever.exists(uri));
  EXPECT_EQ(nullptr, retriever.retrieve(uri));
}

TEST_F(CatkinResourceRetrieverTests, UsesPackageName)
{
  setenv("CMAKE_PREFIX_PATH", WORKSPACE_PATH, 1);

  const Uri uri = Uri::getUri("package://my_package2_actual/other_package.txt");
  const std::string content = "other_package\n";

  CatkinResourceRetriever retriever;
  EXPECT_TRUE(retriever.exists(uri));
  EXPECT_TRUE(CompareResourceContents(content, retriever.retrieve(uri)));
}

TEST_F(CatkinResourceRetrieverTests, WorkspaceDoesNotExist)
{
  setenv("CMAKE_PREFIX_PATH", "", 1);

  const Uri uri = Uri::getUri("package://my_package1/does_not_exist.txt");

  CatkinResourceRetriever retriever;
  EXPECT_FALSE(retriever.exists(uri));
  EXPECT_EQ(nullptr, retriever.retrieve(uri));
}
