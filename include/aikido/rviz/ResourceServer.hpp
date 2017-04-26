#ifndef AIKIDO_RVIZ_RESOURCESERVER_HPP_
#define AIKIDO_RVIZ_RESOURCESERVER_HPP_

#include <mutex>
#include <unordered_map>
#include <dart/dynamics/dynamics.hpp>
#include <microhttpd.h>

namespace aikido {
namespace rviz {

struct BinaryResource
{
  BinaryResource() : mSize(0), mData(nullptr)
  {
  }

  virtual ~BinaryResource()
  {
    if (mData)
    {
      delete[] mData;
    }
  }

  std::string mPath;
  size_t mSize;
  char* mData;
};

struct TextureResource : public BinaryResource
{
  virtual ~TextureResource() = default;
};

typedef std::shared_ptr<TextureResource> TextureResourcePtr;

struct MeshResource : public BinaryResource
{
  virtual ~MeshResource() = default;

  std::unordered_map<std::string, TextureResourcePtr> mTextures;
};

struct ResourceRequest
{
  std::shared_ptr<BinaryResource> resource;
};

class ResourceServer
{
public:
  ResourceServer();
  ResourceServer(const ResourceServer& other) = delete;
  ResourceServer& operator=(const ResourceServer& other) = delete;

  virtual ~ResourceServer();

  bool isRunning() const;
  unsigned short getPort() const;

  bool start(unsigned short port = 0);
  bool stop();

  std::string addMesh(const aiScene& scene, const std::string& scenePath);

private:
  typedef std::shared_ptr<MeshResource> MeshResourcePtr;
  typedef std::shared_ptr<BinaryResource> ResourcePtr;
  typedef std::weak_ptr<BinaryResource> WeakResourcePtr;

  struct MHD_Daemon* mDaemon;
  std::string mHost;
  unsigned short mPort;

  std::mutex mMutex;
  std::unordered_map<aiScene const*, MeshResourcePtr> mScenes;
  std::unordered_map<std::string, WeakResourcePtr> mResources;

  static bool hasBuggyAssimp();

  static int queueHttpError(
      struct MHD_Connection* connection,
      unsigned int code,
      const std::string& message);

  static ssize_t resourceReaderCallback(
      void* cls, uint64_t pos, char* buf, size_t max);

  static void resourceReaderFreeCallback(void* cls);

  static int processConnection(
      void* cls,
      struct MHD_Connection* connection,
      const char* url,
      const char* method,
      const char* version,
      const char* upload_data,
      long unsigned int* upload_data_size,
      void** ptr);

  std::string getMeshURI(const MeshResourcePtr& meshResource) const;
};

} // namespace rviz
} // namespace aikido

#endif
