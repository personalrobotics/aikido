#include <fstream>
#include <assimp/cexport.h>
#include <assimp/version.h>
#include <ros/network.h>
#include <boost/filesystem.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <dart_rviz/ResourceServer.h>

using aikido::rviz::ResourceServer;

static void getTextures(aiScene const &scene, std::string const &scenePath,
                         std::vector<std::pair<std::string, std::string> > *textures)
{
  using boost::filesystem::path;

  static std::vector<aiTextureType> const materialTypes {
    aiTextureType_AMBIENT,
    aiTextureType_DIFFUSE,
    aiTextureType_DISPLACEMENT,
    aiTextureType_EMISSIVE,
    aiTextureType_HEIGHT,
    aiTextureType_LIGHTMAP,
    aiTextureType_NONE,
    aiTextureType_NORMALS,
    aiTextureType_OPACITY,
    aiTextureType_REFLECTION,
    aiTextureType_SHININESS,
    aiTextureType_SPECULAR,
    aiTextureType_UNKNOWN
  };

  if (scenePath.empty()) {
    return;
  }

  path const basePath = path(scenePath).parent_path();

  for (unsigned int imesh = 0; imesh < scene.mNumMeshes; ++imesh) {
    aiMesh const &mesh = *scene.mMeshes[imesh];
    aiMaterial const &material = *scene.mMaterials[mesh.mMaterialIndex];

    for (aiTextureType const &textureType : materialTypes) {
      unsigned int numTextures = material.GetTextureCount(textureType);

      for (unsigned int itexture = 0; itexture < numTextures; ++itexture) {
        aiString textureAssimpPath;
        material.GetTexture(textureType, itexture, &textureAssimpPath, nullptr);

        std::string const textureRelativePath(textureAssimpPath.C_Str());
        path const texturePath = (basePath / textureRelativePath).normalize();

        textures->push_back(
          std::make_pair(textureRelativePath, texturePath.string())
        );
      }
    }
  }
}

ResourceServer::ResourceServer()
  : mDaemon(nullptr)
  , mHost(ros::network::getHost())
  , mPort(0)
{
}

ResourceServer::~ResourceServer()
{
  stop();
}

bool ResourceServer::isRunning() const
{
  return !!mDaemon;
}

unsigned short ResourceServer::getPort() const
{
  return mPort;
}

bool ResourceServer::start(unsigned short port)
{
  if (mDaemon) {
    return false;
  }

  // Pass zero to start on an arbitrary port.
  mDaemon = MHD_start_daemon(
    MHD_USE_SELECT_INTERNALLY, port,
    nullptr, nullptr, // connections filter
    &ResourceServer::processConnection, this, // connection callback
    MHD_OPTION_END
  );

  // Retrieve the port from the MHD daemon.
  if (mDaemon) {
    union MHD_DaemonInfo const *daemonInfo = MHD_get_daemon_info(
      mDaemon, MHD_DAEMON_INFO_LISTEN_FD);
    if (!daemonInfo) {
      ROS_ERROR("Unable to get daemon information.");
      return false;
    }

    struct sockaddr_storage addr;
    socklen_t len = sizeof addr;

    if (getsockname(daemonInfo->listen_fd,
                    reinterpret_cast<struct sockaddr *>(&addr), &len)) {
      ROS_ERROR_STREAM("Getting port failed on socket descriptor "
                       << daemonInfo->listen_fd << ".");
      return false;
    }

    if (addr.ss_family == AF_INET) {
      auto s = reinterpret_cast<struct sockaddr_in *>(&addr);
      mPort = ntohs(s->sin_port);
    } else if (addr.ss_family == AF_INET6) {
      auto s = reinterpret_cast<struct sockaddr_in6 *>(&addr);
      mPort = ntohs(s->sin6_port);
    } else {
      ROS_ERROR("Unknown socket family.");
      return false;
    }

    ROS_DEBUG_STREAM("Started server on port " << mPort);
    return true;
  } else {
    std::cerr << "Failed starting HTTP server on port " << port << "!" << std::endl;
    return false;
  }
}

bool ResourceServer::stop()
{
  if (!mDaemon) {
    return false;
  }

  MHD_stop_daemon(mDaemon);

  mDaemon = nullptr;
  mPort = 0;
  return true;
}

std::string ResourceServer::addMesh(aiScene const &inputScene,
                                    std::string const &scenePath)
{
  std::lock_guard<std::mutex> lock(mMutex);

  // Return the existing mesh. Otherwise, we'll export a fresh copy.
  auto const sceneIt = mScenes.find(&inputScene);
  if (sceneIt != std::end(mScenes)) {
    return getMeshURI(sceneIt->second);
  }

  // Handle a scaling bug in Assimp < 3.1.
  aiScene const *scene = &inputScene;
  if (hasBuggyAssimp()) {
    aiScene *newScene;
    aiCopyScene(scene, &newScene);

    // Assimp < 3.1 hard-codes units to centimeters. This introduces a scale
    // factor of 0.01 that should not be present. We scale the mesh up by a
    // factor of 100 to compensate for this.
    newScene->mRootNode->mTransformation = aiMatrix4x4::Scaling(
      aiVector3D(100.), inputScene.mRootNode->mTransformation);

    scene = newScene;
  }

  // Export the mesh for the first time.
  aiExportDataBlob const *sceneBlob = aiExportSceneToBlob(scene, "collada", 0);

  if (scene != &inputScene) {
    delete scene;
  }

  if (sceneBlob->name.length != 0 || sceneBlob->next != nullptr) {
    ROS_ERROR_STREAM("Failed to export scene '" << scenePath << "'.");
    aiReleaseExportBlob(sceneBlob);
    return "";
  }

  // Add the mesh to the resource server.
  auto sceneResource = std::make_shared<MeshResource>();
  sceneResource->mPath = scenePath;
  sceneResource->mData = new char[sceneBlob->size];
  sceneResource->mSize = sceneBlob->size;
  memcpy(sceneResource->mData, sceneBlob->data, sceneBlob->size);
  aiReleaseExportBlob(sceneBlob);

  // Also add any texture files referenced by the scene.
  std::vector<std::pair<std::string, std::string> > texturePaths;
  getTextures(inputScene, scenePath, &texturePaths);

  for (std::pair<std::string, std::string> const &textureIt : texturePaths) {
    std::string const &relativePath = textureIt.first;
    std::string const &absolutePath = textureIt.second;

    // Check if the texture is already loaded.
    auto const resourceIt = mResources.find(absolutePath);
    if (resourceIt != std::end(mResources) && resourceIt->second.lock()) {
      continue;
    }

    // Load the texture from disk.
    std::ifstream textureStream(absolutePath.c_str(), std::ios::binary);
    std::streampos textureSize = textureStream.tellg();
    textureStream.seekg(0, std::ios::end);
    textureSize = textureStream.tellg() - textureSize;
    textureStream.seekg(0, std::ios::beg);

    auto const textureResource = std::make_shared<TextureResource>();
    textureResource->mPath = absolutePath;
    textureResource->mSize = textureSize;
    textureResource->mData = new char[textureSize];
    textureStream.read(textureResource->mData, textureSize);

    if (!textureStream.good()) {
      ROS_ERROR_STREAM("Failed loading texture '" << absolutePath << "'.");
      continue;
    }

    sceneResource->mTextures[relativePath] = textureResource;
    mResources[absolutePath] = textureResource;
  }

  mScenes[&inputScene] = sceneResource;
  mResources[scenePath] = sceneResource;
  return getMeshURI(sceneResource);
}

std::string ResourceServer::getMeshURI(MeshResourcePtr const &meshResource) const
{
  assert(!!meshResource);

  if (!mDaemon) {
    ROS_ERROR("Requested mesh URI with no HTTP server running.");
    return "";
  }

  std::stringstream ss;
  ss << "http://" << mHost << ":" << mPort << meshResource->mPath;
  return ss.str();
}

bool ResourceServer::hasBuggyAssimp()
{
  // Assimp has a buggy export in < 3.1
  return (aiGetVersionMajor() <= 3 || aiGetVersionMinor() < 1);
}

int ResourceServer::queueHttpError(struct MHD_Connection *connection,
                                   unsigned int code,
                                   std::string const &message)
{
  struct MHD_Response *response = MHD_create_response_from_buffer(
    message.size(), const_cast<char *>(message.c_str()),
    MHD_RESPMEM_MUST_COPY
  );
  int const result = MHD_queue_response(connection, code, response);
  MHD_destroy_response(response);
  return result;
}

ssize_t ResourceServer::resourceReaderCallback(void *cls, uint64_t pos,
                                               char *buf, size_t max)
{
  auto resourceRequest = reinterpret_cast<ResourceRequest *>(cls);

  if (pos + max > resourceRequest->resource->mSize) {
    return MHD_CONTENT_READER_END_WITH_ERROR;
  }

  memcpy(buf, resourceRequest->resource->mData + pos, max);
  return max;
}

void ResourceServer::resourceReaderFreeCallback(void *cls)
{
  delete reinterpret_cast<ResourceRequest *>(cls);
}

int ResourceServer::processConnection(
  void *cls, struct MHD_Connection *connection, char const *url,
  char const *method, char const *version, char const *upload_data,
  long unsigned int *upload_data_size, void **ptr)
{
  static int headerReceived = 0;

  auto *server = static_cast<ResourceServer *>(cls);

  // This function is called twice per connection. First, it is called only
  // with headers (but no body) and we return MHD_YES. We indicate this by
  // changing our context in *ptr, but do not reply.
  if (*ptr != &headerReceived) {
    *ptr = &headerReceived;
    return MHD_YES;
  }

  // Next, it is called with the full request. Now we reply.
  if (std::string(method) != "GET") {
    ROS_WARN_STREAM("Invalid method '" << method << "':" << url);
    return queueHttpError(connection, MHD_HTTP_NOT_IMPLEMENTED,
                          "Only GET is implemented.");
  }
  if (*upload_data_size != 0) {
    ROS_ERROR("Malformed request with non-zero upload data size.");
    return queueHttpError(connection, MHD_HTTP_BAD_REQUEST,
                          "Upload data size must be zero.");
  }

  // Lookup the resource (while locking mResources).
  auto *resourceRequest = new ResourceRequest;
  {
    std::lock_guard<std::mutex> lock(server->mMutex);

    auto it = server->mResources.find(url);
    if (it != std::end(server->mResources)) {
      resourceRequest->resource = it->second.lock();
    }
  }

  // There is no resource with this URI.
  if (!resourceRequest->resource) {
    ROS_WARN_STREAM("Invalid access to: " << url);
    return queueHttpError(connection, MHD_HTTP_NOT_FOUND,
                          "Resource not found.");
  }

  // Respond with the resource. We use the callback form to temporarily lock
  // the BinaryResource shared_ptr to prevent it from being destructed while
  // we are responding.
  ROS_DEBUG_STREAM("Accessed: " << url);

  struct MHD_Response *response = MHD_create_response_from_callback(
    resourceRequest->resource->mSize, resourceRequest->resource->mSize,
    &ResourceServer::resourceReaderCallback, resourceRequest,
    &ResourceServer::resourceReaderFreeCallback
  );
  int const result = MHD_queue_response(connection, MHD_HTTP_OK, response);
  MHD_destroy_response(response);

  return result;
}
