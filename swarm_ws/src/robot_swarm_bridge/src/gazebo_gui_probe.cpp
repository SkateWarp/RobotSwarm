/*
 * Measures the camera that Gazebo Classic's desktop client is actually
 * drawing.  This is deliberately a gzclient plugin: sensor FPS and physics
 * real-time factor do not tell us whether the user's viewport is rendering.
 */

#include <GL/gl.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits.h>
#include <mutex>
#include <sstream>
#include <string>
#include <unistd.h>

#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/msgs/world_stats.pb.h>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/rendering/ogre_gazebo.h>
#include <gazebo/transport/transport.hh>

namespace gazebo
{
class RobotSwarmGuiProbe : public SystemPlugin
{
  public: RobotSwarmGuiProbe()
      : warmupSeconds(ReadPositiveNumber("ROBOTSWARM_GUI_PROBE_WARMUP", 2.0)),
        sampleSeconds(ReadPositiveNumber("ROBOTSWARM_GUI_PROBE_SECONDS", 5.0))
  {
    const char *requestedPath = std::getenv("ROBOTSWARM_GUI_PROBE_REPORT");
    this->reportPath = requestedPath && *requestedPath
        ? requestedPath
        : "/tmp/robotswarm-gazebo-gui-probe.json";
  }

  public: ~RobotSwarmGuiProbe() override
  {
    this->postRenderConnection.reset();
    this->worldStatsSubscriber.reset();
    this->worldTransportNode.reset();
    this->camera.reset();
  }

  public: void Load(int /*argc*/, char ** /*argv*/) override
  {
    this->postRenderConnection = event::Events::ConnectPostRender(
        std::bind(&RobotSwarmGuiProbe::OnPostRender, this));

    gzmsg << "RobotSwarm GUI probe will write " << this->reportPath
          << " after " << this->warmupSeconds << " s warmup and "
          << this->sampleSeconds << " s measurement.\n";
  }

  private: static double ReadPositiveNumber(
      const char *_name, const double _fallback)
  {
    const char *raw = std::getenv(_name);
    if (!raw || !*raw)
      return _fallback;

    char *end = nullptr;
    const double parsed = std::strtod(raw, &end);
    if (end == raw || *end != '\0' || !std::isfinite(parsed) || parsed <= 0.0)
    {
      gzwarn << "Ignoring invalid " << _name << " value [" << raw << "].\n";
      return _fallback;
    }
    return parsed;
  }

  private: static std::string JsonString(const std::string &_value)
  {
    std::ostringstream escaped;
    escaped << '"';
    for (const unsigned char character : _value)
    {
      switch (character)
      {
        case '"': escaped << "\\\""; break;
        case '\\': escaped << "\\\\"; break;
        case '\b': escaped << "\\b"; break;
        case '\f': escaped << "\\f"; break;
        case '\n': escaped << "\\n"; break;
        case '\r': escaped << "\\r"; break;
        case '\t': escaped << "\\t"; break;
        default:
          if (character < 0x20)
          {
            escaped << "\\u" << std::hex << std::setw(4)
                    << std::setfill('0') << static_cast<int>(character)
                    << std::dec << std::setfill(' ');
          }
          else
          {
            escaped << character;
          }
      }
    }
    escaped << '"';
    return escaped.str();
  }

  private: static std::string EnvironmentValue(const char *_name)
  {
    const char *value = std::getenv(_name);
    return value ? value : "";
  }

  private: static std::string ProcessExecutable()
  {
    char path[PATH_MAX + 1] = {0};
    const ssize_t size = readlink("/proc/self/exe", path, PATH_MAX);
    if (size <= 0)
      return "";
    path[size] = '\0';
    return path;
  }

  private: static std::string GlText(const GLenum _name)
  {
    const GLubyte *value = glGetString(_name);
    return value ? reinterpret_cast<const char *>(value) : "";
  }

  private: void ReadRendererIdentity()
  {
    if (!this->glRenderer.empty() && !this->renderDevice.empty())
      return;

    this->glVendor = GlText(GL_VENDOR);
    this->glRenderer = GlText(GL_RENDERER);
    this->glVersion = GlText(GL_VERSION);

    Ogre::Root *root = Ogre::Root::getSingletonPtr();
    if (!root || !root->getRenderSystem())
      return;

    Ogre::RenderSystem *renderSystem = root->getRenderSystem();
    this->renderApi = renderSystem->getName();
    const Ogre::RenderSystemCapabilities *capabilities =
        renderSystem->getCapabilities();
    if (!capabilities)
      return;

    this->renderDevice = capabilities->getDeviceName();
    this->renderVendor = Ogre::RenderSystemCapabilities::vendorToString(
        capabilities->getVendor());
  }

  private: void EnsureWorldStatsSubscriber()
  {
    if (this->worldStatsSubscriber)
      return;

    const std::string worldName = gui::get_world();
    if (worldName.empty())
      return;

    this->worldStatsTopic = "/gazebo/" + worldName + "/world_stats";
    this->worldTransportNode.reset(new transport::Node());
    this->worldTransportNode->Init(worldName);
    this->worldStatsSubscriber = this->worldTransportNode->Subscribe(
        "~/world_stats", &RobotSwarmGuiProbe::OnWorldStats, this);
  }

  private: static double MessageTimeSeconds(const msgs::Time &_time)
  {
    return static_cast<double>(_time.sec()) +
        static_cast<double>(_time.nsec()) * 1e-9;
  }

  private: void OnWorldStats(
      const boost::shared_ptr<const msgs::WorldStatistics> &_message)
  {
    if (!_message)
      return;

    const double simulationTime = MessageTimeSeconds(_message->sim_time());
    const double realTime = MessageTimeSeconds(_message->real_time());

    std::lock_guard<std::mutex> lock(this->performanceMutex);
    if (!this->collectPerformance)
      return;

    if (!this->haveWorldStatsBaseline)
    {
      this->lastSimulationTime = simulationTime;
      this->lastRealTime = realTime;
      this->haveWorldStatsBaseline = true;
      return;
    }

    const double simulationDelta = simulationTime - this->lastSimulationTime;
    const double realDelta = realTime - this->lastRealTime;
    this->lastSimulationTime = simulationTime;
    this->lastRealTime = realTime;
    if (!std::isfinite(simulationDelta) || !std::isfinite(realDelta) ||
        simulationDelta < 0.0 || realDelta <= 0.0)
    {
      return;
    }

    this->simulationTimeDelta += simulationDelta;
    this->realTimeDelta += realDelta;
    ++this->realTimeFactorSamples;
  }

  private: void OnPostRender()
  {
    if (this->reportWritten.load())
      return;

    if (!this->camera)
      this->camera = gui::get_active_camera();
    if (!this->camera || this->camera->ViewportWidth() == 0 ||
        this->camera->ViewportHeight() == 0)
    {
      return;
    }

    this->EnsureWorldStatsSubscriber();
    this->ReadRendererIdentity();
    const auto now = Clock::now();
    if (!this->cameraSeen)
    {
      this->cameraSeen = true;
      this->cameraSeenAt = now;
      return;
    }

    const double cameraAge = SecondsBetween(this->cameraSeenAt, now);
    if (!this->sampling)
    {
      if (cameraAge < this->warmupSeconds)
        return;

      this->sampling = true;
      this->sampleStartedAt = now;
      std::lock_guard<std::mutex> lock(this->performanceMutex);
      this->haveWorldStatsBaseline = false;
      this->collectPerformance = true;
      return;
    }

    const float averageFps = this->camera->AvgFPS();
    if (std::isfinite(averageFps) && averageFps > 0.0f)
    {
      this->cameraFpsSum += averageFps;
      ++this->cameraFpsSamples;
    }
    ++this->postRenderFrames;

    const double elapsed = SecondsBetween(this->sampleStartedAt, now);
    if (elapsed < this->sampleSeconds || this->cameraFpsSamples == 0)
      return;

    {
      std::lock_guard<std::mutex> lock(this->performanceMutex);
      this->collectPerformance = false;
    }
    this->WriteReport(elapsed);
  }

  private: void WriteReport(const double _elapsed)
  {
    const double measuredFps = this->cameraFpsSum / this->cameraFpsSamples;
    const double postRenderRate = this->postRenderFrames / _elapsed;

    double realTimeFactor = 0.0;
    size_t realTimeFactorSamples = 0;
    {
      std::lock_guard<std::mutex> lock(this->performanceMutex);
      realTimeFactorSamples = this->realTimeFactorSamples;
      if (realTimeFactorSamples > 0 && this->realTimeDelta > 0.0)
        realTimeFactor = this->simulationTimeDelta / this->realTimeDelta;
    }

    std::ostringstream report;
    report << std::fixed << std::setprecision(3)
           << "{\n"
           << "  \"schema_version\": 1,\n"
           << "  \"process\": {\"pid\": " << getpid()
           << ", \"executable\": " << JsonString(ProcessExecutable())
           << "},\n"
           << "  \"display\": {\"x11\": "
           << JsonString(EnvironmentValue("DISPLAY"))
           << ", \"wayland\": "
           << JsonString(EnvironmentValue("WAYLAND_DISPLAY")) << "},\n"
           << "  \"camera\": {\"name\": " << JsonString(this->camera->Name())
           << ", \"viewport_width\": " << this->camera->ViewportWidth()
           << ", \"viewport_height\": " << this->camera->ViewportHeight()
           << "},\n"
           << "  \"renderer\": {\"api\": " << JsonString(this->renderApi)
           << ", \"device\": " << JsonString(this->renderDevice)
           << ", \"vendor\": " << JsonString(this->renderVendor)
           << ", \"gl_vendor\": " << JsonString(this->glVendor)
           << ", \"gl_renderer\": " << JsonString(this->glRenderer)
           << ", \"gl_version\": " << JsonString(this->glVersion) << "},\n"
           << "  \"render_measurement\": {\"source\": "
           << JsonString("gazebo::rendering::Camera::AvgFPS")
           << ", \"warmup_seconds\": " << this->warmupSeconds
           << ", \"sample_seconds\": " << _elapsed
           << ", \"samples\": " << this->cameraFpsSamples
           << ", \"average_fps\": " << measuredFps
           << ", \"post_render_rate_fps\": " << postRenderRate << "},\n"
           << "  \"physics_measurement\": {\"source\": "
           << JsonString("gazebo.msgs.WorldStatistics delta(sim_time)/delta(real_time)")
           << ", \"topic\": " << JsonString(this->worldStatsTopic)
           << ", \"samples\": " << realTimeFactorSamples
           << ", \"real_time_factor\": ";
    if (realTimeFactorSamples > 0)
      report << realTimeFactor;
    else
      report << "null";
    report << "}\n}\n";

    const std::string temporaryPath = this->reportPath + "." +
        std::to_string(getpid()) + ".tmp";
    std::ofstream output(temporaryPath, std::ios::out | std::ios::trunc);
    output << report.str();
    output.close();

    if (!output || std::rename(temporaryPath.c_str(), this->reportPath.c_str()) != 0)
    {
      gzerr << "Could not write RobotSwarm GUI probe report ["
            << this->reportPath << "].\n";
      std::remove(temporaryPath.c_str());
      return;
    }

    this->reportWritten.store(true);
    gzmsg << "RobotSwarm GUI probe measured " << measuredFps
          << " rendered FPS; physics RTF is "
          << (realTimeFactorSamples > 0 ? std::to_string(realTimeFactor) : "unavailable")
          << ".\n";
  }

  private: using Clock = std::chrono::steady_clock;

  private: static double SecondsBetween(
      const Clock::time_point &_start, const Clock::time_point &_end)
  {
    return std::chrono::duration_cast<std::chrono::duration<double>>(
        _end - _start).count();
  }

  private: const double warmupSeconds;
  private: const double sampleSeconds;
  private: std::string reportPath;
  private: rendering::UserCameraPtr camera;
  private: event::ConnectionPtr postRenderConnection;
  private: transport::NodePtr worldTransportNode;
  private: transport::SubscriberPtr worldStatsSubscriber;
  private: std::string worldStatsTopic;
  private: bool cameraSeen = false;
  private: bool sampling = false;
  private: Clock::time_point cameraSeenAt;
  private: Clock::time_point sampleStartedAt;
  private: double cameraFpsSum = 0.0;
  private: size_t cameraFpsSamples = 0;
  private: size_t postRenderFrames = 0;
  private: std::string renderApi;
  private: std::string renderDevice;
  private: std::string renderVendor;
  private: std::string glVendor;
  private: std::string glRenderer;
  private: std::string glVersion;
  private: std::mutex performanceMutex;
  private: bool collectPerformance = false;
  private: bool haveWorldStatsBaseline = false;
  private: double lastSimulationTime = 0.0;
  private: double lastRealTime = 0.0;
  private: double simulationTimeDelta = 0.0;
  private: double realTimeDelta = 0.0;
  private: size_t realTimeFactorSamples = 0;
  private: std::atomic<bool> reportWritten{false};
};

GZ_REGISTER_SYSTEM_PLUGIN(RobotSwarmGuiProbe)
}
