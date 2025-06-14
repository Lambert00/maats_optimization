#ifndef POSITIONCONTROL_H
#define POSITIONCONTROL_H

#include <IODevice.h>
#include <Vector3D.h>
#include <mutex>
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>
#include <memory>
#include <queue>
#include <map>

// Forward declare SharedParameters to avoid circular dependency
namespace flair {
  namespace core {
    class Matrix;
    class UdpSocket;
    class SharedParameters;
  }
  namespace gui {
    class DataPlot1D;
    class Label;
  }
  namespace meta {
    class MetaVrpnObject;
  }
  namespace filter {
    class Pid;
    class TrajectoryGenerator2DCircle;
  }
}

// Forward declarations
namespace flair {
namespace filter {
  class TcpClient;
  class PositionControlImpl;
}}

namespace flair {
namespace filter {

/**
 * @brief Class defining a control law for multi-agent aerial transportation
 * with direct server communication
 */
class PositionControl : public core::IODevice {
public:
  /**
   * @brief Constructor
   *
   * @param name Name of the controller
   * @param sharedParameters Shared parameters between UAVs
   * @param uavIndex Index of the UAV in the group
   * @param allUavs Vector of all UAVs in the group
   * @param load The load object
   * @param circle Trajectory generator for circular paths
   */
  PositionControl(std::string name,
                core::SharedParameters *sharedParameters,
                uint16_t uavIndex,
                std::vector<const meta::MetaVrpnObject*> allUavs,
                meta::MetaVrpnObject *load,
                TrajectoryGenerator2DCircle *circle);

  /**
   * @brief Destructor
   */
  ~PositionControl();
  
  /**
   * @brief Behavior modes for the position controller
   */
  enum class BehaviourMode_t {
      GoToPosition,     ///< Hold at a fixed position
      CircleTrajectory, ///< Follow a circular trajectory
  };
  
  /**
   * @brief Tension distribution modes
   */
  enum class TensionDistributionMode_t {
      FixedDistribution,     ///< Use fixed symmetric distribution
      OptimizedDistribution  ///< Use optimizer service for distribution
  };

  /**
   * @brief Data source for tension and direction values
   */
  enum class DataSource {
      None,                   ///< No valid data source
      ServerDirect,           ///< Direct from optimization server
      ServerFallback,         ///< Fallback from server data
      Symmetric               ///< Symmetric distribution (no optimization)
  };

  /**
   * @brief Output desired thrust
   *
   * @return thrust value
   */
  float DesiredThrust(void) const;
  
  /**
   * @brief Output desired quaternion
   *
   * @return quaternion value
   */
  flair::core::Quaternion DesiredQuaternion(void) const;
  
  /**
   * @brief Output desired omega
   *
   * @return omega value
   */
  flair::core::Vector3Df DesiredOmega(void) const;
  
  /**
   * @brief Set reference mode
   *
   * @param behaviourMode Reference mode
   */
  void SetRefMode(BehaviourMode_t behaviourMode);
  
  /**
   * @brief Reset all integral parts
   */
  void ResetI(void);
  
  /**
   * @brief Update control
   */
  void Update(void);
  
  /**
   * @brief Test connection to the optimization server
   *
   * @param timeoutMs Timeout in milliseconds
   * @return true if connection successful
   */
  bool TestServerConnection(int timeoutMs = 500);
  
  /**
   * @brief Check if server connection is active
   * 
   * @return true if connected to server
   */
  bool IsServerConnected() const;
  
  /**
   * @brief Set the tension distribution mode
   *
   * @param mode Distribution mode
   * @return true if mode was successfully set
   */
  bool SetTensionDistributionMode(TensionDistributionMode_t mode);
  
  /**
   * @brief Get current tension distribution mode
   *
   * @return Distribution mode
   */
  TensionDistributionMode_t GetTensionDistributionMode() const;
  
  /**
   * @brief Force reconnection to server
   * 
   * Diagnostic function to manually trigger reconnection
   */
  void ForceReconnect();

private:
  /**
   * @brief Update control using data
   *
   * @param data Input data
   */
  void UpdateFrom(const core::io_data *data);
  
  /**
   * @brief Initialize direct server communication 
   * 
   * @return true if successfully initialized
   */
  bool InitializeServerConnection();
  
  /**
   * @brief Process optimization data from server
   * 
   * @return true if new data received
   */
  bool ProcessOptimizationData();
  
  /**
   * @brief Send load resultant force to server (master only)
   * 
   * @param resultantForce Load resultant force
   * @return true if successfully sent
   */
  bool SendResultantForce(const flair::core::Vector3Df& resultantForce);
  
  /**
   * @brief Initialize symmetric distribution as fallback
   */
  void InitializeSymmetricDistribution();

  /**
   * @brief Calculate symmetric distribution 
   * Implemented differently for master and slave drones
   * 
   * @param directions Array of direction vectors for all drones
   * @param tensions Array of tension values for all drones
   */
  void CalculateSymmetricDistribution(std::vector<flair::core::Vector3Df>& directions, 
                                     std::vector<float>& tensions);
                                     
  /**
   * @brief Calculate symmetric distribution with pre-calculated ul
   * Overloaded version that uses a pre-calculated resultant force
   * 
   * @param directions Array of direction vectors for all drones
   * @param tensions Array of tension values for all drones
   * @param ul Pre-calculated resultant force
   */
  void CalculateSymmetricDistributionWithUl(std::vector<flair::core::Vector3Df>& directions, 
                                           std::vector<float>& tensions,
                                           const flair::core::Vector3Df& ul);
  
  /**
   * @brief Calculate load control using PID controllers
   * 
   * @param loadPos Current load position
   * @param loadSpeed Current load velocity
   * @param loadPosD Desired load position
   * @param loadSpeedD Desired load velocity
   * @return Resultant force on load
   */
  flair::core::Vector3Df CalculateLoadControl(
      const flair::core::Vector3Df& loadPos,
      const flair::core::Vector3Df& loadSpeed,
      const flair::core::Vector3Df& loadPosD,
      const flair::core::Vector3Df& loadSpeedD);
      
  /**
   * @brief Update tension visualization in UI
   */
  void UpdateTensionVisualization();
  
  /**
   * @brief Calculate cable angles between adjacent drones
   * Only performed by master drone
   * 
   * @param directions Vector of cable directions
   * @param angles Output vector for angles (in degrees)
   */
  void CalculateCableAngles(const std::vector<flair::core::Vector3Df>& directions, 
                            std::vector<float>& angles);
  
  /**
   * @brief Calculate and update actual cable angles based on positions
   * Only performed by master drone
   */
  void CalculateActualCableAngles();
  
  /**
   * @brief Calculate dot product of two vectors
   * 
   * @param v1 First vector
   * @param v2 Second vector
   * @return Dot product result
   */
  float DotProduct(const flair::core::Vector3Df& v1, 
                  const flair::core::Vector3Df& v2) const {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
  }
  
  /**
   * @brief Calculate checksum for binary messages
   * 
   * @param data Message data
   * @param size Message size (excluding checksum field)
   * @return Checksum value
   */
  static uint16_t CalculateChecksum(const void* data, size_t size);
  
  /**
   * @brief Start UDP receiver thread to listen for optimization results
   */
  void StartUdpReceiverThread();
  
  /**
   * @brief Send heartbeat message to server to maintain connection (master only)
   */
  void SendHeartbeatMessage();
  
  /**
   * @brief UDP receiver thread function
   */
  void UdpReceiverLoop();
  
  /**
   * @brief Process unicast message from server
   * 
   * @param sequence Sequence number
   * @param direction Direction vector
   * @param tension Tension value
   * @return true if processed successfully
   */
  bool ProcessUnicastData(uint32_t sequence, const flair::core::Vector3Df& direction, float tension);
  
  /**
   * @brief Update display matrix with current distribution data
   * 
   * @param sequence Sequence number
   */
  void UpdateDistributionMatrix(uint32_t sequence);
  
  /**
   * @brief Get current time in milliseconds
   * 
   * @return Current time
   */
  uint64_t GetCurrentTimeMs() const;
  
  /**
   * @brief Check and update connection status
   * 
   * Periodically checks connection status and tries to recover if needed
   */
  void CheckConnectionStatus();
  
  /**
   * @brief Update connection status display in UI
   */
  void UpdateConnectionStatusDisplay();

  // Data members
  core::SharedParameters* sharedParameters;  ///< Shared parameters
  meta::MetaVrpnObject* load;               ///< Load object
  TrajectoryGenerator2DCircle* circle;      ///< Trajectory generator
  uint16_t uavIndex;                        ///< UAV index
  std::vector<const meta::MetaVrpnObject*> allUavs; ///< All UAVs
  
  // Binary communication protocol structures
  #pragma pack(push, 1)
  
  // TCP request message format (matches server format 0xDECA0001)
  struct TcpRequestMessage {
      uint32_t magic;               // Magic number for validation (0xDECA0001)
      uint32_t sequenceNumber;      // Message sequence ID
      uint32_t droneId;             // Source drone ID (typically 0 for master)
      uint64_t timestamp;           // Message creation time (milliseconds)
      float resultantForce[3];      // Desired resultant force vector
      uint16_t checksum;            // Message integrity check
  };
  
  // TCP heartbeat message format (matches server format 0xDECA0003)
  struct TcpHeartbeatMessage {
      uint32_t magic;               // Magic number for validation (0xDECA0003)
      uint32_t droneId;             // Source drone ID
      uint64_t timestamp;           // Message creation time (milliseconds)
      uint16_t checksum;            // Message integrity check
  };
  
  // Unicast message format (matches server format 0xDECA0005)
  struct UdpUnicastMessage {
      uint32_t magic;               // Magic number for validation (0xDECA0005)
      uint32_t sequenceNumber;      // Message sequence ID
      uint8_t droneIndex;           // Target drone index (0-255)
      float direction[3];           // Direction vector [x,y,z]
      float tension;                // Tension value in Newtons
      uint16_t checksum;            // Message integrity check
  };
  
  // Legacy broadcast message format (for backward compatibility)
  struct UdpBroadcastMessage {
      uint32_t magic;               // Magic number for validation (0xDECA0002)
      uint32_t sequenceNumber;      // Message sequence ID (matches request)
      uint32_t droneCount;          // Total number of drones
      uint64_t timestamp;           // Message creation time (milliseconds)
      float directions[10][3];      // Cable direction vectors for each drone (up to 10)
      float tensions[10];           // Cable tension values for each drone (up to 10)
      uint16_t checksum;            // Message integrity check
  };
  #pragma pack(pop)
  
  // Control matrices
  core::Matrix* output;               ///< Output matrix
  core::Matrix* state;                ///< State matrix
  core::Matrix* angleRef;             ///< Angle reference matrix
  core::Matrix* loadRef;              ///< Load reference matrix
  core::Matrix* tensionData;          ///< Tension data matrix
  core::Matrix* cableAnglesData;      ///< Cable angles matrix
  core::Matrix* actualCableAnglesData; ///< Actual cable angles matrix
  core::Matrix* distributionMatrix;   ///< Distribution visualization matrix
  core::Matrix* optimTimeMatrix;      ///< Optimization time matrix (master only)
  
  // PID controllers
  Pid* pidxUav;                       ///< X axis UAV PID
  Pid* pidyUav;                       ///< Y axis UAV PID
  Pid* pidzUav;                       ///< Z axis UAV PID
  Pid* pidxLoad;                      ///< X axis Load PID
  Pid* pidyLoad;                      ///< Y axis Load PID
  Pid* pidzLoad;                      ///< Z axis Load PID
  
  // UI elements
  gui::Label* serverStatusLabel;      ///< Server connection status
  gui::Label* distributionStatusLabel; ///< Distribution status
  
  // Behavior settings
  BehaviourMode_t behaviourMode;      ///< Current behavior mode
  TensionDistributionMode_t tensionMode; ///< Current tension distribution mode
  
  // Unicast socket and thread management
  std::unique_ptr<core::UdpSocket> unicastReceiver; ///< Socket for unicast reception
  std::thread udpReceiverThread;                   ///< Thread for UDP receiver
  std::atomic<bool> receiverRunning;               ///< Flag for receiver thread
  
  // TCP client for sending requests to server
  std::unique_ptr<TcpClient> tcpClient; ///< TCP client for requests
  
  // Concurrency control
  mutable std::mutex optimizationMutex; ///< Mutex for optimization data
  
  // Timing variables
  uint64_t lastUpdateTimeMs;          ///< Last update time
  uint64_t lastRequestTimeMs;         ///< Last request time
  uint64_t lastResultTimeMs;          ///< Last result time
  uint64_t lastStatusUpdateMs;        ///< Last status update time
  uint64_t lastHeartbeatTimeMs;       ///< Last heartbeat time
  uint64_t lastConnectionCheckMs;     ///< Last connection check time
  
  // Optimization time tracking
  std::map<uint32_t, uint64_t> requestTimes; ///< Map of request times by sequence number
  
  // Connection parameters
  int connectionTimeoutMs;            ///< Current connection timeout
  
  // Optimization result data
  DataSource currentDataSource;       ///< Current data source
  flair::core::Vector3Df myDirection; ///< Current cable direction
  float myTension;                    ///< Current cable tension
  
  // Fallback data
  flair::core::Vector3Df lastValidDirection; ///< Last valid direction
  float lastValidTension;             ///< Last valid tension
  bool hasValidData;                  ///< Whether valid data is available
  
  // Server connection status
  std::atomic<bool> serverConnected;  ///< Whether server is connected
  
  // Implementation class for server communication (optional)
  std::unique_ptr<PositionControlImpl> impl;  ///< Implementation helper
  
  // Protocol constants
  static constexpr uint32_t REQUEST_MAGIC = 0xDECA0001;   ///< Request magic number
  static constexpr uint32_t BROADCAST_MAGIC = 0xDECA0002; ///< Broadcast magic number
  static constexpr uint32_t HEARTBEAT_MAGIC = 0xDECA0003; ///< Heartbeat magic number
  static constexpr uint32_t UNICAST_MAGIC = 0xDECA0005;   ///< Unicast magic number
  static constexpr int MAX_DRONES = 10;                   ///< Maximum supported drones
  
  // Communication parameters
  static constexpr int HEARTBEAT_INTERVAL_MS = 1000;     ///< Heartbeat interval
  static constexpr int RECONNECT_INTERVAL_MS = 5000;     ///< Reconnection interval
  static constexpr int RESULT_TIMEOUT_MS = 5000;         ///< Result timeout
  
  // Default ports
  static constexpr int DEFAULT_SERVER_PORT = 5555;       ///< Default server port
  static constexpr int DEFAULT_RESULT_PORT = 5556;       ///< Default result port
};

} // end namespace filter
} // end namespace flair
#endif // POSITIONCONTROL_H