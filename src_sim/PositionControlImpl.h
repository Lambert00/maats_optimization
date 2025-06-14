#ifndef POSITION_CONTROL_IMPL_H
#define POSITION_CONTROL_IMPL_H

#include <Object.h>
#include <Vector3D.h>
#include <UdpSocket.h>
#include <Label.h>

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <map>
#include <thread>
#include <functional>

#include "AsyncServerConnector.h"

namespace flair {
namespace filter {

// Optimization server data packet
struct ServerDataPacket {
    uint32_t sequenceNumber;
    flair::core::Vector3Df direction;
    float tension;
    uint64_t timestamp;
    
    ServerDataPacket() : 
        sequenceNumber(0), 
        tension(0.0f), 
        timestamp(0) {}
    
    ServerDataPacket(uint32_t seq, const flair::core::Vector3Df& dir, float t, uint64_t time) : 
        sequenceNumber(seq), 
        direction(dir), 
        tension(t), 
        timestamp(time) {}
};

// Connection configuration
struct ServerConfig {
    std::string serverAddress;
    int serverPort;
    int resultPort;
    
    ServerConfig() : 
        serverAddress("127.0.0.1"), 
        serverPort(5555), 
        resultPort(5556) {}
    
    ServerConfig(const std::string& addr, int port, int resultP) : 
        serverAddress(addr), 
        serverPort(port), 
        resultPort(resultP) {}
};

// Position control implementation
class PositionControlImpl {
public:
    // Constructor
    PositionControlImpl(const flair::core::Object* parent, 
                       const std::string& name,
                       uint16_t uavIndex,
                       const ServerConfig& config);
    
    // Destructor
    ~PositionControlImpl();
    
    // Initialize server connection
    bool InitializeServerConnection();
    
    // Test server connection
    bool TestServerConnection(int timeoutMs = 500);
    
    // Send resultant force to server (master only)
    bool SendResultantForce(const flair::core::Vector3Df& resultantForce);
    
    // Process data received from server
    bool ProcessServerData(uint32_t sequence, 
                          const flair::core::Vector3Df& direction, 
                          float tension);
    
    // Start UDP receiver for unicast messages
    bool StartUdpReceiver(int port);
    
    // Get current server data
    ServerDataPacket GetCurrentData() const;
    
    // Get server connection status
    bool IsConnected() const;
    
    // Force reconnection (useful for testing)
    void ForceReconnect();
    
    // Set connection callback
    void SetConnectionCallback(std::function<void(bool)> callback);
    
    // Update connection status display (UI)
    void UpdateConnectionStatusDisplay(flair::gui::Label* statusLabel);
    
    // Check and update connection status
    void CheckConnectionStatus();
    
    // Calculate checksum for binary protocols
    static uint16_t CalculateChecksum(const void* data, size_t length);
    
private:
    // Server connector for TCP communication
    std::unique_ptr<AsyncServerConnector> serverConnector;
    
    // UDP socket for receiving unicast messages
    std::unique_ptr<flair::core::UdpSocket> unicastReceiver;
    
    // Thread for UDP message handling
    std::thread udpReceiverThread;
    std::atomic<bool> receiverRunning;
    
    // Server configuration
    ServerConfig config;
    
    // UAV index
    uint16_t uavIndex;
    
    // Current data
    mutable std::mutex dataMutex;
    ServerDataPacket currentData;
    ServerDataPacket lastValidData;
    bool hasValidData;
    
    // Request times for optimization timing
    std::map<uint32_t, uint64_t> requestTimes;
    
    // Timestamps for connection checks
    uint64_t lastStatusUpdateMs;
    uint64_t lastHeartbeatTimeMs;
    
    // Server connection status
    std::atomic<bool> connected;
    
    // Parent object and name
    const flair::core::Object* parent;
    std::string name;
    
    // UDP receiver loop function
    void UdpReceiverLoop();
    
    // Send heartbeat message (master only)
    void SendHeartbeatMessage();
    
    // Magic numbers for binary protocols
    static constexpr uint32_t REQUEST_MAGIC = 0xDECA0001;
    static constexpr uint32_t BROADCAST_MAGIC = 0xDECA0002;
    static constexpr uint32_t HEARTBEAT_MAGIC = 0xDECA0003;
    static constexpr uint32_t UNICAST_MAGIC = 0xDECA0005;
    
    // Get current time in milliseconds
    uint64_t GetCurrentTimeMs() const;
};

}} // namespace flair::filter

#endif // POSITION_CONTROL_IMPL_H