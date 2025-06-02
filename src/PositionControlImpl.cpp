//  created:    2025/05/05
//  filename:   PositionControlImpl.cpp
//
//  author:     Lamberto Vazquez
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    5.1
//
//  purpose:    Implementation of position control with robust server connection
//
//
/*********************************************************************/

#include "PositionControlImpl.h"
#include "AsyncServerConnector.h"

#include <UdpSocket.h>
#include <Vector3D.h>
#include <Object.h>
#include <FrameworkManager.h>
#include <Label.h>

#include <cstring>
#include <algorithm>
#include <sstream>
#include <thread>
#include <chrono>

namespace flair {
namespace filter {

using namespace flair::core;
using namespace flair::gui;

// Binary structure definitions for server communication
#pragma pack(push, 1)

// TCP request message format
struct TcpRequestMessage {
    uint32_t magic;               // Magic number for validation (0xDECA0001)
    uint32_t sequenceNumber;      // Message sequence ID
    uint32_t droneId;             // Source drone ID (typically 0 for master)
    uint64_t timestamp;           // Message creation time (milliseconds)
    float resultantForce[3];      // Desired resultant force vector
    uint16_t checksum;            // Message integrity check
};

// TCP heartbeat message format
struct TcpHeartbeatMessage {
    uint32_t magic;               // Magic number for validation (0xDECA0003)
    uint32_t droneId;             // Source drone ID
    uint64_t timestamp;           // Message creation time (milliseconds)
    uint16_t checksum;            // Message integrity check
};

// Unicast message format
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

//------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------
PositionControlImpl::PositionControlImpl(const Object* parent, 
                                       const std::string& name,
                                       uint16_t uavIndex,
                                       const ServerConfig& config)
    : serverConnector(nullptr),
      unicastReceiver(nullptr),
      receiverRunning(false),
      config(config),
      uavIndex(uavIndex),
      hasValidData(false),
      lastStatusUpdateMs(0),
      lastHeartbeatTimeMs(0),
      connected(false),
      parent(parent),
      name(name)
{
    // Get current time
    uint64_t currentTime = GetCurrentTimeMs();
    lastStatusUpdateMs = currentTime;
    lastHeartbeatTimeMs = currentTime;
    
    // Create server connector
    serverConnector.reset(new AsyncServerConnector(
        parent, name + "_connector", config.serverAddress, config.serverPort));
    
    // Start the connector thread for background connection management
    serverConnector->Start();
    
    // Set default callback for connection state changes
    serverConnector->SetConnectionCallback([this](bool state) {
        connected.store(state);
    });
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
PositionControlImpl::~PositionControlImpl() {
    // First signal thread to stop
    receiverRunning.store(false);
    
    // First stop the server connector - this is important to do before other cleanup
    if (serverConnector) {
        try {
            serverConnector->Stop();
            serverConnector.reset();
        } catch (...) {
            // Ignore errors during shutdown
        }
    }
    
    // Clear the socket to unblock any blocked receiver thread
    try {
        unicastReceiver.reset();
    } catch (...) {
        // Ignore errors during shutdown
    }
    
    // Give the thread time to notice the flag change
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Handle the UDP receiver thread with care
    if (udpReceiverThread.joinable()) {
        try {
            // Set a deadline for thread termination
            auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
            
            // Try to cancel if the thread is stuck
            while (udpReceiverThread.joinable() && 
                  std::chrono::steady_clock::now() < deadline) {
                
                // For stuck threads, try cancellation (non-standard but effective)
                pthread_t native_handle = udpReceiverThread.native_handle();
                pthread_cancel(native_handle);
                
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            
            // Final join
            if (udpReceiverThread.joinable()) {
                udpReceiverThread.join();
            }
        } catch (...) {
            // Ignore all errors during shutdown
        }
    }
}

//------------------------------------------------------------------------------
// Get current time in milliseconds
//------------------------------------------------------------------------------
uint64_t PositionControlImpl::GetCurrentTimeMs() const {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
}

//------------------------------------------------------------------------------
// Calculate message checksum
//------------------------------------------------------------------------------
uint16_t PositionControlImpl::CalculateChecksum(const void* data, size_t length) {
    const uint8_t* buffer = reinterpret_cast<const uint8_t*>(data);
    uint16_t checksum = 0;
    
    // XOR-based checksum calculation
    for (size_t i = 0; i < length; i += 2) {
        uint16_t value = 0;
        
        if (i + 1 < length) {
            // Two bytes available
            uint8_t byte1 = buffer[i];     // High byte
            uint8_t byte2 = buffer[i + 1]; // Low byte
            value = (static_cast<uint16_t>(byte1) << 8) | static_cast<uint16_t>(byte2);
        } else {
            // Only one byte available
            uint8_t byte1 = buffer[i];
            value = static_cast<uint16_t>(byte1) << 8;
        }
        
        // XOR with running checksum
        checksum ^= value;
    }
    
    return checksum;
}

//------------------------------------------------------------------------------
// Initialize server connection
//------------------------------------------------------------------------------
bool PositionControlImpl::InitializeServerConnection() {
    // Check if already connected
    if (serverConnector && serverConnector->IsConnected()) {
        connected.store(true);
        return true;
    }
    
    // If server connector already exists, update its configuration
    if (serverConnector) {
        serverConnector->SetServerAddress(config.serverAddress, config.serverPort);
    } else {
        // Create new server connector
        serverConnector.reset(new AsyncServerConnector(
            parent, name + "_connector", config.serverAddress, config.serverPort));
        
        // Set callback for connection state changes
        serverConnector->SetConnectionCallback([this](bool state) {
            connected.store(state);
        });
        
        // Start the connector thread
        serverConnector->Start();
    }
    
    // Initialize UDP unicast socket for receiving results if not already created
    if (!unicastReceiver) {
        try {
            // For simulation: calculate port based on UAV index to avoid port conflicts
            int calculatedPort = config.resultPort;
            
            // Create a properly bound receiving socket
            unicastReceiver.reset(new UdpSocket(parent, name + "_unicast", calculatedPort));
            
            // Start UDP receiver thread
            if (!receiverRunning.load()) {
                StartUdpReceiver(calculatedPort);
            }
        } catch (const std::exception& e) {
            if (parent) {
                const_cast<Object*>(parent)->Err("Failed to create UDP socket: %s", e.what());
            }
            return false;
        }
    }
    
    // Force an immediate connection attempt
    bool success = serverConnector->ForceConnect(500);
    connected.store(success);
    
    return success;
}

//------------------------------------------------------------------------------
// Test server connection
//------------------------------------------------------------------------------
bool PositionControlImpl::TestServerConnection(int timeoutMs) {
    if (!serverConnector) {
        if (!InitializeServerConnection()) {
            return false;
        }
    }
    
    // Try to connect with the specified timeout
    bool success = serverConnector->ForceConnect(timeoutMs);
    connected.store(success);
    
    return success;
}

//------------------------------------------------------------------------------
// Start UDP receiver for unicast messages
//------------------------------------------------------------------------------
bool PositionControlImpl::StartUdpReceiver(int port) {
    // Already running?
    if (receiverRunning.load()) {
        return true;
    }
    
    // Start the thread
    try {
        receiverRunning.store(true);
        udpReceiverThread = std::thread(&PositionControlImpl::UdpReceiverLoop, this);
        return true;
    } catch (const std::exception& e) {
        if (parent) {
            const_cast<Object*>(parent)->Err("Failed to start UDP receiver thread: %s", e.what());
        }
        receiverRunning.store(false);
        return false;
    }
}

//------------------------------------------------------------------------------
// UDP receiver thread function - dedicated thread for unicast messages
//------------------------------------------------------------------------------
void PositionControlImpl::UdpReceiverLoop() {
    // Buffers and settings
    char buffer[1024];
    char src[64];
    size_t src_size = sizeof(src);
    const int SOCKET_TIMEOUT_MS = 20;  // Short timeout for responsive shutdown
    
    // Enable thread cancellation
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    
    while (receiverRunning.load()) {
        // Exit if termination requested
        if (!receiverRunning.load()) break;
        
        // Skip if socket is not available
        if (!unicastReceiver) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        
        try {
            // Check termination flag before potentially blocking operation
            if (!receiverRunning.load()) break;
            
            // Receive message with short timeout for responsive shutdown
            int srcId = 0;
            ssize_t received = unicastReceiver->RecvMessage(
                buffer, sizeof(buffer), SOCKET_TIMEOUT_MS, src, &src_size, &srcId);
            
            // Check termination flag after potentially blocking operation
            if (!receiverRunning.load()) break;
            
            // Skip if not enough data received
            if (received < 4) continue;
            
            // Message parsing
            uint32_t magic = *((uint32_t*)buffer);
            
            // Handle unicast messages (new format)
            if (magic == UNICAST_MAGIC && received >= sizeof(UdpUnicastMessage)) {
                UdpUnicastMessage* msg = reinterpret_cast<UdpUnicastMessage*>(buffer);
                uint16_t calculatedChecksum = CalculateChecksum(
                    msg, sizeof(UdpUnicastMessage) - sizeof(msg->checksum));
                    
                if (calculatedChecksum == msg->checksum && msg->droneIndex == uavIndex) {
                    Vector3Df direction(msg->direction[0], msg->direction[1], msg->direction[2]);
                    float tension = msg->tension;
                    
                    // Special fallback flag check
                    if (direction.z >= -900 || direction.z <= -1100 || tension >= -0.5) {
                        ProcessServerData(msg->sequenceNumber, direction, tension);
                    } else {
                        connected.store(false);
                    }
                }
            }
            // Handle broadcast messages (legacy format)
            else if (magic == BROADCAST_MAGIC && received >= sizeof(UdpBroadcastMessage)) {
                UdpBroadcastMessage* msg = reinterpret_cast<UdpBroadcastMessage*>(buffer);
                uint16_t calculatedChecksum = CalculateChecksum(
                    msg, sizeof(UdpBroadcastMessage) - sizeof(msg->checksum));
                    
                if (calculatedChecksum == msg->checksum && uavIndex < msg->droneCount) {
                    Vector3Df direction(
                        msg->directions[uavIndex][0],
                        msg->directions[uavIndex][1],
                        msg->directions[uavIndex][2]
                    );
                    ProcessServerData(msg->sequenceNumber, direction, msg->tensions[uavIndex]);
                }
            }
        }
        catch (...) {
            // Check termination flag after exception
            if (!receiverRunning.load()) break;
        }
    }
}

//------------------------------------------------------------------------------
// Process data received from server
//------------------------------------------------------------------------------
bool PositionControlImpl::ProcessServerData(uint32_t sequence, 
                                           const Vector3Df& direction, 
                                           float tension) 
{
    // Validate values
    if (std::isnan(direction.x) || std::isnan(direction.y) || std::isnan(direction.z) ||
        std::isinf(direction.x) || std::isinf(direction.y) || std::isinf(direction.z)) {
        return false;
    }
    
    if (std::isnan(tension) || std::isinf(tension) || tension < 0.0f) {
        return false;
    }
    
    // Create normalized direction
    Vector3Df normalizedDir = direction;
    float norm = normalizedDir.GetNorm();
    if (norm > 0.0001f) {
        normalizedDir = normalizedDir / norm;
    } else {
        normalizedDir = Vector3Df(0, 0, 1);
    }
    
    // Verify tension is non-zero (fix very small tensions)
    if (tension <= 0.001f) {
        // Use a sensible minimum value
        tension = 1.0f;
    }
    
    // Update current data
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        
        // Store current data
        currentData.sequenceNumber = sequence;
        currentData.direction = normalizedDir;
        currentData.tension = tension;
        currentData.timestamp = GetCurrentTimeMs();
        
        // Store as valid fallback data
        lastValidData = currentData;
        hasValidData = true;
    }
    
    // Update master optimization time tracking
    if (uavIndex == 0) {
        // Look up request time for this sequence
        auto requestTimeIt = requestTimes.find(sequence);
        if (requestTimeIt != requestTimes.end()) {
            // Calculate round trip time
            uint64_t requestTime = requestTimeIt->second;
            uint64_t roundTripTime = GetCurrentTimeMs() - requestTime;
            
            // Remove this entry
            requestTimes.erase(requestTimeIt);
            
            // Clean up old entries (over 10 seconds old)
            uint64_t currentTime = GetCurrentTimeMs();
            for (auto it = requestTimes.begin(); it != requestTimes.end();) {
                if (currentTime - it->second > 10000) {
                    it = requestTimes.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }
    
    // Update connection status
    connected.store(true);
    
    return true;
}

//------------------------------------------------------------------------------
// Get current server data
//------------------------------------------------------------------------------
ServerDataPacket PositionControlImpl::GetCurrentData() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    
    // Check if we have recent valid data
    uint64_t currentTime = GetCurrentTimeMs();
    uint64_t timeout = 2000; // 2 seconds timeout
    
    if (hasValidData && (currentTime - currentData.timestamp < timeout)) {
        // Current data is recent and valid
        return currentData;
    } else if (hasValidData) {
        // Fall back to last valid data
        return lastValidData;
    } else {
        // No valid data available
        return ServerDataPacket();
    }
}

//------------------------------------------------------------------------------
// Send resultant force to server (master only)
//------------------------------------------------------------------------------
bool PositionControlImpl::SendResultantForce(const Vector3Df& resultantForce) {
    // Only master sends resultant force
    if (uavIndex != 0) {
        return false;
    }
    
    // Check if server connector is available
    if (!serverConnector) {
        return false;
    }
    
    // Limit sending frequency (max 20 Hz)
    uint64_t currentTime = GetCurrentTimeMs();
    static uint64_t lastSendTime = 0;
    if (currentTime - lastSendTime < 50) {
        return false;
    }
    
    try {
        // Create request
        TcpRequestMessage request;
        std::memset(&request, 0, sizeof(request));
        
        // Set fields
        request.magic = REQUEST_MAGIC;
        request.sequenceNumber = currentTime & 0xFFFFFFFF; // Use time as sequence
        request.droneId = uavIndex;
        request.timestamp = currentTime;
        request.resultantForce[0] = resultantForce.x;
        request.resultantForce[1] = resultantForce.y;
        request.resultantForce[2] = resultantForce.z;
        
        // Calculate checksum
        request.checksum = CalculateChecksum(
            &request, sizeof(request) - sizeof(request.checksum));
        
        // Store request time for optimization timing
        requestTimes[request.sequenceNumber] = currentTime;
        
        // Send to server
        bool sent = serverConnector->SendData(&request, sizeof(request));
        
        if (sent) {
            lastSendTime = currentTime;
            return true;
        }
        
        return false;
    }
    catch (const std::exception& e) {
        if (parent) {
            const_cast<Object*>(parent)->Err("Failed to send resultant force: %s", e.what());
        }
        return false;
    }
}

//------------------------------------------------------------------------------
// Send heartbeat message to server (master only)
//------------------------------------------------------------------------------
void PositionControlImpl::SendHeartbeatMessage() {
    // Only send if we're the master drone
    if (uavIndex != 0 || !serverConnector) {
        return;
    }
    
    // Only send every second
    uint64_t currentTime = GetCurrentTimeMs();
    if (currentTime - lastHeartbeatTimeMs < 1000) {
        return;
    }
    
    lastHeartbeatTimeMs = currentTime;
    
    // Create heartbeat message
    TcpHeartbeatMessage heartbeat;
    std::memset(&heartbeat, 0, sizeof(heartbeat));
    
    // Set fields
    heartbeat.magic = HEARTBEAT_MAGIC;
    heartbeat.droneId = uavIndex;
    heartbeat.timestamp = currentTime;
    
    // Calculate checksum
    heartbeat.checksum = CalculateChecksum(
        &heartbeat, sizeof(heartbeat) - sizeof(heartbeat.checksum));
    
    // Send to server
    if (serverConnector) {
        try {
            serverConnector->SendData(&heartbeat, sizeof(heartbeat));
        }
        catch (const std::exception&) {
            // Silent fail for heartbeat
        }
    }
}

//------------------------------------------------------------------------------
// Get server connection status
//------------------------------------------------------------------------------
bool PositionControlImpl::IsConnected() const {
    return connected.load();
}

//------------------------------------------------------------------------------
// Force reconnection
//------------------------------------------------------------------------------
void PositionControlImpl::ForceReconnect() {
    if (serverConnector) {
        serverConnector->ForceConnect(1000);
    }
}

//------------------------------------------------------------------------------
// Set connection callback
//------------------------------------------------------------------------------
void PositionControlImpl::SetConnectionCallback(std::function<void(bool)> callback) {
    if (serverConnector) {
        serverConnector->SetConnectionCallback(callback);
    }
}

//------------------------------------------------------------------------------
// Update connection status display
//------------------------------------------------------------------------------
void PositionControlImpl::UpdateConnectionStatusDisplay(Label* statusLabel) {
    if (!statusLabel) return;
    
    // Update less frequently to reduce overhead
    uint64_t currentTime = GetCurrentTimeMs();
    if (currentTime - lastStatusUpdateMs < 2000) { // Every 2 seconds
        return;
    }
    
    lastStatusUpdateMs = currentTime;
    
    // Get connection status
    bool isConnected = connected.load();
    
    // Update label
    std::stringstream ss;
    ss << "Server: " << (isConnected ? "Connected" : "Disconnected");
    statusLabel->SetText(ss.str().c_str());
}

//------------------------------------------------------------------------------
// Check and update connection status
//------------------------------------------------------------------------------
void PositionControlImpl::CheckConnectionStatus() {
    // Only check occasionally to avoid excessive checking
    uint64_t currentTime = GetCurrentTimeMs();
    
    // Use adaptive interval based on connection status
    uint64_t checkInterval = connected.load() ? 10000 : 5000; // 10s/5s
    static uint64_t lastCheckTime = 0;
    
    if (currentTime - lastCheckTime < checkInterval) {
        return;
    }
    
    lastCheckTime = currentTime;
    
    // If we have a server connector but it's not connected, 
    // force a reconnection attempt
    if (serverConnector && !serverConnector->IsConnected()) {
        serverConnector->ForceConnect(200);
    }
}

}} // namespace flair::filter