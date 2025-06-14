//  created:    2025/05/05
//  filename:   AsyncServerConnector.cpp
//
//  author:     Lamberto Vazquez
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  purpose:    Asynchronous server connector for optimization server
/*********************************************************************/

#include "AsyncServerConnector.h"
#include "TcpClient.h"

#include <iostream>
#include <sstream>
#include <cstring>
#include <algorithm>
#include <unistd.h>

namespace flair {
namespace filter {

using namespace flair::core;

// Constructor
AsyncServerConnector::AsyncServerConnector(const Object* parent, 
                                         const std::string& name,
                                         const std::string& serverAddress,
                                         int serverPort)
    : Object(parent, name),
      serverAddress(serverAddress),
      serverPort(serverPort),
      running(false),
      forceConnectFlag(false),
      forceConnectTimeout(1000),
      connected(false),
      lastConnectionAttempt(0)
{
    // Create the TCP client
    tcpClient.reset(new TcpClient(this, "tcp_client", serverAddress, serverPort));
    
    // Default connection callback
    connectionCallback = [](bool) {};
}

// Destructor
AsyncServerConnector::~AsyncServerConnector() {
    // Stop the connection thread
    Stop();
}

// Start the connection thread
bool AsyncServerConnector::Start() {
    std::lock_guard<std::mutex> lock(connectMutex);
    
    // Check if already running
    if (running.load()) {
        return true;
    }
    
    try {
        // Set running flag and start thread
        running.store(true);
        connectionThread = std::thread(&AsyncServerConnector::ConnectionThreadFunc, this);
        
        Info("AsyncServerConnector thread started");
        return true;
    }
    catch (const std::exception& e) {
        Err("Failed to start connection thread: %s", e.what());
        running.store(false);
        return false;
    }
}

// Stop the connection thread
void AsyncServerConnector::Stop() {
    // First set all stop flags atomically
    {
        std::lock_guard<std::mutex> lock(connectMutex);
        
        if (!running.load()) {
            return;
        }
        
        running.store(false);
        connected.store(false);
        forceConnectFlag.store(false);
        
        // Close client connection to unblock socket operations
        if (tcpClient) {
            try {
                tcpClient->Close();
            } catch (...) {
                // Ignore errors during shutdown
            }
        }
    }
    
    // Wake up any waiting threads
    connectCV.notify_all();
    
    // Join with timeout and forced cancellation
    if (connectionThread.joinable()) {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
        
        while (connectionThread.joinable() && 
              std::chrono::steady_clock::now() < deadline) {
            // Force thread cancellation for stuck system calls
            try {
                pthread_cancel(connectionThread.native_handle());
            } catch (...) {}
            
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        
        // Final join attempt
        if (connectionThread.joinable()) {
            connectionThread.join();
        }
    }
}

// Background thread function
void AsyncServerConnector::ConnectionThreadFunc() {
    const int RECONNECT_BASE_MS = 5000;
    const int RECONNECT_MAX_MS = 60000;
    int reconnectInterval = RECONNECT_BASE_MS;
    
    // Enable thread cancellation
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    
    while (running.load()) {
        // Check termination flag
        if (!running.load()) break;
        
        bool shouldConnect = false;
        int timeout = 1000;  // Default timeout
        
        {
            std::unique_lock<std::mutex> lock(connectMutex);
            
            // Determine if connection attempt is needed
            if (forceConnectFlag.load()) {
                shouldConnect = true;
                timeout = forceConnectTimeout;
                forceConnectFlag.store(false);
            } else {
                uint64_t now = GetCurrentTimeMs();
                shouldConnect = !tcpClient->IsConnected() && 
                               (now - lastConnectionAttempt >= reconnectInterval);
            }
            
            // Wait if no connection needed
            if (!shouldConnect) {
                connectCV.wait_for(lock, std::chrono::milliseconds(500), 
                                  [this] { 
                                      return !running.load() || forceConnectFlag.load(); 
                                  });
                continue;
            }
            
            lastConnectionAttempt = GetCurrentTimeMs();
        }
        
        // Check termination flag again before connection attempt
        if (!running.load()) break;
        
        // Attempt connection (outside lock)
        bool prevConnected = connected.load();
        bool newConnected = tcpClient->Connect(timeout);
        connected.store(newConnected);
        
        // Update reconnection interval
        if (newConnected) {
            reconnectInterval = RECONNECT_BASE_MS;
        } else {
            reconnectInterval = std::min(reconnectInterval * 2, RECONNECT_MAX_MS);
        }
        
        // Notify about connection state change if needed
        if (prevConnected != newConnected) {
            std::lock_guard<std::mutex> callbackLock(callbackMutex);
            connectionCallback(newConnected);
        }
    }
}

// Check if connected to the server
bool AsyncServerConnector::IsConnected() const {
    return connected.load();
}

// Send data to the server
bool AsyncServerConnector::SendData(const void* data, size_t size) {
    // No need to check if connected - TcpClient will try to reconnect if needed
    return tcpClient->SendRaw(data, size);
}

// Receive data from the server
bool AsyncServerConnector::ReceiveData(void* buffer, size_t size, size_t& bytesRead, int timeoutMs) {
    if (!connected.load()) {
        bytesRead = 0;
        return false;
    }
    
    return tcpClient->ReceiveRaw(buffer, size, bytesRead, timeoutMs);
}

// Get the last error message
std::string AsyncServerConnector::GetLastError() const {
    return tcpClient->GetLastError();
}

// Set connection state change callback
void AsyncServerConnector::SetConnectionCallback(std::function<void(bool)> callback) {
    std::lock_guard<std::mutex> lock(callbackMutex);
    connectionCallback = callback ? callback : [](bool) {};
}

// Change server address and port
void AsyncServerConnector::SetServerAddress(const std::string& newServerAddress, int newServerPort) {
    bool changed = false;
    
    {
        std::lock_guard<std::mutex> lock(connectMutex);
        
        // Check if address or port changed
        if (serverAddress != newServerAddress || serverPort != newServerPort) {
            serverAddress = newServerAddress;
            serverPort = newServerPort;
            changed = true;
        }
    }
    
    // Update TCP client if changed
    if (changed) {
        tcpClient->SetServer(newServerAddress, newServerPort);
        
        // Force reconnection attempt
        ForceConnect(1000);
    }
}

// Force an immediate connection attempt
bool AsyncServerConnector::ForceConnect(int timeoutMs) {
    {
        std::lock_guard<std::mutex> lock(connectMutex);
        
        // Set force flag and timeout
        forceConnectFlag.store(true);
        forceConnectTimeout = timeoutMs;
    }
    
    // Notify thread to wake up
    connectCV.notify_one();
    
    // Wait a bit for the connection to be attempted
    std::this_thread::sleep_for(std::chrono::milliseconds(std::min(timeoutMs, 100)));
    
    // Return current connection state
    return tcpClient->IsConnected();
}

} // namespace filter
} // namespace flair