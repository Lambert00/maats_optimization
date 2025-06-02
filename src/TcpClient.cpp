// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2023/04/15
//  filename:   TcpClient.cpp
//
//  author:     Lamberto Vazquez
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    5.0
//
//  purpose:    TCP client for optimization server communication
//
/*********************************************************************/

#include "TcpClient.h"

#include <iostream>
#include <sstream>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <cerrno>
#include <algorithm>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/tcp.h>

namespace flair {
namespace filter {

using namespace flair::core;

// Constructor
TcpClient::TcpClient(const Object* parent, 
                     const std::string& name,
                     const std::string& serverAddress, 
                     int serverPort)
    : Object(parent, name),
      serverAddress_(serverAddress),
      serverPort_(serverPort),
      socket_(-1),
      connected_(false),
      reconnectIntervalMs_(5000),  // 5 seconds between reconnection attempts
      lastReconnectAttemptMs_(0)
{
    // Initialize status
    status_.connected = false;
    status_.lastSendTimeMs = 0;
    status_.lastRecvTimeMs = 0;
    status_.lastSuccessfulConnectTimeMs = 0;
    status_.sendCount = 0;
    status_.recvCount = 0;
    status_.errorCount = 0;
    status_.avgLatencyMs = 0;
    status_.lastErrorCode = 0;
    status_.serverAddress = serverAddress_;
    status_.serverPort = serverPort_;
    
    Info("Created TCP client for server %s:%d", serverAddress_.c_str(), serverPort_);
}

// Destructor
TcpClient::~TcpClient() {
    // Fully close socket and clean up resources
    try {
        // Use Close instead of Disconnect to ensure proper cleanup
        Close();
    } catch (const std::exception& e) {
        Err("Error during TCP client destruction: %s", e.what());
    }
    
    Info("TCP client destroyed");
}

// Connect to server with robust error handling
bool TcpClient::Connect(int timeoutMs) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Check if we've attempted reconnection too recently
    uint64_t currentTime = GetCurrentTimeMs();
    if (currentTime - lastReconnectAttemptMs_ < 1000 && socket_ < 0) {
        // Too soon for another attempt
        return false;
    }
    
    lastReconnectAttemptMs_ = currentTime;
    
    // If already connected, return success
    if (socket_ >= 0) {
        // Verify connection is still valid with a quick poll
        struct pollfd pfd;
        pfd.fd = socket_;
        pfd.events = POLLOUT;
        
        if (poll(&pfd, 1, 0) >= 0 && !(pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            // Connection appears valid
            connected_ = true;
            return true;
        }
        
        // Socket is in bad state, close it
        Info("Socket appears broken, closing for reconnection");
        ::close(socket_);
        socket_ = -1;
        connected_ = false;
    }
    
    try {
        // Create socket
        socket_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (socket_ < 0) {
            status_.lastErrorCode = errno;
            lastError_ = "Failed to create socket: " + std::string(strerror(errno));
            Err("%s: %s", ObjectName().c_str(), lastError_.c_str());
            
            {
                std::lock_guard<std::mutex> lock(statsMutex_);
                status_.errorCount++;
            }
            
            socket_ = -1;
            connected_ = false;
            return false;
        }
        
        // Set keep-alive for persistent connections
        int yes = 1;
        if (setsockopt(socket_, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(int)) < 0) {
            Warn("Failed to set SO_KEEPALIVE");
        }
        
        // Set non-blocking mode for timeout support
        int flags = ::fcntl(socket_, F_GETFL, 0);
        ::fcntl(socket_, F_SETFL, flags | O_NONBLOCK);
        
        // Prepare server address
        struct sockaddr_in serverAddr;
        std::memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(serverPort_);
        
        // Handle hostname or IP address
        struct addrinfo hints, *result;
        std::memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        
        if (getaddrinfo(serverAddress_.c_str(), nullptr, &hints, &result) != 0) {
            status_.lastErrorCode = errno;
            lastError_ = "Failed to resolve hostname: " + serverAddress_;
            Err("%s: %s", ObjectName().c_str(), lastError_.c_str());
            
            ::close(socket_);
            socket_ = -1;
            
            {
                std::lock_guard<std::mutex> lock(statsMutex_);
                status_.errorCount++;
            }
            
            connected_ = false;
            return false;
        }
        
        // Copy the resolved address
        struct sockaddr_in* addr = (struct sockaddr_in*)result->ai_addr;
        serverAddr.sin_addr = addr->sin_addr;
        
        // Free the address info result
        freeaddrinfo(result);
        
        // Try to connect
        int result_code = ::connect(socket_, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
        if (result_code < 0 && errno != EINPROGRESS) {
            status_.lastErrorCode = errno;
            lastError_ = "Connection failed: " + std::string(strerror(errno));
            Err("%s: %s", ObjectName().c_str(), lastError_.c_str());
            
            ::close(socket_);
            socket_ = -1;
            
            {
                std::lock_guard<std::mutex> lock(statsMutex_);
                status_.errorCount++;
            }
            
            connected_ = false;
            return false;
        }
        
        // Wait for connection with timeout
        if (result_code < 0) {
            struct pollfd pfd;
            pfd.fd = socket_;
            pfd.events = POLLOUT;
            
            result_code = ::poll(&pfd, 1, timeoutMs);
            
            if (result_code <= 0) {
                status_.lastErrorCode = (result_code == 0) ? ETIMEDOUT : errno;
                if (result_code == 0) {
                    lastError_ = "Connection timeout";
                } else {
                    lastError_ = "Poll error: " + std::string(strerror(errno));
                }
                
                Err("%s: %s", ObjectName().c_str(), lastError_.c_str());
                
                ::close(socket_);
                socket_ = -1;
                
                {
                    std::lock_guard<std::mutex> lock(statsMutex_);
                    status_.errorCount++;
                }
                
                connected_ = false;
                return false;
            }
            
            // Check if connection was successful
            int error = 0;
            socklen_t len = sizeof(error);
            if (::getsockopt(socket_, SOL_SOCKET, SO_ERROR, &error, &len) < 0 || error != 0) {
                status_.lastErrorCode = (error != 0) ? error : errno;
                if (error != 0) {
                    lastError_ = "Connection failed: " + std::string(strerror(error));
                } else {
                    lastError_ = "Getsockopt failed: " + std::string(strerror(errno));
                }
                
                Err("%s: %s", ObjectName().c_str(), lastError_.c_str());
                
                ::close(socket_);
                socket_ = -1;
                
                {
                    std::lock_guard<std::mutex> lock(statsMutex_);
                    status_.errorCount++;
                }
                
                connected_ = false;
                return false;
            }
        }
        
        // Set socket back to blocking mode
        flags = ::fcntl(socket_, F_GETFL, 0);
        ::fcntl(socket_, F_SETFL, flags & ~O_NONBLOCK);
        
        // Set TCP_NODELAY to disable Nagle's algorithm
        int flag = 1;
        setsockopt(socket_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int));
        
        // Set timeouts
        struct timeval tv;
        tv.tv_sec = timeoutMs / 1000;
        tv.tv_usec = (timeoutMs % 1000) * 1000;
        
        setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(tv));
        setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO, (char*)&tv, sizeof(tv));
        
        // Update status
        connected_ = true;
        
        {
            std::lock_guard<std::mutex> lock(statsMutex_);
            status_.connected = true;
            status_.lastSuccessfulConnectTimeMs = GetCurrentTimeMs();
            status_.serverAddress = serverAddress_;
            status_.serverPort = serverPort_;
        }
        
        Info("%s: Connected to %s:%d", ObjectName().c_str(), serverAddress_.c_str(), serverPort_);
        
        return true;
    }
    catch (const std::exception& e) {
        // Handle any unexpected exceptions
        lastError_ = std::string("Unexpected exception during connect: ") + e.what();
        Err("%s: %s", ObjectName().c_str(), lastError_.c_str());
        
        if (socket_ >= 0) {
            ::close(socket_);
            socket_ = -1;
        }
        
        {
            std::lock_guard<std::mutex> lock(statsMutex_);
            status_.errorCount++;
        }
        
        connected_ = false;
        return false;
    }
}

// Disconnect from server but keep socket information
void TcpClient::Disconnect() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (socket_ >= 0) {
        ::close(socket_);
        socket_ = -1;
    }
    
    connected_ = false;
    
    {
        std::lock_guard<std::mutex> lock(statsMutex_);
        status_.connected = false;
    }
    
    Info("%s: Disconnected from %s:%d", ObjectName().c_str(), serverAddress_.c_str(), serverPort_);
}

// Fully close connection and clean up all resources
void TcpClient::Close() {
    // Set disconnected state first to prevent new operations
    connected_.store(false);
    
    // Log shutdown
    Info("%s: Permanently closing connection to %s:%d", 
         ObjectName().c_str(), serverAddress_.c_str(), serverPort_);
    
    {
        // Lock for socket operations
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Ensure socket is completely closed
        if (socket_ >= 0) {
            // First try a graceful shutdown to let the server know we're going away
            ::shutdown(socket_, SHUT_RDWR);
            
            // Then close the socket forcefully
            ::close(socket_);
            socket_ = -1;
        }
    }
    
    // Update statistics - using separate lock to avoid deadlocks
    {
        std::lock_guard<std::mutex> lock(statsMutex_);
        status_.connected = false;
    }
    
    // Reset backoff timers (no lock needed, only used in connection operations)
    reconnectIntervalMs_ = 5000;
    lastReconnectAttemptMs_ = 0;
    
    Info("%s: Connection to %s:%d permanently closed", 
         ObjectName().c_str(), serverAddress_.c_str(), serverPort_);
}

// Check if connected
bool TcpClient::IsConnected() const {
    // Simple check of the flag - actual connection verified during operations
    return connected_.load();
}

// Set server address and port
void TcpClient::SetServer(const std::string& serverAddress, int serverPort) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Only update if values changed
    if (serverAddress_ != serverAddress || serverPort_ != serverPort) {
        serverAddress_ = serverAddress;
        serverPort_ = serverPort;
        
        // Update status
        {
            std::lock_guard<std::mutex> statsLock(statsMutex_);
            status_.serverAddress = serverAddress_;
            status_.serverPort = serverPort_;
        }
        
        // Reconnect if currently connected
        if (connected_) {
            Info("%s: Server changed to %s:%d, reconnecting...", 
                 ObjectName().c_str(), serverAddress_.c_str(), serverPort_);
            Disconnect();
            Connect(1000);
        } else {
            Info("%s: Server changed to %s:%d", 
                 ObjectName().c_str(), serverAddress_.c_str(), serverPort_);
        }
    }
}

// Get last error message
std::string TcpClient::GetLastError() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lastError_;
}

// Get current status
TcpClient::Status TcpClient::GetStatus() const {
    std::lock_guard<std::mutex> lock(statsMutex_);
    Status result = status_;
    
    // Make sure server address and port are up to date
    result.serverAddress = serverAddress_;
    result.serverPort = serverPort_;
    
    return result;
}

// Get current time in milliseconds
uint64_t TcpClient::GetCurrentTimeMs() const {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
}

// Try to reconnect to server
bool TcpClient::TryReconnect() {
    // Check if too soon for another attempt
    uint64_t currentTime = GetCurrentTimeMs();
    if (currentTime - lastReconnectAttemptMs_ < reconnectIntervalMs_) {
        return false;
    }
    
    // Attempt reconnection with increasing backoff
    bool success = Connect(1000);
    
    if (success) {
        // Reset reconnect interval on success
        reconnectIntervalMs_ = 5000; // Reset to base value
        Info("%s: Reconnected to %s:%d", ObjectName().c_str(), serverAddress_.c_str(), serverPort_);
    } else {
        // Increase backoff time for next attempt (max 30 seconds)
        reconnectIntervalMs_ = std::min(reconnectIntervalMs_ * 2, (uint64_t)30000);
        Warn("%s: Reconnection to %s:%d failed, next attempt in %ld ms", 
             ObjectName().c_str(), serverAddress_.c_str(), serverPort_, reconnectIntervalMs_);
    }
    
    return success;
}

// Send raw data over socket with robust error handling
bool TcpClient::SendRaw(const void* data, size_t size) {
    // Quick check if not connected
    if (!connected_.load() || socket_ < 0) {
        // Try to reconnect 
        if (!TryReconnect()) {
            // Still not connected, return failure
            return false;
        }
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Check if socket is valid
    if (socket_ < 0) {
        lastError_ = "Socket not connected";
        connected_ = false;
        return false;
    }
    
    try {
        // Set a send timeout
        struct timeval tv;
        tv.tv_sec = 1;  // 1 second timeout
        tv.tv_usec = 0;
        setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO, (char*)&tv, sizeof(tv));
        
        // Send data
        const char* buffer = static_cast<const char*>(data);
        size_t remaining = size;
        
        while (remaining > 0) {
            ssize_t sent = ::send(socket_, buffer, remaining, 0);
            
            if (sent < 0) {
                status_.lastErrorCode = errno;
                lastError_ = "Send error: " + std::string(strerror(errno));
                Err("%s: %s", ObjectName().c_str(), lastError_.c_str());
                
                // Socket error, mark as disconnected
                connected_ = false;
                
                {
                    std::lock_guard<std::mutex> lock(statsMutex_);
                    status_.errorCount++;
                }
                
                return false;
            }
            
            buffer += sent;
            remaining -= sent;
        }
        
        // Update statistics
        {
            std::lock_guard<std::mutex> lock(statsMutex_);
            status_.sendCount++;
            status_.lastSendTimeMs = GetCurrentTimeMs();
        }
        
        return true;
    }
    catch (const std::exception& e) {
        // Handle any unexpected exceptions
        lastError_ = std::string("Unexpected exception during send: ") + e.what();
        Err("%s: %s", ObjectName().c_str(), lastError_.c_str());
        
        connected_ = false;
        
        {
            std::lock_guard<std::mutex> lock(statsMutex_);
            status_.errorCount++;
        }
        
        return false;
    }
}

// Receive raw data from socket with improved error handling
bool TcpClient::ReceiveRaw(void* buffer, size_t size, size_t& bytesRead, int timeoutMs) {
    // Quick check if not connected
    if (!connected_.load() || socket_ < 0) {
        bytesRead = 0;
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Check if socket is valid
    if (socket_ < 0) {
        lastError_ = "Socket not connected";
        bytesRead = 0;
        connected_ = false;
        return false;
    }
    
    try {
        // Set up poll with timeout
        struct pollfd pfd;
        pfd.fd = socket_;
        pfd.events = POLLIN;
        
        // Wait for data
        int result = ::poll(&pfd, 1, timeoutMs);
        
        if (result <= 0) {
            status_.lastErrorCode = (result == 0) ? ETIMEDOUT : errno;
            if (result == 0) {
                // Timeout - not necessarily an error
                lastError_ = "Receive timeout";
            } else {
                // Poll error
                lastError_ = "Poll error: " + std::string(strerror(errno));
                Err("%s: %s", ObjectName().c_str(), lastError_.c_str());
                
                // Socket error, mark as disconnected
                connected_ = false;
            }
            
            bytesRead = 0;
            return false;
        }
        
        // Check for socket errors
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            lastError_ = "Socket in error state";
            Err("%s: %s", ObjectName().c_str(), lastError_.c_str());
            
            connected_ = false;
            bytesRead = 0;
            return false;
        }
        
        // Receive data
        ssize_t received = ::recv(socket_, buffer, size, 0);
        
        if (received < 0) {
            status_.lastErrorCode = errno;
            lastError_ = "Receive error: " + std::string(strerror(errno));
            Err("%s: %s", ObjectName().c_str(), lastError_.c_str());
            
            // Socket error, mark as disconnected
            connected_ = false;
            
            bytesRead = 0;
            return false;
        }
        
        if (received == 0) {
            // Connection closed by server
            lastError_ = "Connection closed by server";
            Warn("%s: %s", ObjectName().c_str(), lastError_.c_str());
            
            // Mark as disconnected
            connected_ = false;
            
            bytesRead = 0;
            return false;
        }
        
        // Update statistics
        {
            std::lock_guard<std::mutex> lock(statsMutex_);
            status_.recvCount++;
            status_.lastRecvTimeMs = GetCurrentTimeMs();
        }
        
        bytesRead = static_cast<size_t>(received);
        return true;
    }
    catch (const std::exception& e) {
        // Handle any unexpected exceptions
        lastError_ = std::string("Unexpected exception during receive: ") + e.what();
        Err("%s: %s", ObjectName().c_str(), lastError_.c_str());
        
        connected_ = false;
        bytesRead = 0;
        
        {
            std::lock_guard<std::mutex> lock(statsMutex_);
            status_.errorCount++;
        }
        
        return false;
    }
}

} // namespace filter
} // namespace flair