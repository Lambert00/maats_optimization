// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
* \file TcpClient.h
* \brief TCP client for optimization server communication
* \author Lamberto Vazquez, Copyright Heudiasyc UMR UTC/CNRS 7253
* \date 2023/04/15
* \version 5.0
*/

#ifndef TCPCLIENT_H
#define TCPCLIENT_H

#include <Object.h>
#include <Vector3D.h>
#include <string>
#include <mutex>
#include <atomic>
#include <chrono>
#include <thread>
#include <queue>

namespace flair {
namespace filter {

/**
 * @brief TCP client for communication with the optimization server
 * 
 * Manages a persistent TCP connection to the optimization server
 * and provides efficient binary message exchange.
 */
class TcpClient : public flair::core::Object {
public:
    /**
     * @brief Constructor
     * 
     * @param parent Parent object
     * @param name Object name
     * @param serverAddress Server IP address or hostname
     * @param serverPort Server port
     */
    TcpClient(const flair::core::Object* parent, 
              const std::string& name,
              const std::string& serverAddress = "172.26.213.70", 
              int serverPort = 5555);
    
    /**
     * @brief Destructor
     */
    ~TcpClient();
    
    /**
     * @brief Connect to the server
     * 
     * @param timeoutMs Connection timeout in milliseconds
     * @return true if connection successful
     */
    bool Connect(int timeoutMs = 1000);
    
    /**
     * @brief Disconnect from the server but keep socket
     */
    void Disconnect();
    
    /**
     * @brief Fully close the connection and clean up resources
     * 
     * Closes the socket and ensures all resources are released.
     * This should be called before destroying the object.
     */
    void Close();
    
    /**
     * @brief Check if connected to the server
     * 
     * @return true if connected
     */
    bool IsConnected() const;
    
    /**
     * @brief Change server address
     * 
     * @param serverAddress New server address
     * @param serverPort New server port
     */
    void SetServer(const std::string& serverAddress, int serverPort);
    
    /**
     * @brief Send raw data to server
     * 
     * @param data Data buffer
     * @param size Data size
     * @return true if sent successfully
     */
    bool SendRaw(const void* data, size_t size);
    
    /**
     * @brief Receive raw data from server
     * 
     * @param buffer Output buffer
     * @param size Buffer size
     * @param bytesRead Number of bytes read
     * @param timeoutMs Timeout in milliseconds
     * @return true if received successfully
     */
    bool ReceiveRaw(void* buffer, size_t size, size_t& bytesRead, int timeoutMs = 1000);
    
    /**
     * @brief Get last error message
     * 
     * @return Error message
     */
    std::string GetLastError() const;
    
    /**
     * @brief Status structure
     */
    struct Status {
        bool connected;                  ///< Connection status
        uint64_t lastSendTimeMs;         ///< Last message send time
        uint64_t lastRecvTimeMs;         ///< Last message receive time
        uint64_t lastSuccessfulConnectTimeMs; ///< Last successful connection time
        uint32_t sendCount;              ///< Number of sent messages
        uint32_t recvCount;              ///< Number of received messages
        uint32_t errorCount;             ///< Number of errors
        uint64_t avgLatencyMs;           ///< Average response latency
        int lastErrorCode;               ///< Last system error code
        std::string serverAddress;       ///< Current server address
        int serverPort;                  ///< Current server port
    };
    
    /**
     * @brief Get current status
     * 
     * @return Status structure
     */
    Status GetStatus() const;
    
    /**
     * @brief Try to reconnect to server
     * 
     * Uses exponential backoff to avoid overwhelming the server
     * 
     * @return true if reconnection successful
     */
    bool TryReconnect();

private:
    /**
     * @brief Get current time in milliseconds
     * 
     * @return Current time
     */
    uint64_t GetCurrentTimeMs() const;
    
    // Configuration
    std::string serverAddress_;          ///< Server address
    int serverPort_;                     ///< Server port
    int socket_;                         ///< Socket file descriptor
    
    // Status
    std::atomic<bool> connected_;        ///< Connection status
    mutable std::mutex mutex_;           ///< Mutex for thread safety
    mutable std::string lastError_;      ///< Last error message
    
    // Statistics
    Status status_;                      ///< Current status
    mutable std::mutex statsMutex_;      ///< Statistics mutex
    
    // Connection parameters
    uint64_t reconnectIntervalMs_;       ///< Reconnection interval (with backoff)
    uint64_t lastReconnectAttemptMs_;    ///< Last reconnection attempt time
};

} // namespace filter
} // namespace flair

#endif // TCPCLIENT_H