#ifndef ASYNC_SERVER_CONNECTOR_H
#define ASYNC_SERVER_CONNECTOR_H

#include <Object.h>
#include <Vector3D.h>
#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>
#include <queue>
#include <functional>
#include <memory>

namespace flair {
namespace filter {

// Forward declarations
class TcpClient;

/**
 * @brief Class that manages asynchronous server communication
 * 
 * This class handles server connection attempts in a background thread,
 * allowing the control logic to continue running regardless of server status.
 */
class AsyncServerConnector : public flair::core::Object {
public:
    /**
     * @brief Constructor
     * 
     * @param parent Parent object
     * @param name Object name
     * @param serverAddress Server IP address or hostname
     * @param serverPort Server port number
     */
    AsyncServerConnector(const flair::core::Object* parent, 
                        const std::string& name,
                        const std::string& serverAddress = "127.0.0.1",
                        int serverPort = 5555);
    
    /**
     * @brief Destructor - ensures background thread is properly terminated
     */
    ~AsyncServerConnector();
    
    /**
     * @brief Start the connector thread
     * 
     * Begins background connection attempts if not already running
     * 
     * @return true if thread started successfully
     */
    bool Start();
    
    /**
     * @brief Stop the connector thread
     * 
     * Terminates background connection attempts
     */
    void Stop();
    
    /**
     * @brief Check if connected to the server
     * 
     * @return true if currently connected
     */
    bool IsConnected() const;
    
    /**
     * @brief Send data to the server with automatic reconnection
     * 
     * @param data Data buffer to send
     * @param size Size of data in bytes
     * @return true if data was sent successfully
     */
    bool SendData(const void* data, size_t size);
    
    /**
     * @brief Try to receive data with timeout
     * 
     * @param buffer Output buffer for received data
     * @param size Buffer size in bytes
     * @param bytesRead Number of bytes actually read
     * @param timeoutMs Timeout in milliseconds
     * @return true if data was received
     */
    bool ReceiveData(void* buffer, size_t size, size_t& bytesRead, int timeoutMs = 1000);
    
    /**
     * @brief Get last error message
     * 
     * @return Error message
     */
    std::string GetLastError() const;
    
    /**
     * @brief Set callback for connection state changes
     * 
     * @param callback Function to call when connection state changes
     */
    void SetConnectionCallback(std::function<void(bool)> callback);
    
    /**
     * @brief Change server address and port
     * 
     * @param serverAddress New server address
     * @param serverPort New server port
     */
    void SetServerAddress(const std::string& serverAddress, int serverPort);
    
    /**
     * @brief Force an immediate connection attempt
     * 
     * @param timeoutMs Connection timeout in milliseconds
     * @return true if connection successful
     */
    bool ForceConnect(int timeoutMs = 1000);

private:
    /**
     * @brief Background thread function that handles connection attempts
     */
    void ConnectionThreadFunc();
    
    // TcpClient for actual communication
    std::unique_ptr<TcpClient> tcpClient;
    
    // Thread management
    std::thread connectionThread;
    std::atomic<bool> running;
    std::condition_variable connectCV;
    std::mutex connectMutex;
    
    // Connection parameters
    std::string serverAddress;
    int serverPort;
    std::atomic<bool> forceConnectFlag;
    int forceConnectTimeout;
    
    // Connection callback
    std::function<void(bool)> connectionCallback;
    std::mutex callbackMutex;
    
    // Status tracking
    std::atomic<bool> connected;
    uint64_t lastConnectionAttempt;
    
    /**
     * @brief Get current time in milliseconds
     */
    uint64_t GetCurrentTimeMs() const {
        auto now = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();
    }
};

} // namespace filter
} // namespace flair

#endif // ASYNC_SERVER_CONNECTOR_H