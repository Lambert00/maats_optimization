// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2023/04/15
//  filename:   PositionControl.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    5.0
//
//  purpose:    Class defining a control law with direct server communication
//
/*********************************************************************/

#include "PositionControl.h"
#include "PositionControlImpl.h"
#include "TcpClient.h"
#include "SharedParameters.h"

#include <FrameworkManager.h>
#include <Matrix.h>
#include <LayoutPosition.h>
#include <Layout.h>
#include <Tab.h>
#include <TabWidget.h>
#include <DataPlot1D.h>
#include <DoubleSpinBox.h>
#include <Label.h>
#include <MetaVrpnObject.h>
#include <VrpnClient.h>
#include <math.h>
#include <Pid.h>
#include <Euler.h>
#include <Quaternion.h>
#include <TrajectoryGenerator2DCircle.h>
#include <UdpSocket.h>

#include <cstring>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <random>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cctype>

#define G 9.81f
#define PI ((float)3.14159265358979323846)

namespace flair {
namespace filter {

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::meta;

//------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------
PositionControl::PositionControl(std::string name,
                               SharedParameters *sharedParameters,
                               uint16_t uavIndex,
                               std::vector<const MetaVrpnObject*> allUavs,
                               MetaVrpnObject *load,
                               TrajectoryGenerator2DCircle *circle)
  : IODevice(getFrameworkManager(), name),
    sharedParameters(sharedParameters),
    load(load),
    circle(circle),
    uavIndex(uavIndex),
    allUavs(allUavs),
    lastUpdateTimeMs(0),
    lastRequestTimeMs(0),
    lastResultTimeMs(0),
    lastStatusUpdateMs(0),
    lastHeartbeatTimeMs(0),
    lastConnectionCheckMs(0),
    connectionTimeoutMs(100),
    myTension(0.0f),
    lastValidTension(0.0f),
    hasValidData(false),
    receiverRunning(false),
    currentDataSource(DataSource::None),
    serverConnected(false),
    serverStatusLabel(nullptr),
    distributionStatusLabel(nullptr),
    optimTimeMatrix(nullptr)
{
    // Set default behavior mode
    behaviourMode = BehaviourMode_t::GoToPosition;
    tensionMode = TensionDistributionMode_t::FixedDistribution;

    // Create UI tabs
    Tab *mainTab = new Tab(getFrameworkManager()->GetTabWidget(), name);
    TabWidget *tw = new TabWidget(mainTab->NewRow(), "laws");
    Tab *setupTab = new Tab(tw, "Setup");
    Tab *graphTab = new Tab(tw, "Graphes");
    Tab *refTab = new Tab(tw, "references");
    Tab *optimizationTab = new Tab(tw, "Optimization");
    Tab *commTab = new Tab(tw, "Communication");
    
    // Create PID controllers for UAV position
    pidxUav = new Pid(setupTab->NewRow(), "PID x Uav", 100);
    pidyUav = new Pid(setupTab->LastRowLastCol(), "PID y Uav", 100);
    pidzUav = new Pid(setupTab->LastRowLastCol(), "PID z Uav", 100);
    
    pidxUav->UseDefaultPlot(graphTab->NewRow());
    pidyUav->UseDefaultPlot(graphTab->LastRowLastCol());
    pidzUav->UseDefaultPlot(graphTab->LastRowLastCol());
    
    // Create PID controllers for load - master only
    if (uavIndex == 0) {
        pidxLoad = new Pid(setupTab->NewRow(), "PID x Load", 100);
        pidyLoad = new Pid(setupTab->LastRowLastCol(), "PID y Load", 100);
        pidzLoad = new Pid(setupTab->LastRowLastCol(), "PID z Load", 100);
        
        pidxLoad->UseDefaultPlot(graphTab->NewRow());
        pidyLoad->UseDefaultPlot(graphTab->LastRowLastCol());
        pidzLoad->UseDefaultPlot(graphTab->LastRowLastCol());
    } else {
        // Null pointers for non-master drones
        pidxLoad = nullptr;
        pidyLoad = nullptr;
        pidzLoad = nullptr;
    }
    
    // Create output matrix (f, quaternion, omega)
    MatrixDescriptor *desc = new MatrixDescriptor(8, 1);
    desc->SetElementName(0, 0, "f");
    desc->SetElementName(1, 0, "q0_d");
    desc->SetElementName(2, 0, "q1_d");
    desc->SetElementName(3, 0, "q2_d");
    desc->SetElementName(4, 0, "q3_d");
    desc->SetElementName(5, 0, "wx_d");
    desc->SetElementName(6, 0, "wy_d");
    desc->SetElementName(7, 0, "wz_d");
    output = new Matrix(this, desc, floatType, name);
    delete desc;
    
    // Create angle reference matrix
    if (uavIndex == 0) {
        desc = new MatrixDescriptor(2, 1);
        desc->SetElementName(0, 0, "phi");
        desc->SetElementName(1, 0, "theta");
    } else {
        desc = new MatrixDescriptor(2, 2);
        desc->SetElementName(0, 0, "phi");
        desc->SetElementName(0, 1, "phi_ref");
        desc->SetElementName(1, 0, "theta");
        desc->SetElementName(1, 1, "theta_ref");
    }
    angleRef = new Matrix(this, desc, floatType, "angle_ref");
    delete desc;
    
    // Create load reference matrix
    desc = new MatrixDescriptor(5, 1);
    desc->SetElementName(0, 0, "load_ref x");
    desc->SetElementName(1, 0, "load_ref y");
    desc->SetElementName(2, 0, "load_ref z");
    desc->SetElementName(3, 0, "load_ref dx");
    desc->SetElementName(4, 0, "load_ref dy");
    loadRef = new Matrix(this, desc, floatType, "load_ref");
    delete desc;
    
    // Create tension data matrix
    desc = new MatrixDescriptor(1, 1);
    desc->SetElementName(0, 0, "My Tension (N)");
    tensionData = new Matrix(this, desc, floatType, "tension_data");
    delete desc;
    
    // Create cable angles matrices only for master drone
    if (uavIndex == 0 && allUavs.size() >= 2) {
        // Create matrix for actual cable angles
        desc = new MatrixDescriptor(allUavs.size(), 1);
        for (size_t i = 0; i < allUavs.size(); i++) {
            size_t j = (i + 1) % allUavs.size();
            std::string label = "Angle Drone_" + std::to_string(i) + "-Drone_" + std::to_string(j) + " (deg)";
            desc->SetElementName(i, 0, label);
        }
        actualCableAnglesData = new Matrix(this, desc, floatType, "cable_angles");
        delete desc;
        
        // Store directions for display
        cableAnglesData = nullptr;
    } else {
        cableAnglesData = nullptr;
        actualCableAnglesData = nullptr;
    }
    
    // Create distribution matrix for visualization
    desc = new MatrixDescriptor(1, 6);
    desc->SetElementName(0, 0, "Sequence");
    desc->SetElementName(0, 1, "Update Time");
    desc->SetElementName(0, 2, "Dir-X");
    desc->SetElementName(0, 3, "Dir-Y");
    desc->SetElementName(0, 4, "Dir-Z");
    desc->SetElementName(0, 5, "Tension");
    distributionMatrix = new Matrix(this, desc, floatType, "distribution");
    delete desc;
    
    // Create optimization time matrix for master only
    if (uavIndex == 0) {
        desc = new MatrixDescriptor(1, 2);
        desc->SetElementName(0, 0, "Sequence");
        desc->SetElementName(0, 1, "Response Time (ms)");
        optimTimeMatrix = new Matrix(this, desc, floatType, "optimization_time");
        delete desc;
    }
    
    // Create UI for optimization visualization
    const DataPlot::Color_t colors[] = {
        DataPlot::Red, DataPlot::Green, DataPlot::Blue, 
        DataPlot::Yellow, DataPlot::Black, DataPlot::White
    };
    
    // Create tension plot
    DataPlot1D *tensionPlot = new DataPlot1D(optimizationTab->NewRow(), "My Cable Tension", 0, 5);
    tensionPlot->AddCurve(tensionData->Element(0, 0), DataPlot::Red, "My Tension");
    
    // Create direction plot
    DataPlot1D *directionPlot = new DataPlot1D(optimizationTab->LastRowLastCol(), "My Cable Direction", -1, 1);
    directionPlot->AddCurve(distributionMatrix->Element(0, 2), DataPlot::Red, "X");
    directionPlot->AddCurve(distributionMatrix->Element(0, 3), DataPlot::Green, "Y");
    directionPlot->AddCurve(distributionMatrix->Element(0, 4), DataPlot::Blue, "Z");
    
    if (uavIndex == 0) {
        // Create cable angle plot
        if (actualCableAnglesData) {
            DataPlot1D *anglesPlot = new DataPlot1D(optimizationTab->NewRow(), "Cable Angles", 0, 180);
            for (size_t i = 0; i < std::min(allUavs.size(), (size_t)4); i++) {
                size_t j = (i + 1) % allUavs.size();
                std::string label = "Drone_" + std::to_string(i) + "-Drone_" + std::to_string(j);
                anglesPlot->AddCurve(actualCableAnglesData->Element(i, 0), colors[i % 6], label);
            }
        }
        
        // Create optimization time plot
        if (optimTimeMatrix) {
            DataPlot1D *timeoutPlot = new DataPlot1D(optimizationTab->LastRowLastCol(), "Optimization Time", 0, 500);
            timeoutPlot->AddCurve(optimTimeMatrix->Element(0, 1), DataPlot::Red, "Response Time (ms)");
        }
    }
    
    // Create server communication UI
    new Label(commTab->NewRow(), "OPTIMIZATION SERVER", true);
    
    // Create status labels
    serverStatusLabel = new Label(commTab->NewRow(), "Server: Not connected");
    distributionStatusLabel = new Label(commTab->LastRowLastCol(), "Distribution: Symmetric");
    
    // Add optimization time display for master
    if (uavIndex == 0 && optimTimeMatrix) {
        DataPlot1D *optimTimePlot = new DataPlot1D(commTab->NewRow(), "Optimization Round Trip Time", 0, 500);
        optimTimePlot->AddCurve(optimTimeMatrix->Element(0, 1), DataPlot::Red, "Time (ms)");
    }
    
    // Setup load plots
    load->AddDataToLog(loadRef);
    load->xPlot()->AddCurve(loadRef->Element(0, 0), DataPlot::Blue);
    load->yPlot()->AddCurve(loadRef->Element(1, 0), DataPlot::Blue);
    load->zPlot()->AddCurve(loadRef->Element(2, 0), DataPlot::Blue);
    load->VxPlot()->AddCurve(loadRef->Element(3, 0), DataPlot::Blue);
    load->VyPlot()->AddCurve(loadRef->Element(4, 0), DataPlot::Blue);
    
    // Setup angle plots
    DataPlot1D *phi_plot = new DataPlot1D(refTab->NewRow(), "phi", -180, 180);
    DataPlot1D *theta_plot = new DataPlot1D(refTab->LastRowLastCol(), "theta", 0, 90);
    
    if (uavIndex == 0) {
        phi_plot->AddCurve(angleRef->Element(0, 0));
        theta_plot->AddCurve(angleRef->Element(1, 0));
    } else {
        phi_plot->AddCurve(angleRef->Element(0, 0));
        phi_plot->AddCurve(angleRef->Element(0, 1), DataPlot::Blue);
        theta_plot->AddCurve(angleRef->Element(1, 0));
        theta_plot->AddCurve(angleRef->Element(1, 1), DataPlot::Blue);
    }
    
    // Create state matrix
    if (uavIndex == 0) {
        desc = new MatrixDescriptor(9, 1);
        desc->SetElementName(0, 0, "ui.x");
        desc->SetElementName(1, 0, "ui.y");
        desc->SetElementName(2, 0, "ui.z");
        desc->SetElementName(3, 0, "perror.x");
        desc->SetElementName(4, 0, "perror.y");
        desc->SetElementName(5, 0, "perror.z");
        desc->SetElementName(6, 0, "ul.x");
        desc->SetElementName(7, 0, "ul.y");
        desc->SetElementName(8, 0, "ul.z");
    } else {
        desc = new MatrixDescriptor(6, 1);
        desc->SetElementName(0, 0, "ui.x");
        desc->SetElementName(1, 0, "ui.y");
        desc->SetElementName(2, 0, "ui.z");
        desc->SetElementName(3, 0, "perror.x");
        desc->SetElementName(4, 0, "perror.y");
        desc->SetElementName(5, 0, "perror.z");
    }
    state = new Matrix(this, desc, floatType, name);
    delete desc;
    
    // Add matrices to log
    AddDataToLog(state);
    AddDataToLog(tensionData);
    AddDataToLog(distributionMatrix);
    
    if (actualCableAnglesData) {
        AddDataToLog(actualCableAnglesData);
    }
    
    if (optimTimeMatrix) {
        AddDataToLog(optimTimeMatrix);
    }
    
    // Initialize symmetric distribution as fallback
    InitializeSymmetricDistribution();
    
    // Set up server configuration and implementation object
    if (sharedParameters->AreValid()) {
        if (sharedParameters->GetUseOptimizedDistribution()) {
            try {
                // Get server settings from shared parameters
                flair::filter::ServerConfig serverConfig;
                serverConfig.serverAddress = sharedParameters->GetServerAddress();
                serverConfig.serverPort = sharedParameters->GetServerPort();
                serverConfig.resultPort = sharedParameters->GetResultPort();
                
                // Create implementation helper object
                impl.reset(new PositionControlImpl(this, 
                                                  ObjectName() + "_impl", 
                                                  uavIndex, 
                                                  serverConfig));
                
                // Set connection callback to update our status
                impl->SetConnectionCallback([this](bool status) {
                    serverConnected.store(status);
                    if (serverStatusLabel) {
                        serverStatusLabel->SetText(status ? "Server: Connected" : "Server: Disconnected");
                    }
                });
                
                Info("Created PositionControlImpl object for advanced server handling");
            } catch (const std::exception& e) {
                Warn("Failed to create PositionControlImpl: %s", e.what());
            }
        }
        
        // Set distribution mode based on current setting
        if (sharedParameters->GetUseOptimizedDistribution()) {
            tensionMode = TensionDistributionMode_t::OptimizedDistribution;
        } else {
            tensionMode = TensionDistributionMode_t::FixedDistribution;
            if (distributionStatusLabel) {
                distributionStatusLabel->SetText("Distribution: Symmetric");
            }
        }
        
        // Initialize fallback mode
        InitializeSymmetricDistribution();
        
        try {
            if (sharedParameters->GetUseOptimizedDistribution()) {
                // Schedule a connection attempt in the background
                Info("Server connection will be attempted in the background");
                
                // Prepare the server communication
                bool connected = InitializeServerConnection();
                if (!connected) {
                    Info("Server connection prepared, will connect in background");
                }
            }
        } catch (...) {
            Warn("Server connection preparation failed, continuing with symmetric distribution");
        }
    }
    
    SetIsReady(true);
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
PositionControl::~PositionControl() {
    // Signal threads to stop
    serverConnected.store(false);
    receiverRunning.store(false);
    
    impl.reset();
    
    // Close TCP client 
    if (tcpClient) {
        try {
            tcpClient->Close();
            tcpClient.reset();
        } catch (...) {
            // Ignore errors during cleanup
        }
    }
    
    // Release socket to unblock receiver thread
    unicastReceiver.reset();
    
    // Wait briefly to allow thread to notice termination flags
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Join with timeout and forced cancellation
    if (udpReceiverThread.joinable()) {
        try {
            auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
            
            while (udpReceiverThread.joinable() && 
                  std::chrono::steady_clock::now() < deadline) {
                // Force thread cancellation for stuck system calls
                pthread_cancel(udpReceiverThread.native_handle());
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            
            if (udpReceiverThread.joinable()) {
                udpReceiverThread.join();
            }
        } catch (...) {
            // Ignore errors during shutdown
        }
    }
}

//------------------------------------------------------------------------------
// Calculate message checksum
//------------------------------------------------------------------------------
uint16_t PositionControl::CalculateChecksum(const void* data, size_t length) {
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
bool PositionControl::InitializeServerConnection() {
    uint64_t currentTime = GetCurrentTimeMs();
    if (currentTime - lastStatusUpdateMs < 1000) {
        return false;
    }
    
    lastStatusUpdateMs = currentTime;
    
    try {
        // Get server settings from shared parameters
        std::string serverAddress = "127.0.0.1"; // Server IP address
        int serverPort = DEFAULT_SERVER_PORT;
        int resultPort = DEFAULT_RESULT_PORT;
        
        if (sharedParameters && sharedParameters->AreValid()) {
            // Override with settings from shared parameters
            serverAddress = sharedParameters->GetServerAddress();
            serverPort = sharedParameters->GetServerPort();
            resultPort = sharedParameters->GetResultPort();
        }
        
        // Initialize TCP client for sending requests (all drones) if not already created
        if (!tcpClient) {
            Info("Creating new TCP client for server at %s:%d", serverAddress.c_str(), serverPort);
            tcpClient.reset(new TcpClient(this, ObjectName() + "_client", serverAddress, serverPort));
        } else if (tcpClient->GetStatus().serverAddress != serverAddress || 
                  tcpClient->GetStatus().serverPort != serverPort) {
            // Server configuration changed, update client
            Info("Updating TCP client to %s:%d", serverAddress.c_str(), serverPort);
            tcpClient->SetServer(serverAddress, serverPort);
        }
        
        // Initialize UDP unicast socket for receiving results
        if (!unicastReceiver) {
            try {
                // For simulation: calculate port based on UAV index to avoid port conflicts
                int calculatedPort = resultPort + uavIndex;
                
                Info("Creating new UDP unicast socket on port %d", calculatedPort);
                // FIXED: Use port-only constructor to create a properly bound receiving socket
                unicastReceiver.reset(new UdpSocket(this, ObjectName() + "_unicast", calculatedPort));
                
                // Start UDP receiver thread
                if (!receiverRunning.load()) {
                    StartUdpReceiverThread();
                }
            } catch (const std::exception& e) {
                Err("Failed to create UDP socket: %s", e.what());
            }
        }
        
        // Test connection
        bool connected = false;
        if (tcpClient) {
            // Only attempt connection if not already connected
            if (tcpClient->IsConnected()) {
                connected = true;
            } else {
                // Use a very short timeout for the initial connection attempt
                Info("Testing server connection with %dms timeout", connectionTimeoutMs);
                connected = tcpClient->Connect(connectionTimeoutMs);
            }
        }
        
        // Update connection status
        bool wasConnected = serverConnected.load();
        serverConnected.store(connected);
        
        // Log connection status changes
        if (connected && !wasConnected) {
            Info("Server connection established");
        } else if (!connected && wasConnected) {
            Warn("Server connection lost");
        }
        
        // Update status label
        if (serverStatusLabel) {
            std::stringstream ss;
            ss << "Server: " << (connected ? "Connected" : "Disconnected");
            serverStatusLabel->SetText(ss.str().c_str());
        }
        
        // Update SharedParameters connection status
        if (sharedParameters) {
            sharedParameters->UpdateOptimizerStatus(connected);
        }
        
        // Return connection status
        return connected;
    }
    catch (const std::exception& e) {
        // Log error
        Err("Server connection error: %s", e.what());
        
        // Update connection status
        serverConnected.store(false);
        
        // Update status label
        if (serverStatusLabel) {
            serverStatusLabel->SetText("Server: Connection error");
        }
        
        // Fall back to symmetric distribution
        if (tensionMode == TensionDistributionMode_t::OptimizedDistribution) {
            tensionMode = TensionDistributionMode_t::FixedDistribution;
            if (distributionStatusLabel) {
                distributionStatusLabel->SetText("Distribution: Fallback to Symmetric");
            }
            Info("Falling back to symmetric distribution");
            InitializeSymmetricDistribution();
        }
        
        return false;
    }
}

//------------------------------------------------------------------------------
// Start UDP receiver thread
//------------------------------------------------------------------------------
void PositionControl::StartUdpReceiverThread() {
    // Simple wrapper implementation that maintains existing functionality
    if (!receiverRunning.load() && unicastReceiver) {
        // Set flag
        receiverRunning.store(true);
        
        // Start thread
        try {
            udpReceiverThread = std::thread(&PositionControl::UdpReceiverLoop, this);
        } catch (const std::exception& e) {
            Err("Failed to start UDP receiver thread: %s", e.what());
            receiverRunning.store(false);
        }
    }
}

//------------------------------------------------------------------------------
// UDP receiver thread function - dedicated thread for unicast messages
//------------------------------------------------------------------------------
void PositionControl::UdpReceiverLoop() {
    Info("UDP receiver thread started");
    
    // Buffer for receiving UDP messages
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
            
            uint32_t magic = *((uint32_t*)buffer);
            
            // Process unicast message (new format)
            if (magic == UNICAST_MAGIC && received >= sizeof(UdpUnicastMessage)) {
                UdpUnicastMessage* msg = reinterpret_cast<UdpUnicastMessage*>(buffer);
                
                // Check checksum and drone index
                uint16_t calculatedChecksum = CalculateChecksum(
                    msg, sizeof(UdpUnicastMessage) - sizeof(msg->checksum));
                    
                if (calculatedChecksum == msg->checksum && msg->droneIndex == uavIndex) {
                    // Extract data
                    Vector3Df direction(msg->direction[0], msg->direction[1], msg->direction[2]);
                    float tension = msg->tension;
                    
                    // Check for fallback flag
                    if (direction.z >= -900 || direction.z <= -1100 || tension >= -0.5) {
                        ProcessUnicastData(msg->sequenceNumber, direction, tension);
                    } else {
                        // Special flag recognized - use local symmetric fallback
                        Info("Received fallback flag from server - using symmetric distribution");
                        InitializeSymmetricDistribution();
                        serverConnected.store(false);
                    }
                }
            }
            // Process broadcast message
            else if (magic == BROADCAST_MAGIC && received >= sizeof(UdpBroadcastMessage)) {
                UdpBroadcastMessage* msg = reinterpret_cast<UdpBroadcastMessage*>(buffer);
                
                // Check checksum and drone index
                uint16_t calculatedChecksum = CalculateChecksum(
                    msg, sizeof(UdpBroadcastMessage) - sizeof(msg->checksum));
                    
                if (calculatedChecksum == msg->checksum && uavIndex < msg->droneCount) {
                    // Extract data
                    Vector3Df direction(
                        msg->directions[uavIndex][0],
                        msg->directions[uavIndex][1],
                        msg->directions[uavIndex][2]
                    );
                    
                    ProcessUnicastData(msg->sequenceNumber, direction, msg->tensions[uavIndex]);
                }
            }
        }
        catch (const std::exception&) {
            // Check termination flag after exception
            if (!receiverRunning.load()) break;
        }
    }
    
    Info("UDP receiver thread loop has ended");
}

//------------------------------------------------------------------------------
// Process unicast data received from server
//------------------------------------------------------------------------------
bool PositionControl::ProcessUnicastData(uint32_t sequence, const Vector3Df& direction, float tension) {
    // Validate values
    if (std::isnan(direction.x) || std::isnan(direction.y) || std::isnan(direction.z) ||
        std::isinf(direction.x) || std::isinf(direction.y) || std::isinf(direction.z)) {
        return false;
    }
    
    if (std::isnan(tension) || std::isinf(tension) || tension < 0.0f) {
        return false;
    }
    
    // Update internal state with optimized values
    std::lock_guard<std::mutex> lock(optimizationMutex);
    
    // Store raw values
    myDirection = direction;
    myTension = tension;
    
    // Normalize direction
    float norm = myDirection.GetNorm();
    if (norm > 0.0001f) {
        myDirection = myDirection / norm;
    } else {
        myDirection = Vector3Df(0, 0, 1);
    }
    
    // Verify tension is non-zero (fix very small or zero tensions)
    if (myTension <= 0.001f) {
        auto p = sharedParameters->GetDatas();
        float defaultTension = (p.mLoad * G) / allUavs.size();
        myTension = defaultTension;
    }
    
    // Store as valid fallback data
    lastValidDirection = myDirection;
    lastValidTension = myTension;
    hasValidData = true;
    
    // Update data source and timestamps
    currentDataSource = DataSource::ServerDirect;
    lastResultTimeMs = GetCurrentTimeMs();
    
    // Update optimization time tracking for master only
    if (uavIndex == 0 && optimTimeMatrix) {
        // Look up request time for this sequence
        auto requestTimeIt = requestTimes.find(sequence);
        if (requestTimeIt != requestTimes.end()) {
            uint64_t requestTime = requestTimeIt->second;
            uint64_t roundTripTime = GetCurrentTimeMs() - requestTime;
            
            // Update optimization time matrix
            optimTimeMatrix->GetMutex();
            optimTimeMatrix->SetValueNoMutex(0, 0, sequence);
            optimTimeMatrix->SetValueNoMutex(0, 1, roundTripTime);
            optimTimeMatrix->ReleaseMutex();
            
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
    
    // Update visualization matrix
    UpdateDistributionMatrix(sequence);
    
    // Update UI status
    if (distributionStatusLabel) {
        distributionStatusLabel->SetText("Distribution: Optimized");
    }
    
    // Update server connection status
    serverConnected.store(true);
    
    return true;
}

//------------------------------------------------------------------------------
// Update distribution matrix with current values
//------------------------------------------------------------------------------
void PositionControl::UpdateDistributionMatrix(uint32_t sequence) {
    distributionMatrix->GetMutex();
    distributionMatrix->SetValueNoMutex(0, 0, sequence);
    distributionMatrix->SetValueNoMutex(0, 1, GetCurrentTimeMs());
    distributionMatrix->SetValueNoMutex(0, 2, myDirection.x);
    distributionMatrix->SetValueNoMutex(0, 3, myDirection.y);
    distributionMatrix->SetValueNoMutex(0, 4, myDirection.z);
    distributionMatrix->SetValueNoMutex(0, 5, myTension);
    distributionMatrix->ReleaseMutex();
}

//------------------------------------------------------------------------------
// Test server connection
//------------------------------------------------------------------------------
bool PositionControl::TestServerConnection(int timeoutMs) {
    // Check if client exists
    if (!tcpClient) {
        // Try initializing the connection, don't continue if it fails
        if (!InitializeServerConnection()) {
            serverConnected.store(false);
            return false;
        }
    }
    
    // Check if we already have an active connection
    if (tcpClient->IsConnected()) {
        serverConnected.store(true);
        return true;
    }
    
    // Try to connect with a limited timeout to avoid blocking the control loop
    try {
        // Use the shortest possible timeout between the requested value and 50ms
        int actualTimeout = std::min(timeoutMs, 50);
        
        // Try to connect
        bool connected = tcpClient->Connect(actualTimeout);
        serverConnected.store(connected);
        
        // Update status label
        if (serverStatusLabel) {
            std::stringstream ss;
            ss << "Server: " << (connected ? "Connected" : "Connection failed");
            serverStatusLabel->SetText(ss.str().c_str());
        }
        
        // Update SharedParameters connection status
        if (sharedParameters) {
            sharedParameters->UpdateOptimizerStatus(connected);
        }
        
        return connected;
    } catch (const std::exception& e) {
        // Log error and return false
        Err("TestServerConnection error: %s", e.what());
        serverConnected.store(false);
        return false;
    }
}

//------------------------------------------------------------------------------
// Check if server is connected
//------------------------------------------------------------------------------
bool PositionControl::IsServerConnected() const {
    return serverConnected.load();
}

//------------------------------------------------------------------------------
// Set tension distribution mode
//------------------------------------------------------------------------------
bool PositionControl::SetTensionDistributionMode(TensionDistributionMode_t mode) {
    // Only change if different
    if (mode == tensionMode) {
        return true;
    }
    
    if (mode == TensionDistributionMode_t::OptimizedDistribution) {
        // Ensure connection is established
        if (!TestServerConnection(200)) {
            return false;
        }
    }
    
    // Update mode
    tensionMode = mode;
    
    // Update status
    if (distributionStatusLabel) {
        distributionStatusLabel->SetText(
            mode == TensionDistributionMode_t::OptimizedDistribution ? 
            "Distribution: Optimized" : "Distribution: Symmetric");
    }
    
    return true;
}

//------------------------------------------------------------------------------
// Get current tension distribution mode
//------------------------------------------------------------------------------
PositionControl::TensionDistributionMode_t PositionControl::GetTensionDistributionMode() const {
    return tensionMode;
}

//------------------------------------------------------------------------------
// Calculate symmetric distribution
//------------------------------------------------------------------------------
void PositionControl::CalculateSymmetricDistribution(std::vector<Vector3Df>& directions, 
                                                   std::vector<float>& tensions) {
    // Make sure vectors are the right size
    directions.resize(allUavs.size());
    tensions.resize(allUavs.size());
    
    auto params = sharedParameters->GetDatas();
    
    // Set up symmetrically distributed directions for slave drones (exactly as original)
    for (size_t i = 1; i < allUavs.size(); i++) {
        float angle = 2 * PI / allUavs.size() * i;
        float thetaRad = Euler::ToRadian(params.thetaDesired);
        
        // Calculate direction vector - exactly as in the original code
        directions[i].x = -sinf(thetaRad) * cosf(angle);
        directions[i].y = -sinf(thetaRad) * sinf(angle);
        directions[i].z = cosf(thetaRad);
        
        // Set fixed tensions for slave drones - exactly as in the original code
        tensions[i] = params.mLoad * G / allUavs.size();
    }
    
    // For master drone (index 0), calculate based on resultant force
    if (uavIndex == 0) {
        // Calculate load PID control
        Vector3Df loadPos, loadSpeed;
        load->GetPosition(loadPos);
        load->GetSpeed(loadSpeed);
        
        Vector3Df loadPosD, loadSpeedD;
        if (behaviourMode == BehaviourMode_t::GoToPosition) {
            loadPosD.x = params.xDesired;
            loadPosD.y = params.yDesired;
            loadPosD.z = params.zDesired;
            loadSpeedD = Vector3Df(0, 0, 0);
        } else {
            Vector2Df circlePos2D, circleVel2D;
            circle->GetPosition(circlePos2D);
            circle->GetSpeed(circleVel2D);
            
            loadPosD.x = circlePos2D.x;
            loadPosD.y = circlePos2D.y;
            loadPosD.z = params.zDesired;
            loadSpeedD = Vector3Df(circleVel2D.x, circleVel2D.y, 0);
        }
        
        // Run PID controllers for load position
        pidxLoad->SetValues(loadPos.x - loadPosD.x, loadSpeed.x - loadSpeedD.x);
        pidyLoad->SetValues(loadPos.y - loadPosD.y, loadSpeed.y - loadSpeedD.y);
        pidzLoad->SetValues(loadPos.z - loadPosD.z, loadSpeed.z - loadSpeedD.z);
        
        pidxLoad->Update(GetTime());
        pidyLoad->Update(GetTime());
        pidzLoad->Update(GetTime());
        
        // Calculate load force ul - exactly as in the original code
        Vector3Df ul = params.mLoad * (G * Vector3Df(0, 0, 1)) + 
                      Vector3Df(pidxLoad->Output(), pidyLoad->Output(), pidzLoad->Output());
        
        // Calculate master drone tension and direction (t0D_alpha0D)
        Vector3Df t0D_alpha0D = ul;
        for (size_t i = 1; i < allUavs.size(); i++) {
            t0D_alpha0D -= tensions[i] * directions[i];
        }
        
        // Get tension and normalize direction
        tensions[0] = t0D_alpha0D.GetNorm();
        directions[0] = t0D_alpha0D / tensions[0];
    }
}

//------------------------------------------------------------------------------
// Calculate symmetric distribution with pre-calculated ul - avoids duplicate calculation
//------------------------------------------------------------------------------
void PositionControl::CalculateSymmetricDistributionWithUl(std::vector<Vector3Df>& directions, 
                                                         std::vector<float>& tensions,
                                                         const Vector3Df& ul) {
    // Make sure vectors are the right size
    directions.resize(allUavs.size());
    tensions.resize(allUavs.size());
    
    auto params = sharedParameters->GetDatas();
    
    // Set up symmetrically distributed directions for slave drones
    for (size_t i = 1; i < allUavs.size(); i++) {
        float angle = 2 * PI / allUavs.size() * i;
        float thetaRad = Euler::ToRadian(params.thetaDesired);
        
        // Calculate direction vector
        directions[i].x = -sinf(thetaRad) * cosf(angle);
        directions[i].y = -sinf(thetaRad) * sinf(angle);
        directions[i].z = cosf(thetaRad);
        
        // Set fixed tensions for slave drones
        tensions[i] = params.mLoad * G / allUavs.size();
    }
    
    if (uavIndex == 0) {
        // Calculate master drone tension and direction (t0D_alpha0D) using the provided ul
        Vector3Df t0D_alpha0D = ul;
        for (size_t i = 1; i < allUavs.size(); i++) {
            t0D_alpha0D -= tensions[i] * directions[i];
        }
        
        // Get tension and normalize direction
        tensions[0] = t0D_alpha0D.GetNorm();
        directions[0] = t0D_alpha0D / tensions[0];
    }
}

//------------------------------------------------------------------------------
// Initialize symmetric distribution as fallback
//------------------------------------------------------------------------------
void PositionControl::InitializeSymmetricDistribution() {
    if (!sharedParameters->AreValid()) return;
    
    // Calculate symmetric distribution
    std::vector<Vector3Df> directions;
    std::vector<float> tensions;
    
    // Check if we already have an ul value calculated in the state matrix
    if (uavIndex == 0 && state) {
        // Try to use the existing ul if available
        state->GetMutex();
        bool hasUlValues = true;
        Vector3Df ul;
        
        // Read values if they exist
        ul.x = state->ValueNoMutex(6, 0);
        ul.y = state->ValueNoMutex(7, 0);
        ul.z = state->ValueNoMutex(8, 0);
        state->ReleaseMutex();
        
        if (hasUlValues && (ul.GetNorm() > 0.001f)) {
            // Use the pre-calculated ul
            CalculateSymmetricDistributionWithUl(directions, tensions, ul);
        } else {
            // Use the original method if no valid ul is available
            CalculateSymmetricDistribution(directions, tensions);
        }
    } else {
        // For non-master drones, use the original method
        CalculateSymmetricDistribution(directions, tensions);
    }
    
    // Store my own direction and tension
    myDirection = directions[uavIndex];
    myTension = tensions[uavIndex];
    
    // Set as current data source
    currentDataSource = DataSource::Symmetric;
    
    // Store as valid data
    lastValidDirection = myDirection;
    lastValidTension = myTension;
    hasValidData = true;
    
    // Update distribution matrix for visualization
    distributionMatrix->GetMutex();
    distributionMatrix->SetValueNoMutex(0, 0, 0); // Zero sequence for symmetric
    distributionMatrix->SetValueNoMutex(0, 1, GetCurrentTimeMs());
    distributionMatrix->SetValueNoMutex(0, 2, myDirection.x);
    distributionMatrix->SetValueNoMutex(0, 3, myDirection.y);
    distributionMatrix->SetValueNoMutex(0, 4, myDirection.z);
    distributionMatrix->SetValueNoMutex(0, 5, myTension);
    distributionMatrix->ReleaseMutex();
    
    // Update status
    if (distributionStatusLabel) {
        if (sharedParameters->GetUseOptimizedDistribution()) {
            distributionStatusLabel->SetText("Distribution: Fallback to Symmetric");
        } else {
            distributionStatusLabel->SetText("Distribution: Symmetric");
        }
    }
}

//------------------------------------------------------------------------------
// Send heartbeat message to server
//------------------------------------------------------------------------------
void PositionControl::SendHeartbeatMessage() {
    // Only send if we're the master drone and server connection is established
    if (uavIndex != 0 || !tcpClient || !tcpClient->IsConnected()) {
        return;
    }
    
    // Only send every HEARTBEAT_INTERVAL_MS
    uint64_t currentTime = GetCurrentTimeMs();
    if (currentTime - lastHeartbeatTimeMs < HEARTBEAT_INTERVAL_MS) {
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
    if (tcpClient) {
        try {
            tcpClient->SendRaw(&heartbeat, sizeof(heartbeat));
        }
        catch (const std::exception&) {
            // Silent fail for heartbeat
        }
    }
}

//------------------------------------------------------------------------------
// Send resultant force to server
//------------------------------------------------------------------------------
bool PositionControl::SendResultantForce(const Vector3Df& resultantForce) {
    // Only master sends resultant force
    if (uavIndex != 0) {
        return false;
    }
    
    // Check if TCP client is available
    if (!tcpClient) {
        if (!InitializeServerConnection()) {
            // Fall back to symmetric distribution if server unavailable
            if (tensionMode == TensionDistributionMode_t::OptimizedDistribution) {
                tensionMode = TensionDistributionMode_t::FixedDistribution;
                if (distributionStatusLabel) {
                    distributionStatusLabel->SetText("Distribution: Fallback to Symmetric");
                }
                InitializeSymmetricDistribution();
            }
            return false;
        }
    }
    
    // Check connection state
    if (!tcpClient->IsConnected()) {
        // Try to reconnect with brief timeout
        if (!tcpClient->Connect(connectionTimeoutMs)) {
            // Connection failed, update status
            serverConnected.store(false);
            if (serverStatusLabel) {
                serverStatusLabel->SetText("Server: Disconnected");
            }
            
            // Fall back to symmetric distribution if using optimization
            if (tensionMode == TensionDistributionMode_t::OptimizedDistribution) {
                tensionMode = TensionDistributionMode_t::FixedDistribution;
                if (distributionStatusLabel) {
                    distributionStatusLabel->SetText("Distribution: Fallback to Symmetric");
                }
                InitializeSymmetricDistribution();
            }
            
            return false;
        }
        
        // Connection restored
        serverConnected.store(true);
        if (serverStatusLabel) {
            serverStatusLabel->SetText("Server: Connected");
        }
    }
    
    // Limit sending frequency
    uint64_t currentTime = GetCurrentTimeMs();
    if (currentTime - lastRequestTimeMs < 50) { // Max 20 Hz
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
        bool sent = tcpClient->SendRaw(&request, sizeof(request));
        
        if (sent) {
            lastRequestTimeMs = currentTime;
            return true;
        } else {
            // Send failed - update connection status
            if (!tcpClient->IsConnected()) {
                serverConnected.store(false);
                if (serverStatusLabel) {
                    serverStatusLabel->SetText("Server: Connection lost");
                }
                
                // Fall back to symmetric distribution if using optimization
                if (tensionMode == TensionDistributionMode_t::OptimizedDistribution) {
                    tensionMode = TensionDistributionMode_t::FixedDistribution;
                    if (distributionStatusLabel) {
                        distributionStatusLabel->SetText("Distribution: Fallback to Symmetric");
                    }
                    InitializeSymmetricDistribution();
                }
            }
            
            return false;
        }
    }
    catch (const std::exception&) {
        // Fall back to symmetric distribution if using optimization
        if (tensionMode == TensionDistributionMode_t::OptimizedDistribution) {
            tensionMode = TensionDistributionMode_t::FixedDistribution;
            if (distributionStatusLabel) {
                distributionStatusLabel->SetText("Distribution: Fallback to Symmetric");
            }
            InitializeSymmetricDistribution();
        }
        
        return false;
    }
}

//------------------------------------------------------------------------------
// Calculate cable angles
//------------------------------------------------------------------------------
void PositionControl::CalculateCableAngles(
    const std::vector<Vector3Df>& directions, std::vector<float>& angles)
{
    // Only master drone calculates angles
    if (uavIndex != 0) return;
    
    // Resize output vector if needed
    if (angles.size() != directions.size()) {
        angles.resize(directions.size());
    }
    
    // Calculate angle between each pair of adjacent cables
    for (size_t i = 0; i < directions.size(); i++) {
        size_t j = (i + 1) % directions.size(); // Next drone, wrapping around
        
        // Calculate dot product
        float dotProduct = DotProduct(directions[i], directions[j]);
        
        // Clamp to valid range for acos
        dotProduct = std::max(-1.0f, std::min(1.0f, dotProduct));
        
        // Calculate angle in degrees
        angles[i] = Euler::ToDegree(acosf(dotProduct));
    }
}

//------------------------------------------------------------------------------
// Calculate actual cable angles
//------------------------------------------------------------------------------
void PositionControl::CalculateActualCableAngles() {
    // Only master drone calculates angles
    if (uavIndex != 0 || !actualCableAnglesData) {
        return;
    }
    
    // Get load position
    Vector3Df loadPos;
    load->GetPosition(loadPos);
    
    // Calculate actual cable directions
    std::vector<Vector3Df> actualDirections(allUavs.size());
    
    for (size_t i = 0; i < allUavs.size(); i++) {
        Vector3Df uavPos;
        allUavs[i]->GetPosition(uavPos);
        
        // Vector from load to UAV (actual cable direction)
        Vector3Df direction = uavPos - loadPos;
        
        // Normalize
        float norm = direction.GetNorm();
        if (norm > 0.0001f) {
            direction = direction / norm;
        } else {
            direction = Vector3Df(0, 0, 1); // Default if too close
        }
        
        actualDirections[i] = direction;
    }
    
    // Calculate angles between adjacent cables
    std::vector<float> actualAngles(allUavs.size());
    CalculateCableAngles(actualDirections, actualAngles);
    
    // Update actual angles matrix
    actualCableAnglesData->GetMutex();
    for (size_t i = 0; i < allUavs.size(); i++) {
        actualCableAnglesData->SetValueNoMutex(i, 0, actualAngles[i]);
    }
    actualCableAnglesData->SetDataTime(GetTime());
    actualCableAnglesData->ReleaseMutex();
    
    ProcessUpdate(actualCableAnglesData);
}

//------------------------------------------------------------------------------
// Update tension visualization
//------------------------------------------------------------------------------
void PositionControl::UpdateTensionVisualization() {
    if (!tensionData || !sharedParameters->AreValid()) {
        return;
    }
    
    // Update tension for this drone
    tensionData->GetMutex();
    tensionData->SetValueNoMutex(0, 0, myTension);
    tensionData->SetDataTime(GetTime());
    tensionData->ReleaseMutex();
    
    ProcessUpdate(tensionData);
}

//------------------------------------------------------------------------------
// Check connection status and attempt recovery if needed
//------------------------------------------------------------------------------
void PositionControl::CheckConnectionStatus() {
    // Periodically check connection status and try to recover if needed
    uint64_t currentTime = GetCurrentTimeMs();
    
    // Use adaptive time based on connection status
    uint64_t checkInterval = serverConnected.load() ? 10000 : 5000;  // Less frequent if connected
    
    // Check only at intervals
    if (currentTime - lastConnectionCheckMs <= checkInterval) {
        return;  // Not time to check yet
    }
    
    // Update timestamp
    lastConnectionCheckMs = currentTime;
    
    // Check if we're using optimization but server is disconnected
    if (tensionMode == TensionDistributionMode_t::OptimizedDistribution && 
        (!tcpClient || !tcpClient->IsConnected())) {
        
        // Try to reconnect
        if (!InitializeServerConnection() || (tcpClient && !tcpClient->TryReconnect())) {
            // If still can't connect, fall back to symmetric distribution
            tensionMode = TensionDistributionMode_t::FixedDistribution;
            if (distributionStatusLabel) {
                distributionStatusLabel->SetText("Distribution: Fallback to Symmetric");
            }
            InitializeSymmetricDistribution();
        } else {
            // Connection restored
            serverConnected.store(true);
            if (serverStatusLabel) {
                serverStatusLabel->SetText("Server: Connected");
            }
        }
    }
    
    // If we're in fallback mode, periodically try to restore optimization
    if (sharedParameters->GetUseOptimizedDistribution() && 
        tensionMode == TensionDistributionMode_t::FixedDistribution) {
        
        // Try to connect to server
        if (InitializeServerConnection() && 
            tcpClient && tcpClient->IsConnected()) {
            
            // If connected, switch back to optimization
            tensionMode = TensionDistributionMode_t::OptimizedDistribution;
            if (distributionStatusLabel) {
                distributionStatusLabel->SetText("Distribution: Optimized");
            }
        }
    }
}

//------------------------------------------------------------------------------
// Update connection status display
//------------------------------------------------------------------------------
void PositionControl::UpdateConnectionStatusDisplay() {
    if (!serverStatusLabel) return;
    
    uint64_t currentTime = GetCurrentTimeMs();
    const uint64_t UPDATE_INTERVAL_MS = 5000;
    
    if (currentTime - lastStatusUpdateMs < UPDATE_INTERVAL_MS) {
        return;  // Not time to update yet
    }
    
    lastStatusUpdateMs = currentTime;
    
    // Build status string
    std::stringstream ss;
    ss << "Server: ";
    
    bool isConnected = (tcpClient && tcpClient->IsConnected());
    ss << (isConnected ? "Connected" : "Disconnected");
    
    // Update label
    serverStatusLabel->SetText(ss.str().c_str());
}

//------------------------------------------------------------------------------
// Force reconnection (diagnostic function)
//------------------------------------------------------------------------------
void PositionControl::ForceReconnect() {
    // Force reconnection to the server
    if (tcpClient) {
        // Try to reconnect with a longer timeout for definitive answer
        bool success = tcpClient->Connect(500);
        serverConnected.store(success);
        
        // Update status
        if (serverStatusLabel) {
            std::stringstream ss;
            ss << "Server: " << (success ? "Connected" : "Connection failed");
            serverStatusLabel->SetText(ss.str().c_str());
        }
    } else {
        // Initialize connection if client doesn't exist
        InitializeServerConnection();
    }
}

//------------------------------------------------------------------------------
// Update control - main entry point
//------------------------------------------------------------------------------
void PositionControl::Update() {
    UpdateFrom(NULL);
}

//------------------------------------------------------------------------------
// Update control from data
//------------------------------------------------------------------------------
void PositionControl::UpdateFrom(const io_data *data) {
    // Skip if parameters not valid
    if (!sharedParameters->AreValid()) return;
    SharedParameters::datas_t params=sharedParameters->GetDatas();
    
    // Check connection status and try to recover if needed
    CheckConnectionStatus();
    
    // Try to initialize server connection if needed
    if (sharedParameters->GetUseOptimizedDistribution() && 
        (!tcpClient || !unicastReceiver)) {
        InitializeServerConnection();
    }
    
    // Send heartbeat to server (master only)
    if (uavIndex == 0 && sharedParameters->GetUseOptimizedDistribution() && tcpClient) {
        SendHeartbeatMessage();
    }
    
    // Get current time
    uint64_t currentTime = GetCurrentTimeMs();
    
    // Check for timeout on server data
    if (currentDataSource == DataSource::ServerDirect && 
        currentTime - lastResultTimeMs > RESULT_TIMEOUT_MS) {
        
        if (hasValidData) {
            // Fall back to last valid data
            myDirection = lastValidDirection;
            myTension = lastValidTension;
            currentDataSource = DataSource::ServerFallback;
            
            if (distributionStatusLabel) {
                distributionStatusLabel->SetText("Distribution: Fallback to last valid data");
            }
        } else {
            // Fall back to symmetric distribution
            InitializeSymmetricDistribution();
        }
    }
    
    // Update connection status display occasionally
    if (currentTime - lastStatusUpdateMs > 5000) {
        UpdateConnectionStatusDisplay();
        lastStatusUpdateMs = currentTime;
    }
    
    // Get UAV and load positions and velocities
    Vector3Df uavPos, uavSpeed;
    allUavs[uavIndex]->GetPosition(uavPos);
    allUavs[uavIndex]->GetSpeed(uavSpeed);
    
    Vector3Df loadPos, loadSpeed;
    load->GetPosition(loadPos);
    load->GetSpeed(loadSpeed);
    
    // Calculate desired load position and derivatives
    Vector3Df loadPosD, loadSpeedD, loadAccD, loadJerkD;
    
    if (behaviourMode == BehaviourMode_t::GoToPosition) {
        loadPosD.x = params.xDesired;
        loadPosD.y = params.yDesired;
        loadPosD.z = params.zDesired;
        loadSpeedD = Vector3Df(0, 0, 0);
        loadAccD = Vector3Df(0, 0, 0);
        loadJerkD = Vector3Df(0, 0, 0);
    }
    if (behaviourMode == BehaviourMode_t::CircleTrajectory) {
        Vector2Df circlePos2D, circleVel2D, circleAcc2D, circleJerk2D;
        circle->GetPosition(circlePos2D);
        circle->GetSpeed(circleVel2D);
        circle->GetAcceleration(circleAcc2D);
        circle->GetJerk(circleJerk2D);
        
        loadPosD.x = circlePos2D.x;
        loadPosD.y = circlePos2D.y;
        loadPosD.z = params.zDesired;
        loadSpeedD = Vector3Df(circleVel2D.x, circleVel2D.y, 0);
        loadAccD = Vector3Df(circleAcc2D.x, circleAcc2D.y, 0);
        loadJerkD = Vector3Df(circleJerk2D.x, circleJerk2D.y, 0);
    }
    
    // Only master calculates cable angles
    if (uavIndex == 0) {
        CalculateActualCableAngles();
    }
    
    // Master drone calculates load control
    Vector3Df alphaSelfDp(0, 0, 0); // Initialize to zero vector exactly as in original code
    if (uavIndex == 0) {
        // Calculate load control
        pidxLoad->SetValues(loadPos.x-loadPosD.x,loadSpeed.x-loadSpeedD.x);
        pidyLoad->SetValues(loadPos.y-loadPosD.y,loadSpeed.y-loadSpeedD.y);
        pidzLoad->SetValues(loadPos.z-loadPosD.z,loadSpeed.z-loadSpeedD.z);
        
        pidxLoad->Update(GetTime());
        pidyLoad->Update(GetTime());
        pidzLoad->Update(GetTime());

        Vector3Df ul=params.mLoad*(G*Vector3Df(0,0,1))+Vector3Df(pidxLoad->Output(),pidyLoad->Output(),pidzLoad->Output());
        
        // Store for visualization
        state->GetMutex();
        state->SetValueNoMutex(6, 0, ul.x);
        state->SetValueNoMutex(7, 0, ul.y);
        state->SetValueNoMutex(8, 0, ul.z);
        state->ReleaseMutex();
        
        // Send to server if optimization enabled
        if (sharedParameters->GetUseOptimizedDistribution()) {
            SendResultantForce(ul);
        }
    }
    
    // Get current cable direction and tension based on current mode
    Vector3Df cableDir;
    float tension;
    
    if (sharedParameters->GetUseOptimizedDistribution()) {
        // When optimization is enabled, always use whatever values we have
        cableDir = myDirection;
        tension = myTension;
        
        // Ensure non-zero tensions
        if (tension <= 0.001f) {
            float defaultTension = (params.mLoad * G) / allUavs.size();
            tension = defaultTension;
            myTension = tension;
        }
    } else {
        // Use symmetric distribution per original implementation
        std::vector<Vector3Df> directions;
        std::vector<float> tensions;
        
        // Reuse the already calculated ul for master drone (avoid duplicate calculation)
        if (uavIndex == 0) {
            Vector3Df ul;
            state->GetMutex();
            ul.x = state->ValueNoMutex(6, 0);
            ul.y = state->ValueNoMutex(7, 0);
            ul.z = state->ValueNoMutex(8, 0);
            state->ReleaseMutex();
            CalculateSymmetricDistributionWithUl(directions, tensions, ul);
        } else {
            // For slave drones, use the original method
            CalculateSymmetricDistribution(directions, tensions);
        }
        
        // Use my own direction and tension
        cableDir = directions[uavIndex];
        tension = tensions[uavIndex];
        
        // Update stored values
        myDirection = cableDir;
        myTension = tension;
        currentDataSource = DataSource::Symmetric;
    }
    
    // Calculate position error for UAV
    Vector3Df perror = uavPos - loadPosD + params.lCable * cableDir;
    Vector3Df derror = uavSpeed - loadSpeedD + params.lCable * alphaSelfDp;
    
    // Run position PID
    pidxUav->SetValues(perror.x, derror.x);
    pidyUav->SetValues(perror.y, derror.y);
    pidzUav->SetValues(perror.z, derror.z);
    
    pidxUav->Update(GetTime());
    pidyUav->Update(GetTime());
    pidzUav->Update(GetTime());
    
    // Calculate control input
    Vector3Df vi(-pidxUav->Output(), -pidyUav->Output(), -pidzUav->Output());
    
    // Calculate UAV control force
    Vector3Df ui = params.mUav * (Vector3Df(0, 0, -G) + loadAccD) - tension * cableDir + vi;
    
    // Calculate derivative of control (uip) - FIXED to match original code exactly
    Vector3Df vip;
    vip.x = -pidxUav->GetKp() * (uavSpeed.x - loadSpeedD.x) - pidxUav->GetKi() * (uavPos.x - loadPosD.x + params.lCable * cableDir.x);
    vip.y = -pidyUav->GetKp() * (uavSpeed.y - loadSpeedD.y) - pidyUav->GetKi() * (uavPos.y - loadPosD.y + params.lCable * cableDir.y);
    vip.z = -pidzUav->GetKp() * (uavSpeed.z - loadSpeedD.z) - pidzUav->GetKi() * (uavPos.z - loadPosD.z + params.lCable * cableDir.z);
    Vector3Df uip = params.mUav * loadJerkD + vip;
    
    // Calculate desired quaternion and angular velocity
    float psi_d = 0;  // Desired yaw angle
    float psip_d = 0; // Desired yaw rate
    
    Vector3Df uu, uup;
    float norm = ui.GetNorm();
    float norm3 = norm*norm*norm;
    
    uu = ui;
    uu.Normalize();
    
    // Calculate dot product
    float u = DotProduct(ui, uip);

    uup.x = uip.x / norm - ui.x * u / norm3;
    uup.y = uip.y / norm - ui.y * u / norm3;
    uup.z = uip.z / norm - ui.z * u / norm3;
    
    float u_3= sqrtf( -2*uu.z+2);
    
    // Create quaternion from direction
    Quaternion refQuaternion;
    refQuaternion.q0 = u_3 * cosf(psi_d / 2) / 2;
    refQuaternion.q1 = (-uu.x * sinf(psi_d / 2) + uu.y * cosf(psi_d / 2)) / u_3;
    refQuaternion.q2 = (-uu.x * cosf(psi_d / 2) - uu.y * sinf(psi_d / 2)) / u_3;
    refQuaternion.q3 = sinf(psi_d / 2) * u_3 / 2;
    
    // Desired Angular Velocity
    Vector3Df refOmega;
    refOmega.x = -uup.x*sinf(psi_d)+uup.y*cosf(psi_d)+uup.z*(uu.x*sinf(psi_d)-uu.y*cosf(psi_d))/(1-uu.z);
    refOmega.y = -uup.x*cosf(psi_d)-uup.y*sinf(psi_d)+uup.z*(uu.x*cosf(psi_d)+uu.y*sinf(psi_d))/(1-uu.z);
    refOmega.z = psip_d-(-uu.x*uup.y+uu.y*uup.x)/(1-uu.z);
    
    // Store outputs
    output->GetMutex();
    output->SetValueNoMutex(0, 0, norm);
    output->SetValueNoMutex(1, 0, refQuaternion.q0);
    output->SetValueNoMutex(2, 0, refQuaternion.q1);
    output->SetValueNoMutex(3, 0, refQuaternion.q2);
    output->SetValueNoMutex(4, 0, refQuaternion.q3);
    output->SetValueNoMutex(5, 0, refOmega.x);
    output->SetValueNoMutex(6, 0, refOmega.y);
    output->SetValueNoMutex(7, 0, refOmega.z);
    output->ReleaseMutex();
    output->SetDataTime(GetTime());
    
    // Store state info
    state->GetMutex();
    state->SetValueNoMutex(0, 0, ui.x);
    state->SetValueNoMutex(1, 0, ui.y);
    state->SetValueNoMutex(2, 0, ui.z);
    state->SetValueNoMutex(3, 0, perror.x);
    state->SetValueNoMutex(4, 0, perror.y);
    state->SetValueNoMutex(5, 0, perror.z);
    state->SetDataTime(GetTime());
    state->ReleaseMutex();
    
    // Update load reference values
    loadRef->GetMutex();
    loadRef->SetValueNoMutex(0, 0, loadPosD.x);
    loadRef->SetValueNoMutex(1, 0, loadPosD.y);
    loadRef->SetValueNoMutex(2, 0, loadPosD.z);
    loadRef->SetValueNoMutex(3, 0, loadSpeedD.x);
    loadRef->SetValueNoMutex(4, 0, loadSpeedD.y);
    loadRef->SetDataTime(GetTime());
    loadRef->ReleaseMutex();
    
    // Update displayed angles
    Vector3Df alphaSelf = loadPos - uavPos;
    alphaSelf.Normalize();
    
    float phi = Euler::ToDegree(atan2f(-alphaSelf.y, -alphaSelf.x));
    float theta = Euler::ToDegree(atan2f(sqrtf(alphaSelf.x * alphaSelf.x + alphaSelf.y * alphaSelf.y), alphaSelf.z));
    
    angleRef->SetValue(0, 0, phi);
    angleRef->SetValue(1, 0, theta);
    
    // Set angle reference values for display
    if (uavIndex != 0) {
        float displayTheta = 2 * PI / allUavs.size() * uavIndex;
        if (displayTheta > PI) displayTheta -= 2 * PI;
        angleRef->SetValue(0, 1, Euler::ToDegree(theta));
        angleRef->SetValue(1, 1, params.thetaDesired);
    }
    
    // Update tension visualization
    UpdateTensionVisualization();
    
    // Process matrix update
    ProcessUpdate(output);
    
    // Update timestamp
    lastUpdateTimeMs = currentTime;
}

//------------------------------------------------------------------------------
// Get desired thrust
//------------------------------------------------------------------------------
float PositionControl::DesiredThrust(void) const {
    return output->Value(0, 0);
}

//------------------------------------------------------------------------------
// Get desired quaternion
//------------------------------------------------------------------------------
Quaternion PositionControl::DesiredQuaternion(void) const {
    Quaternion quat;
    output->GetMutex();
    quat.q0 = output->ValueNoMutex(1, 0);
    quat.q1 = output->ValueNoMutex(2, 0);
    quat.q2 = output->ValueNoMutex(3, 0);
    quat.q3 = output->ValueNoMutex(4, 0);
    output->ReleaseMutex();
    
    return quat;
}

//------------------------------------------------------------------------------
// Get desired omega
//------------------------------------------------------------------------------
Vector3Df PositionControl::DesiredOmega(void) const {
    Vector3Df omega;
    output->GetMutex();
    omega.x = output->ValueNoMutex(5, 0);
    omega.y = output->ValueNoMutex(6, 0);
    omega.z = output->ValueNoMutex(7, 0);
    output->ReleaseMutex();
    
    return omega;
}

//------------------------------------------------------------------------------
// Set reference mode
//------------------------------------------------------------------------------
void PositionControl::SetRefMode(BehaviourMode_t behaviourMode) {
    this->behaviourMode = behaviourMode;
}

//------------------------------------------------------------------------------
// Reset integral parts of PIDs
//------------------------------------------------------------------------------
void PositionControl::ResetI(void) {
    pidxUav->Reset();
    pidyUav->Reset();
    pidzUav->Reset();
    if (uavIndex == 0) {
        pidxLoad->Reset();
        pidyLoad->Reset();
        pidzLoad->Reset();
    }
}

//------------------------------------------------------------------------------
// Get current time in milliseconds
//------------------------------------------------------------------------------
uint64_t PositionControl::GetCurrentTimeMs() const {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
}

} // end namespace filter
} // end namespace flair