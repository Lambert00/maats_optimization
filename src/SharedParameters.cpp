// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2023/04/15
//  filename:   SharedParameters.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    5.0
//
//  purpose:    Class defining shared parameters
//
/*********************************************************************/

#include "SharedParameters.h"
#include <DoubleSpinBox.h>
#include <PushButton.h>
#include <Tab.h>
#include <Label.h>
#include <FrameworkManager.h>
#include <UdpSocket.h>
#include <Uav.h>
#include <sstream>
#include <string.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;

namespace flair {
namespace core {

SharedParameters::SharedParameters(const Object *parent, string name): 
    Object(parent, name), 
    serverAddress("172.26.213.70"),  // Optimization server address
    serverPort(DEFAULT_SERVER_PORT),  // Default TCP port (5555)
    resultPort(DEFAULT_RESULT_PORT),  // Default UDP port (5556)
    useOptimizedDistribution(false),  // Default to disabled
    areValid(false),
    sharedParamTab(nullptr),
    useOptimizer(nullptr),
    connectionStatus(nullptr)
{
    // Determine if this is the master drone (Drone_0)
    isMaster = (flair::meta::GetUav() && flair::meta::GetUav()->ObjectName() == "Drone_0");
    
    // Create UI tab
    sharedParamTab = new Tab(getFrameworkManager()->GetTabWidget(), "shared_param", 0);
    
    // Create physical parameter controls
    lCable = new DoubleSpinBox(sharedParamTab->NewRow(), "cable length", " m", 0, 5, 0.1, 1, 0);
    mLoad = new DoubleSpinBox(sharedParamTab->LastRowLastCol(), "load mass", " kg", 0, 5, 0.1, 2, 0);
    thetaDesired = new DoubleSpinBox(sharedParamTab->NewRow(), "theta", " deg", 0, 90, 1, 1, 0);
    xDesired = new DoubleSpinBox(sharedParamTab->LastRowLastCol(), "x load desired", " m", -5, 5, 0.1, 1, 0);
    yDesired = new DoubleSpinBox(sharedParamTab->LastRowLastCol(), "y load desired", " m", -5, 5, 0.1, 1, 0);
    zDesired = new DoubleSpinBox(sharedParamTab->LastRowLastCol(), "z load desired", " m", -5, 5, 0.1, 1, 0);
    mUav = new DoubleSpinBox(sharedParamTab->NewRow(), "uav mass", " kg", 0, 5, 0.1, 2, 0);
    gainCompensationZCharge = new DoubleSpinBox(sharedParamTab->LastRowLastCol(), "gainCompensationZCharge", -5, 5, 0.1, 2, 0);
    
    // Create UI for optimization control
    new Label(sharedParamTab->NewRow(), "OPTIMIZATION SERVER", true);
    
    // Add optimizer toggle button
    useOptimizer = new PushButton(sharedParamTab->NewRow(), "Enable Optimization");
    
    // Add status label
    connectionStatus = new Label(sharedParamTab->LastRowLastCol(), "Optimizer: Disabled");
    
    // Initialize parameters from UI
    CopyFromUI();
    
    // Ensure optimization is initially disabled
    useOptimizedDistribution.store(false);
    if (connectionStatus) {
        connectionStatus->SetText("Optimizer: Disabled");
    }
    
    // Set parameters as valid
    areValid = true;
    
    // Log initialization
    Info("SharedParameters initialized. Master status: %s, Optimization: DISABLED", isMaster ? "true" : "false");
}

SharedParameters::~SharedParameters() {
    // Nothing to clean up - UI elements are handled by framework
}

bool SharedParameters::AreValid(void) const {
    return areValid;
}

Tab* SharedParameters::GetTab(void) const {
    return sharedParamTab;
}

ssize_t SharedParameters::GetSize(void) const {
    return sizeof(datas_t);
}

SharedParameters::datas_t SharedParameters::GetDatas(void) const {
    return sharedParameters;
}

std::string SharedParameters::GetServerAddress() const {
    return serverAddress;
}

void SharedParameters::SetServerAddress(const std::string& address) {
    serverAddress = address;
}

int SharedParameters::GetServerPort() const {
    return serverPort;
}

void SharedParameters::SetServerPort(int port) {
    serverPort = port;
}

int SharedParameters::GetResultPort() const {
    return resultPort;
}

void SharedParameters::SetResultPort(int port) {
    resultPort = port;
}

bool SharedParameters::IsMaster() const {
    return isMaster;
}

bool SharedParameters::GetUseOptimizedDistribution() const {
    return useOptimizedDistribution.load();
}

void SharedParameters::EnableOptimizer(bool enable, UdpSocket* message) {
    // Store previous state to detect change
    bool previousState = useOptimizedDistribution.load();
    
    // Update optimization state
    useOptimizedDistribution.store(enable);
    
    // Update UI
    if (connectionStatus) {
        connectionStatus->SetText(enable ? "Optimizer: Enabled" : "Optimizer: Disabled");
    }
    
    // Log status change with high visibility
    if (previousState != enable) {
        Info("============================================");
        Info("%s: Optimization %s", ObjectName().c_str(), 
             enable ? "ENABLED" : "DISABLED");
        Info("============================================");
        
        // Broadcast the change to all drones if a message socket was provided
        if (message) {
            BroadcastOptimizerState(message);
        }
    } else {
        // Log even if state didn't change
        Info("%s: Optimization remains %s", ObjectName().c_str(), 
             enable ? "ENABLED" : "DISABLED");
    }
}

void SharedParameters::BroadcastOptimizerState(UdpSocket* message) {
    if (!message) {
        Warn("Cannot broadcast optimizer state: no message socket provided");
        return;
    }
    
    // Create a compact message format
    bool isEnabled = useOptimizedDistribution.load();
    std::string optimizerMsg = "OptimizerState:" + std::string(isEnabled ? "1" : "0");
    
    // Send the message
    try {
        message->SendMessage(optimizerMsg.c_str());
        Info("Broadcast optimizer state: %s", isEnabled ? "ENABLED" : "DISABLED");
    } catch (const std::exception& e) {
        Err("Failed to broadcast optimizer state: %s", e.what());
    }
}

void SharedParameters::UpdateOptimizerStatus(bool connected) {
    // Update UI status
    if (connectionStatus) {
        if (connected) {
            if (useOptimizedDistribution.load()) {
                connectionStatus->SetText("Optimizer: Connected (Active)");
            } else {
                connectionStatus->SetText("Optimizer: Connected (Inactive)");
            }
        } else {
            if (useOptimizedDistribution.load()) {
                connectionStatus->SetText("Optimizer: Connection Failed");
            } else {
                connectionStatus->SetText("Optimizer: Disabled");
            }
        }
    }
}

void SharedParameters::CheckButtons(UdpSocket* message) {
    // Check for optimizer toggle
    if (useOptimizer && useOptimizer->Clicked()) {
        // Toggle the optimizer state
        bool currentState = useOptimizedDistribution.load();
        EnableOptimizer(!currentState, message);
        
        // Log the action with high visibility
        Info("-----------------------------------------------");
        Info("User clicked optimizer toggle button: %s", 
             useOptimizedDistribution.load() ? "ENABLED" : "DISABLED");
        Info("-----------------------------------------------");
    }
}

void SharedParameters::Send(UdpSocket *message) {
    if (!isMaster) {
        Warn("Send not called from master, leaving!\n");
        return;
    }
    
    // Update local data from UI before sending
    CopyFromUI();
    
    // Send parameters to other drones
    Info("Sending shared parameters to other drones");
    message->SendMessage((char*)&sharedParameters, sizeof(datas_t));
    
    // Also send the current optimizer state separately
    // This ensures all drones have the same optimization setting
    BroadcastOptimizerState(message);
}

void SharedParameters::CopyFromBuffer(char* buffer) {
    // Store previous data
    datas_t prevParams = sharedParameters;
    
    // Copy data from buffer
    memcpy(&sharedParameters, buffer, sizeof(datas_t));
    
    // Mark as valid
    areValid = true;
    
    // Log parameter changes for debugging
    if (prevParams.lCable != sharedParameters.lCable ||
        prevParams.mLoad != sharedParameters.mLoad ||
        prevParams.mUav != sharedParameters.mUav ||
        prevParams.thetaDesired != sharedParameters.thetaDesired ||
        prevParams.xDesired != sharedParameters.xDesired ||
        prevParams.yDesired != sharedParameters.yDesired ||
        prevParams.zDesired != sharedParameters.zDesired) {
        
        Info("Received updated shared parameters");
    }
}

void SharedParameters::CopyFromUI(void) {
    // Update shared parameters from UI
    sharedParameters.lCable = lCable->Value();
    sharedParameters.mLoad = mLoad->Value();
    sharedParameters.mUav = mUav->Value();
    sharedParameters.thetaDesired = thetaDesired->Value();
    sharedParameters.xDesired = xDesired->Value();
    sharedParameters.yDesired = yDesired->Value();
    sharedParameters.zDesired = zDesired->Value();
    sharedParameters.gainCompensationZCharge = gainCompensationZCharge->Value();
}

} // end namespace core
} // end namespace flair