//  created:    2023/04/15
//  filename:   ChargeSuspendue.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    5.0
//
//  purpose:    Multi-drone cooperative transport with server optimization
//
//
/*********************************************************************/

#include "ChargeSuspendue.h"

#include <Uav.h>
#include <FrameworkManager.h>
#include <TargetController.h>
#include <UdpSocket.h>
#include <Vector3D.h>
#include <Vector2D.h>
#include <Euler.h>
#include <Quaternion.h>
#include <Matrix.h>
#include <AhrsData.h>

#include <GridLayout.h>
#include <TabWidget.h>
#include <Tab.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <Label.h>

#include <Ahrs.h>
#include <MetaUsRangeFinder.h>
#include <MetaDualShock3.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>

#include <Pid.h>
#include <PidThrust.h>

#include <TrajectoryGenerator2DCircle.h>

#include "PositionControl.h"
#include "AttitudeControl.h"
#include "SharedParameters.h"

#include <math.h>
#include <string.h>
#include <sstream>
#include <chrono>

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;

#define G 9.81

// Helper function to get current time in milliseconds
uint64_t ChargeSuspendue::GetCurrentTimeMs() const {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

ChargeSuspendue::ChargeSuspendue(string broadcast, TargetController *controller, uint16_t uavNumber): 
    UavStateMachine(controller), 
    behaviourMode(BehaviourMode_t::Default), 
    vrpnLost(false),
    serverConnected(false),
    lastServerCheckTime(0),
    lastOptimizerSyncTime(0)
{
    Uav* uav = GetUav();

    // Set up VRPN client
    VrpnClient* vrpnclient = new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(), 80, uav->GetDefaultVrpnConnectionType());
    if (vrpnclient->ConnectionType() != VrpnClient::Vrpn) {
        Err("vrpn connection type is not handled in the code\n");
    }
    
    // Set up load VRPN object
    chargeVrpn = new MetaVrpnObject("charge");
    getFrameworkManager()->AddDeviceToLog(chargeVrpn);
    
    // Set up all UAV VRPN objects
    for(uint16_t i = 0; i < uavNumber; i++) {
        stringstream s;
        s << "Drone_" << i;
        MetaVrpnObject* uavVrpn = new MetaVrpnObject(s.str());
        allUavs.push_back(uavVrpn);
        getFrameworkManager()->AddDeviceToLog(uavVrpn);
        if(s.str() == uav->ObjectName()){ 
           uavIndex = i;
        } 
    }

    // Configure plots
    uav->GetAhrs()->YawPlot()->AddCurve(allUavs[uavIndex]->State()->Element(2), DataPlot::Green);
    
    // Create trajectory generator
    circle = new TrajectoryGenerator2DCircle(vrpnclient->GetLayout()->NewRow(), "charge_ref");
    
    // Create shared parameters
    sharedParameters = new SharedParameters(this, "shared_params");
    
    // Create UI buttons - only for master
    if (IsMaster()) {
        sendSharedParameters = new PushButton(sharedParameters->GetTab()->NewRow(), "send shared parameters");
        positionHold = new PushButton(sharedParameters->GetTab()->NewRow(), "position hold");
        gotoPosition = new PushButton(sharedParameters->GetTab()->NewRow(), "goto position");
        startCircle = new PushButton(sharedParameters->GetTab()->NewRow(), "start circle");
        stopCircle = new PushButton(sharedParameters->GetTab()->LastRowLastCol(), "stop_circle");
        
        // Create server status UI
        serverStatusLabel = new Label(sharedParameters->GetTab()->NewRow(), "Server: Not connected");
    } else {
        // Initialize pointers to nullptr for non-master drones
        sendSharedParameters = nullptr;
        positionHold = nullptr;
        gotoPosition = nullptr;
        startCircle = nullptr;
        stopCircle = nullptr;
        serverStatusLabel = nullptr;
    }
    
    if(uav->GetType() == "mamboedu") {
        SetFailSafeAltitudeSensor(allUavs[uavIndex]->GetAltitudeSensor());
    }
    
    // Create PIDs for position hold
    u_x = new Pid(setupLawTab->At(1, 0), "u_x");
    u_x->UseDefaultPlot(graphLawTab->NewRow());
    u_y = new Pid(setupLawTab->At(1, 1), "u_y");
    u_y->UseDefaultPlot(graphLawTab->LastRowLastCol());

    // Create position controller
    positionControl = new PositionControl("custom_law", sharedParameters, uavIndex, allUavs, chargeVrpn, circle);
    getFrameworkManager()->AddDeviceToLog(positionControl);
    
    // Create message socket
    message = new UdpSocket(uav, "Message", broadcast, true);

    // Create orientation data
    customReferenceOrientation = new AhrsData(this, "reference");
    uav->GetAhrs()->AddPlot(customReferenceOrientation, DataPlot::Yellow);
    customOrientation = new AhrsData(this, "orientation");
    
    // Create attitude controller
    attitudeControl = new AttitudeControl(setupLawTab->At(2, 0), "law");
    attitudeControl->UseDefaultPlot(graphLawTab->NewRow());
    getFrameworkManager()->AddDeviceToLog(attitudeControl);
    attitudeControl->AddDataToLog(customReferenceOrientation);
    attitudeControl->AddDataToLog(customOrientation);
   
    // Set up plots
    allUavs[uavIndex]->XyPlot()->AddCurve(chargeVrpn->NEDPosition::Output()->Element(1), chargeVrpn->NEDPosition::Output()->Element(0), DataPlot::Yellow, "charge");
    allUavs[uavIndex]->XyPlot()->AddCurve(circle->GetMatrix()->Element(0, 1), circle->GetMatrix()->Element(0, 0), DataPlot::Green, "charge_ref");
    
    // Add other UAVs to plot
    for (size_t i = 0; i < allUavs.size(); i++) {
        if(i == uavIndex) continue;
        allUavs[uavIndex]->XyPlot()->AddCurve(allUavs.at(i)->NEDPosition::Output()->Element(1), allUavs.at(i)->NEDPosition::Output()->Element(0), DataPlot::Black, "other_uav");
    }
    
    // Start VRPN client
    vrpnclient->Start();
    
    // Ensure optimizer mode is disabled by default for master
    if (IsMaster() && sharedParameters) {
        sharedParameters->EnableOptimizer(false);
    }
}

ChargeSuspendue::~ChargeSuspendue() {
    // Log that destructor is called
    Thread::Info("ChargeSuspendue destructor called - starting cleanup\n");
    
    // Stop checking server before any cleanup
    serverConnected.store(false);
    
    // First clean up position control to ensure server connections are closed
    if (positionControl) {
        Thread::Info("Cleaning up position control\n");
        // Allow position control to safely clean up its connections
        delete positionControl;
        positionControl = nullptr;
    }
    
    // Clean up UDP message socket
    if (message) {
        Thread::Info("Cleaning up UDP socket\n");
        delete message;
        message = nullptr;
    }
    
    // Clean up attitude control
    if (attitudeControl) {
        Thread::Info("Cleaning up attitude control\n");
        delete attitudeControl;
        attitudeControl = nullptr;
    }
    
    // Clean up shared parameters last, as other components might need it during cleanup
    if (sharedParameters) {
        Thread::Info("Cleaning up shared parameters\n");
        delete sharedParameters;
        sharedParameters = nullptr;
    }
    
    Thread::Info("ChargeSuspendue cleanup complete\n");
}

bool ChargeSuspendue::IsMaster(void) const {
    return (GetUav()->ObjectName() == "Drone_0");
}

const AhrsData *ChargeSuspendue::GetOrientation(void) const {
    // Get yaw from VRPN
    Quaternion vrpnQuaternion;
    allUavs[uavIndex]->GetQuaternion(vrpnQuaternion);

    // Get roll, pitch and angular velocity from IMU
    Quaternion ahrsQuaternion;
    Vector3Df ahrsAngularSpeed;
    GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion, ahrsAngularSpeed);

    // Combine IMU and VRPN data
    Euler ahrsEuler = ahrsQuaternion.ToEuler();
    ahrsEuler.yaw = vrpnQuaternion.ToEuler().yaw;
    Quaternion mixQuaternion = ahrsEuler.ToQuaternion();

    // Update custom orientation
    customOrientation->SetQuaternionAndAngularRates(mixQuaternion, ahrsAngularSpeed);

    return customOrientation;
}

void ChargeSuspendue::AltitudeValues(float &z, float &dz) const {
    Vector3Df uav_pos, uav_vel;

    allUavs[uavIndex]->GetPosition(uav_pos);
    allUavs[uavIndex]->GetSpeed(uav_vel);
    
    // z and dz must be in UAV's frame
    z = -uav_pos.z;
    dz = -uav_vel.z;
}

float ChargeSuspendue::ComputeCustomThrust(void) {
    SharedParameters::datas_t params = sharedParameters->GetDatas();
    return -positionControl->DesiredThrust() * params.gainCompensationZCharge;
}

void ChargeSuspendue::ComputeCustomTorques(Euler &torques) {
    // Update circle trajectory if in circle mode
    if (behaviourMode == BehaviourMode_t::CircleTrajectory) {
        circle->Update(GetTime());
    }
    
    // Get desired orientation from position controller
    Quaternion refQuaternion = positionControl->DesiredQuaternion();
    Vector3Df referenceOmega = positionControl->DesiredOmega();
 
    // Set reference orientation
    customReferenceOrientation->SetQuaternionAndAngularRates(refQuaternion, referenceOmega);

    // Update attitude controller
    attitudeControl->SetValues(customOrientation, customReferenceOrientation);
    attitudeControl->Update(GetTime());

    // Get torques from attitude controller
    torques.roll = attitudeControl->Output(0);
    torques.pitch = attitudeControl->Output(1);
    torques.yaw = attitudeControl->Output(2);
}

const AhrsData *ChargeSuspendue::GetReferenceOrientation(void) {
    // Create reference angles with desired yaw
    Euler refAngles;
    refAngles.yaw = yawHold;
    
    // Get position error
    Vector3Df pos_err; // in UAV coordinate system
    Vector2Df vel_err; // in UAV coordinate system
    PositionValues(pos_err, vel_err);
    
    // Update PIDs
    u_x->SetValues(pos_err.x, vel_err.x);
    u_x->Update(GetTime());
    u_y->SetValues(pos_err.y, vel_err.y);
    u_y->Update(GetTime());
    
    // Set reference angles from PID outputs
    refAngles.pitch = u_x->Output();
    refAngles.roll = -u_y->Output();
    
    // Update reference orientation
    customReferenceOrientation->SetQuaternionAndAngularRates(refAngles.ToQuaternion(), Vector3Df(0, 0, 0));

    return customReferenceOrientation;
}

void ChargeSuspendue::PositionValues(Vector3Df &pos_error, Vector2Df &vel_error) {
    Vector3Df uav_pos, uav_vel; // in VRPN coordinate system
    Vector2Df uav_2Dvel; // in VRPN coordinate system

    // Get current position and velocity
    allUavs[uavIndex]->GetPosition(uav_pos);
    allUavs[uavIndex]->GetSpeed(uav_vel);
    uav_vel.To2Dxy(uav_2Dvel);

    // Calculate error
    pos_error = uav_pos - posHold;
    vel_error = uav_2Dvel;

    // Transform error to UAV frame
    Quaternion currentQuaternion = GetCurrentQuaternion();
    Euler currentAngles; // in VRPN frame
    currentQuaternion.ToEuler(currentAngles);
    pos_error.RotateZ(-currentAngles.yaw);
    vel_error.Rotate(-currentAngles.yaw);
}

void ChargeSuspendue::CheckOptimizerConnection(void) {
    if (!sharedParameters || !positionControl) {
        return;
    }
    
    // Only check occasionally to avoid excessive checking
    uint64_t currentTime = GetCurrentTimeMs();
    
    // Adaptive check interval
    uint64_t checkInterval = serverConnected.load() ? 10000 : 5000;
    
    if (currentTime - lastServerCheckTime < checkInterval) {
        return; 
    }
    
    lastServerCheckTime = currentTime;
    
    // Check optimizer connection status
    if (IsMaster() && positionControl && sharedParameters) {
        bool isOptimizerEnabled = sharedParameters->GetUseOptimizedDistribution();
        bool prevConnected = serverConnected.load();
        
        // Default to not connected
        bool connected = false;
        
        // Only test connection if optimization is enabled, and don't block flight operations
        if (isOptimizerEnabled) {
            try {
                // Very short timeout to ensure we don't block for too long
                connected = positionControl->TestServerConnection(50);
            } catch (const std::exception& e) {
                Thread::Err("Error checking server connection: %s\n", e.what());
                connected = false;
            }
            
            // Update stored state
            serverConnected.store(connected);
            
            // Update UI
            if (serverStatusLabel) {
                std::string status = connected ? "Connected" : "Disconnected";
                serverStatusLabel->SetText(("Server: " + status).c_str());
            }
            
            // Update SharedParameters connection status
            if (sharedParameters) {
                sharedParameters->UpdateOptimizerStatus(connected);
            }
            
            // If server is not connected and optimizer is enabled, automatically
            // switch to fixed distribution mode to prevent control issues
            if (!connected && isOptimizerEnabled) {
                // Disable optimizer since the server is not available
                Thread::Info("Server not connected, switching to fixed distribution mode\n");
                
                if (sharedParameters) {
                    sharedParameters->EnableOptimizer(false);
                }
                
                // Update position control mode
                if (positionControl) {
                    positionControl->SetTensionDistributionMode(
                        PositionControl::TensionDistributionMode_t::FixedDistribution);
                }
            }
            
            // Log connection state changes
            if (prevConnected && !connected) {
                Thread::Info("Server connection lost, using fixed distribution mode\n");
            } else if (!prevConnected && connected) {
                Thread::Info("Server connection established\n");
            }
        }
        
        // Periodically sync optimizer state with all drones (every 30 seconds)
        if (currentTime - lastOptimizerSyncTime > 30000) {
            lastOptimizerSyncTime = currentTime;
            
            // Send current optimizer state to all drones, but only if message system is available
            if (sharedParameters) {
                try {
                    std::string stateMsg = "OptimizerState:" + std::string(isOptimizerEnabled ? "1" : "0");
                    SendCommand(stateMsg);
                } catch (...) {
                    // Ignore errors during sync
                }
            }
        }
    }
}

bool ChargeSuspendue::SendCommand(const std::string& command) {
    if (!message) {
        return false;
    }
    
    try {
        message->SendMessage(command);
        return true;
    }
    catch (const std::exception& e) {
        Err("Failed to send command: %s", e.what());
        return false;
    }
}

void ChargeSuspendue::SignalEvent(Event_t event) {
    // Call parent implementation
    UavStateMachine::SignalEvent(event);

    switch(event) {
    case Event_t::Stabilized:
        break;
        
    case Event_t::EmergencyStop:
        // Send emergency stop message
        SendCommand("EmergencyStop");
        break;
        
    case Event_t::TakingOff:
        // Send take off message
        SendCommand("TakeOff");
        
        // Master sends parameters
        if (IsMaster()) {
            // Send shared parameters to all drones
            sharedParameters->Send(message);
            
            // After sending parameters, also send optimizer state explicitly
            char optimizerMsg[32];
            bool isEnabled = sharedParameters->GetUseOptimizedDistribution();
            snprintf(optimizerMsg, sizeof(optimizerMsg), "OptimizerState:%d", isEnabled ? 1 : 0);
            SendCommand(optimizerMsg);
        }
        
        // Enter position hold
        PositionHold();
        break;
        
    case Event_t::StartLanding:
        // Enter position hold
        PositionHold();
        
        // Send landing message
        SendCommand("Landing");
        break;
        
    case Event_t::EnteringControlLoop:
        // Check messages
        CheckMessages();
        
        // Update position control
        positionControl->Update();
        
        // Check server connection
        CheckOptimizerConnection();
        break;
        
    case Event_t::EnteringFailSafeMode:
        // Reset behavior mode
        behaviourMode = BehaviourMode_t::Default;
        break;
        
    case Event_t::StartLog:
    case Event_t::StopLog:
        // No special action needed
        break;
    }
}

void ChargeSuspendue::CheckMessages(void) {
    char msg[1024]; // Buffer for messages
    char src[64];
    
    // Check buffer size
    if (sizeof(msg) < sharedParameters->GetSize()) {
        Err("CheckMessages buf size is too small for SharedParameters_t\n");
    }
    
    size_t src_size = sizeof(src);
    ssize_t rcv_size;
    
    // Process all available messages
    while (true) {
        rcv_size = message->RecvMessage(msg, sizeof(msg), TIME_NONBLOCK, src, &src_size);
        if (rcv_size <= 0) break;
        
        // Skip own messages
        if (strcmp(src, GetUav()->ObjectName().c_str()) == 0) {
            continue;
        }
        
        // Process commands
        if (strcmp(msg, "TakeOff") == 0) {
            TakeOff();
        } 
        else if (strcmp(msg, "Landing") == 0) {
            Land();
        } 
        else if (strcmp(msg, "Goto") == 0) {
            GotoPosition();
        } 
        else if (strcmp(msg, "StartCircle") == 0) {
            StartCircle();
        } 
        else if (strcmp(msg, "StopCircle") == 0) {
            StopCircle();
        } 
        else if (strcmp(msg, "Hold") == 0) {
            PositionHold();
        } 
        else if (strcmp(msg, "EmergencyStop") == 0) {
            EmergencyStop();
        } 
        else if (strcmp(msg, "StartLog") == 0) {
            getFrameworkManager()->StartLog();
        } 
        else if (strcmp(msg, "StopLog") == 0) {
            getFrameworkManager()->StopLog();
        } 
        else if (strcmp(msg, "EnableOptimizer") == 0) {
            if (sharedParameters) {
                sharedParameters->EnableOptimizer(true);
                
                // Update tension distribution mode in position control
                if (positionControl) {
                    positionControl->SetTensionDistributionMode(
                        PositionControl::TensionDistributionMode_t::OptimizedDistribution);
                }
            }
        } 
        else if (strcmp(msg, "DisableOptimizer") == 0) {
            if (sharedParameters) {
                sharedParameters->EnableOptimizer(false);
                
                // Update tension distribution mode in position control
                if (positionControl) {
                    positionControl->SetTensionDistributionMode(
                        PositionControl::TensionDistributionMode_t::FixedDistribution);
                }
            }
        }
        else if (strncmp(msg, "OptimizerState:", 15) == 0) {
            // Parse the optimizer state message
            bool newState = (msg[15] == '1');
            
            // Update local state
            if (sharedParameters) {
                sharedParameters->EnableOptimizer(newState);
            }
            
            // Update tension distribution mode in position control
            if (positionControl) {
                auto newMode = newState ? 
                    PositionControl::TensionDistributionMode_t::OptimizedDistribution : 
                    PositionControl::TensionDistributionMode_t::FixedDistribution;
                
                positionControl->SetTensionDistributionMode(newMode);
            }
        }
        // Process shared parameters
        else if (rcv_size == sharedParameters->GetSize()) {
            sharedParameters->CopyFromBuffer(msg);
        }
    }
}

void ChargeSuspendue::ExtraSecurityCheck(void) {
    // Check if VRPN tracking is lost
    if (!vrpnLost && behaviourMode != BehaviourMode_t::Default) {
        // Check UAV tracking
        for (size_t i = 0; i < allUavs.size(); i++) {
            if (!allUavs[i]->IsTracked(500)) {
                Thread::Err("Optitrack, UAV lost\n");
                vrpnLost = true;
                EnterFailSafeMode();
                Land();
            }
        }
        
        // Check load tracking
        if (!chargeVrpn->IsTracked(500)) {
            Thread::Err("Optitrack, load lost\n");
            vrpnLost = true;
            EnterFailSafeMode();
            Land();
        }
    }
}

void ChargeSuspendue::ExtraCheckJoystick(void) {
    // R1 and Circle - Start circle
    if (GetTargetController()->ButtonClicked(4) && GetTargetController()->IsButtonPressed(9)) {
        StartCircle();
        SendCommand("StartCircle");
    }

    // R1 and Cross - Stop circle
    if (GetTargetController()->ButtonClicked(5) && GetTargetController()->IsButtonPressed(9)) {
        StopCircle();
        SendCommand("StopCircle");
    }
    
    // R1 and Square - Position hold
    if (GetTargetController()->ButtonClicked(2) && GetTargetController()->IsButtonPressed(9)) {
        PositionHold();
        SendCommand("Hold");
    }
    
    // L2 - Start logging
    if (GetTargetController()->ButtonClicked(7)) {
        getFrameworkManager()->StartLog();
        SendCommand("StartLog");
    }
    
    // R2 - Stop logging
    if (GetTargetController()->ButtonClicked(10)) {
        getFrameworkManager()->StopLog();
        SendCommand("StopLog");
    }
    
    // L1 and Triangle - Toggle optimizer
    if (GetTargetController()->ButtonClicked(3) && GetTargetController()->IsButtonPressed(8)) {
        if (IsMaster() && sharedParameters) {
            // Get current state
            bool currentState = sharedParameters->GetUseOptimizedDistribution();
            
            // Toggle state
            bool newState = !currentState;
            sharedParameters->EnableOptimizer(newState);
            
            // Send toggle command to all drones
            SendCommand(newState ? "EnableOptimizer" : "DisableOptimizer");
            
            // Also send explicit state message
            std::string stateMsg = "OptimizerState:" + std::string(newState ? "1" : "0");
            SendCommand(stateMsg);
            
            // Update position control mode
            if (positionControl) {
                positionControl->SetTensionDistributionMode(newState ?
                    PositionControl::TensionDistributionMode_t::OptimizedDistribution :
                    PositionControl::TensionDistributionMode_t::FixedDistribution);
            }
        }
    }
}

void ChargeSuspendue::ExtraCheckPushButton(void) {
    // Only the master has buttons
    if (!IsMaster()) {
        return;
    }
    
    // Go to position
    if (gotoPosition && gotoPosition->Clicked()) {
        GotoPosition();
        SendCommand("Goto");
    }
    
    // Start circle
    if (startCircle && startCircle->Clicked()) {
        StartCircle();
        SendCommand("StartCircle");
    }
    
    // Stop circle
    if (stopCircle && stopCircle->Clicked()) {
        StopCircle();
        SendCommand("StopCircle");
    }
    
    // Position hold
    if (positionHold && positionHold->Clicked() && (behaviourMode != BehaviourMode_t::PositionHold)) {
        PositionHold();
        SendCommand("Hold");
    }
    
    // Send shared parameters
    if (sendSharedParameters && sendSharedParameters->Clicked()) {
        sharedParameters->Send(message);
        
        // After sending parameters, also send optimizer state explicitly
        char optimizerMsg[32];
        bool isEnabled = sharedParameters->GetUseOptimizedDistribution();
        snprintf(optimizerMsg, sizeof(optimizerMsg), "OptimizerState:%d", isEnabled ? 1 : 0);
        SendCommand(optimizerMsg);
    }
    
    // Check buttons in shared parameters (for optimizer toggle)
    if (sharedParameters) {
        bool oldState = sharedParameters->GetUseOptimizedDistribution();
        
        // Check buttons (may toggle optimizer state)
        sharedParameters->CheckButtons();
        
        // If state changed, broadcast it
        bool newState = sharedParameters->GetUseOptimizedDistribution();
        if (oldState != newState) {
            // Send toggle command
            SendCommand(newState ? "EnableOptimizer" : "DisableOptimizer");
            
            // Also send explicit state message (more reliable)
            std::string stateMsg = "OptimizerState:" + std::string(newState ? "1" : "0");
            SendCommand(stateMsg);
            
            // Update position control mode
            if (positionControl) {
                positionControl->SetTensionDistributionMode(newState ?
                    PositionControl::TensionDistributionMode_t::OptimizedDistribution :
                    PositionControl::TensionDistributionMode_t::FixedDistribution);
            }
        }
    }
}

void ChargeSuspendue::PositionHold(void) {
    // Set custom orientation mode
    if (SetOrientationMode(OrientationMode_t::Custom)) {
        Thread::Info("Holding position\n");
    } else {
        Thread::Warn("Could not hold position, error SetOrientationMode(OrientationMode_t::Custom) failed\n");
        return;
    }
    
    // Set behavior mode
    behaviourMode = BehaviourMode_t::PositionHold;
    
    // Get current orientation
    Quaternion vrpnQuaternion;
    allUavs[uavIndex]->GetQuaternion(vrpnQuaternion);
    yawHold = vrpnQuaternion.ToEuler().yaw;

    // Get current position
    allUavs[uavIndex]->GetPosition(posHold);
    
    // Reset PIDs
    u_x->Reset();
    u_y->Reset();
}

void ChargeSuspendue::StartCircle(void) {
    // Send shared parameters if master
    if (IsMaster()) {
        sharedParameters->Send(message);
        
        // Also send optimizer state
        std::string stateMsg = "OptimizerState:" + 
            std::string(sharedParameters->GetUseOptimizedDistribution() ? "1" : "0");
        SendCommand(stateMsg);
    }
    
    // Check if parameters are valid
    if (!sharedParameters->AreValid()) {
        Thread::Warn("Parameters never received! Going to position hold\n");
        PositionHold();
        SendCommand("Hold");
        return;
    }
    
    // Set custom torque mode
    if (!SetTorqueMode(TorqueMode_t::Custom)) {
        Thread::Warn("Could not StartCircle, error SetTorqueMode(TorqueMode_t::Custom) failed\n");
        return;
    }
    
    // Set custom thrust mode
    if (!SetThrustMode(ThrustMode_t::Custom)) {
        Thread::Warn("Could not goto position, error SetThrustMode(ThrustMode_t::Custom) failed\n");
        return;
    }
    
    Thread::Info("StartCircle\n");
    behaviourMode = BehaviourMode_t::CircleTrajectory;

    // Get current orientation
    Quaternion vrpnQuaternion;
    allUavs[uavIndex]->GetQuaternion(vrpnQuaternion);
    yawHold = vrpnQuaternion.ToEuler().yaw;
    
    // Set circle trajectory mode
    positionControl->SetRefMode(PositionControl::BehaviourMode_t::CircleTrajectory);
    positionControl->ResetI();
    
    // Get current load position
    Vector3Df charge_pos;
    Vector2Df charge_2Dpos;
    chargeVrpn->GetPosition(charge_pos);
    charge_pos.To2Dxy(charge_2Dpos);
    
    // Set circle center and start trajectory
    circle->SetCenter(charge_2Dpos - Vector2Df(circle->GetRadius(), 0));
    circle->StartTraj(charge_2Dpos);
}

void ChargeSuspendue::StopCircle(void) {
    // Check if in circle mode
    if (behaviourMode != BehaviourMode_t::CircleTrajectory) {
        Thread::Warn("Not in circle mode\n");
        return;
    }
    
    // Finish trajectory
    circle->FinishTraj();
    Thread::Info("Finishing circle\n");
}

void ChargeSuspendue::GotoPosition(void) {
    // Send shared parameters if master
    if (IsMaster()) {
        sharedParameters->Send(message);
        
        // Also send optimizer state
        std::string stateMsg = "OptimizerState:" + 
            std::string(sharedParameters->GetUseOptimizedDistribution() ? "1" : "0");
        SendCommand(stateMsg);
    }
    
    // Check if parameters are valid
    if (!sharedParameters->AreValid()) {
        Thread::Warn("Parameters never received! Going to position hold\n");
        PositionHold();
        SendCommand("Hold");
        return;
    }
    
    // Set custom torque mode
    if (!SetTorqueMode(TorqueMode_t::Custom)) {
        Thread::Warn("Could not goto position, error SetTorqueMode(TorqueMode_t::Custom) failed\n");
        return;
    }
    
    // Set custom thrust mode
    if (!SetThrustMode(ThrustMode_t::Custom)) {
        Thread::Warn("Could not goto position, error SetThrustMode(ThrustMode_t::Custom) failed\n");
        return;
    }
    
    Thread::Info("Going to position\n");
    behaviourMode = BehaviourMode_t::GoToPosition;
    
    // Set position control mode
    positionControl->SetRefMode(PositionControl::BehaviourMode_t::GoToPosition);
    positionControl->ResetI();
    
    // Get current orientation
    Quaternion vrpnQuaternion;
    allUavs[uavIndex]->GetQuaternion(vrpnQuaternion);
    yawHold = vrpnQuaternion.ToEuler().yaw;
}