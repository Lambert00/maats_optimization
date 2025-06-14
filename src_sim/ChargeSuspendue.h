//  created:    2023/04/15
//  filename:   ChargeSuspendue.h
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

#ifndef CHARGESUSPENDUE_H
#define CHARGESUSPENDUE_H

#include <UavStateMachine.h>
#include <atomic>

namespace flair {
    namespace core {
        class UdpSocket;
        class AhrsData;
        class SharedParameters;
    }
    namespace meta {
        class MetaVrpnObject;
    }
    namespace gui {
        class PushButton;
        class Label;
    }
    namespace filter {
        class PositionControl;
        class AttitudeControl;
        class TrajectoryGenerator2DCircle;
    }
}

/**
 * @brief Main controller for multi-drone cooperative transport
 */
class ChargeSuspendue : public flair::meta::UavStateMachine {
    public:
        /**
         * @brief Constructor
         *
         * @param broadcast Broadcast address
         * @param controller Target controller
         * @param uavNumber Number of UAVs in the system
         */
        ChargeSuspendue(std::string broadcast, flair::sensor::TargetController *controller, uint16_t uavNumber);
        
        /**
         * @brief Destructor
         */
        ~ChargeSuspendue();

    private:
        /**
         * @brief Behavior modes for the system
         */
        enum class BehaviourMode_t {
            Default,            ///< Default mode
            GoToPosition,       ///< Go to a specific position
            CircleTrajectory,   ///< Follow a circular trajectory
            PositionHold,       ///< Hold current position
        };
        
        /**
         * @brief Check if this is the master drone
         *
         * @return true if master
         */
        bool IsMaster(void) const;
        
        /**
         * @brief Get current orientation
         *
         * @return Orientation data
         */
        const flair::core::AhrsData *GetOrientation(void) const;
        
        /**
         * @brief Get altitude values
         *
         * @param z Output altitude
         * @param dz Output altitude rate
         */
        void AltitudeValues(float &z, float &dz) const;
        
        /**
         * @brief Get position values for position hold
         *
         * @param pos_error Output position error
         * @param vel_error Output velocity error
         */
        void PositionValues(flair::core::Vector3Df &pos_error, flair::core::Vector2Df &vel_error);
        
        /**
         * @brief Get reference orientation for position hold
         *
         * @return Reference orientation
         */
        const flair::core::AhrsData *GetReferenceOrientation(void);
        
        /**
         * @brief Handle system events
         *
         * @param event Event to handle
         */
        void SignalEvent(Event_t event);
        
        /**
         * @brief Check for messages from other drones
         */
        void CheckMessages(void);
        
        /**
         * @brief Compute custom thrust
         *
         * @return Thrust value
         */
        float ComputeCustomThrust(void);
        
        /**
         * @brief Compute custom torques
         *
         * @param torques Output torques
         */
        void ComputeCustomTorques(flair::core::Euler &torques);
        
        /**
         * @brief Extra security checks
         */
        void ExtraSecurityCheck(void);
        
        /**
         * @brief Check joystick input
         */
        void ExtraCheckJoystick(void);
        
        /**
         * @brief Check button input
         */
        void ExtraCheckPushButton(void);
        
        /**
         * @brief Hold position
         */
        void PositionHold(void);
        
        /**
         * @brief Start circular trajectory
         */
        void StartCircle(void);
        
        /**
         * @brief Stop circular trajectory
         */
        void StopCircle(void);
        
        /**
         * @brief Go to position
         */
        void GotoPosition(void);
        
        /**
         * @brief Check server connection
         */
        void CheckOptimizerConnection(void);
        
        /**
         * @brief Send command to other drones
         * 
         * @param command Command to send
         * @return true if sent successfully
         */
        bool SendCommand(const std::string& command);

        /**
         * @brief Get current time in milliseconds
         * 
         * @return Current time
         */
        uint64_t GetCurrentTimeMs() const;

        // Behavior state
        BehaviourMode_t behaviourMode;   ///< Current behavior mode
        bool vrpnLost;                   ///< Whether VRPN tracking is lost
        std::atomic<bool> serverConnected; ///< Whether server is connected
        
        // Position hold variables
        flair::core::Vector3Df posHold;  ///< Position to hold
        float yawHold;                   ///< Yaw angle to hold
        
        // Communication
        flair::core::UdpSocket *message; ///< Message socket
        
        // UI elements
        flair::gui::PushButton *gotoPosition;         ///< Go to position button
        flair::gui::PushButton *positionHold;         ///< Position hold button
        flair::gui::PushButton *sendSharedParameters; ///< Send parameters button
        flair::gui::PushButton *startCircle;          ///< Start circle button
        flair::gui::PushButton *stopCircle;           ///< Stop circle button
        flair::gui::Label *serverStatusLabel;         ///< Server status label
        
        // Orientation data
        flair::core::AhrsData *customReferenceOrientation; ///< Reference orientation
        flair::core::AhrsData *customOrientation;          ///< Current orientation
        
        // VRPN objects
        flair::meta::MetaVrpnObject *chargeVrpn;           ///< Load VRPN object
        std::vector<const flair::meta::MetaVrpnObject *> allUavs; ///< All UAV VRPN objects
        
        // Control objects
        flair::filter::PositionControl *positionControl;    ///< Position controller
        flair::filter::AttitudeControl *attitudeControl;    ///< Attitude controller
        flair::core::SharedParameters *sharedParameters;    ///< Shared parameters
        flair::filter::TrajectoryGenerator2DCircle *circle; ///< Circle trajectory generator
        
        // Position hold PID controllers
        flair::filter::Pid *u_x, *u_y;  ///< Position PIDs
        
        // Drone identifier
        uint16_t uavIndex;              ///< Index of this drone
        
        // Time tracking
        uint64_t lastServerCheckTime;   ///< Last server check time
        uint64_t lastOptimizerSyncTime; ///< Last optimizer sync time
};

#endif // CHARGESUSPENDUE_H