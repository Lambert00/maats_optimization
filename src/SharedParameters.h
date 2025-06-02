// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
* \file SharedParameters.h
* \brief Class defining shared parameters
* \author Guillaume Sanahuja, Copyright Heudiasyc UMR UTC/CNRS 7253
* \date 2023/04/15
* \version 5.0
*/

#ifndef SHARED_PARAMETERS_H
#define SHARED_PARAMETERS_H

#include <Object.h>
#include <string>
#include <atomic>
#include <vector>

namespace flair {
  namespace core {
    class UdpSocket;
  }
  namespace gui {
    class DoubleSpinBox;
    class Tab;
    class PushButton;
    class Label;
  }
}

namespace flair {
namespace core {

/**
 * @brief Class defining shared parameters
 */
class SharedParameters : public core::Object {
public:
  /**
   * @brief Constructor
   *
   * @param parent Parent object
   * @param name Object name
   */
  SharedParameters(const core::Object *parent, std::string name);

  /**
   * @brief Destructor
   */
  ~SharedParameters();
  
  /**
   * @brief Data structure for shared parameters
   */
  typedef struct datas {
    float lCable;                  ///< Cable length
    float mLoad;                   ///< Load mass
    float mUav;                    ///< UAV mass
    float thetaDesired;            ///< Desired cable angle
    float xDesired;                ///< Desired X position
    float yDesired;                ///< Desired Y position
    float zDesired;                ///< Desired Z position
    float gainCompensationZCharge; ///< Gain compensation
  } datas_t;

  /**
   * @brief Get the UI tab
   *
   * @return Tab pointer
   */
  gui::Tab* GetTab(void) const;
  
  /**
   * @brief Get size of data structure
   *
   * @return Size in bytes
   */
  ssize_t GetSize(void) const;
  
  /**
   * @brief Copy from UI to internal structure
   */
  void CopyFromUI(void);
  
  /**
   * @brief Copy from buffer to internal structure
   *
   * @param buffer Input buffer
   */
  void CopyFromBuffer(char* buffer);
  
  /**
   * @brief Send parameters to other drones
   *
   * @param message UDP socket
   */
  void Send(core::UdpSocket *message);
  
  /**
   * @brief Get current parameters
   *
   * @return Parameters structure
   */
  datas_t GetDatas(void) const;
  
  /**
   * @brief Check if parameters are valid
   *
   * @return true if valid
   */
  bool AreValid(void) const;
  
  /**
   * @brief Check if using optimized distribution
   *
   * @return true if using optimization
   */
  bool GetUseOptimizedDistribution() const;
  
  /**
   * @brief Enable or disable optimization
   *
   * @param enable Whether to enable optimization
   * @param message UDP socket to broadcast change (optional)
   */
  void EnableOptimizer(bool enable, core::UdpSocket* message = nullptr);
  
  /**
   * @brief Update optimizer status UI
   *
   * @param connected Whether server is connected
   */
  void UpdateOptimizerStatus(bool connected);
  
  /**
   * @brief Check UI buttons for clicks
   * 
   * @param message UDP socket to broadcast changes
   */
  void CheckButtons(core::UdpSocket* message = nullptr);
  
  /**
   * @brief Get server address
   *
   * @return Server address
   */
  std::string GetServerAddress() const;
  
  /**
   * @brief Set server address
   * 
   * @param address New server address
   */
  void SetServerAddress(const std::string& address);
  
  /**
   * @brief Get TCP request port
   *
   * @return TCP port number
   */
  int GetServerPort() const;
  
  /**
   * @brief Set TCP server port
   * 
   * @param port Port number
   */
  void SetServerPort(int port);
  
  /**
   * @brief Get UDP result port
   *
   * @return UDP port number
   */
  int GetResultPort() const;
  
  /**
   * @brief Set UDP result port
   * 
   * @param port Port number
   */
  void SetResultPort(int port);
  
  /**
   * @brief Get is master status
   * 
   * @return true if this is the master drone
   */
  bool IsMaster() const;
  
  /**
   * @brief Broadcast the current optimization state to all drones
   * 
   * This ensures all drones have the same optimization setting
   * 
   * @param message UDP socket to use for sending
   */
  void BroadcastOptimizerState(core::UdpSocket* message);

private:
  // UI elements
  gui::Tab *sharedParamTab;
  gui::DoubleSpinBox *xDesired, *yDesired, *zDesired, *lCable, *mLoad, *mUav, *thetaDesired, *gainCompensationZCharge;
  gui::PushButton *useOptimizer;
  gui::Label *connectionStatus;
  
  // Internal data
  datas_t sharedParameters;
  bool areValid;
  
  // Server configuration
  std::string serverAddress;       ///< Server address
  int serverPort;                  ///< TCP server port
  int resultPort;                  ///< UDP result port
  
  // Optimization state (not part of shared parameters struct)
  std::atomic<bool> useOptimizedDistribution; ///< Whether optimized distribution is enabled
  
  // Determine if this is a master drone
  bool isMaster;
  
  // Constants
  static const int DEFAULT_SERVER_PORT = 5555;  ///< Default TCP port
  static const int DEFAULT_RESULT_PORT = 5556;  ///< Default UDP port
};

} // end namespace core
} // end namespace flair

#endif // SHARED_PARAMETERS_H