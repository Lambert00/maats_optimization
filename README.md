# MAATS Tension Optimizer

Real-time tension optimization system for Multi-Agent Aerial Transportation Systems (MAATS). Coordinates multiple drones carrying a cable-suspended payload using server-based optimization.

## System Overview

- **Server**: SNOPT-based three-phase optimizer (172.26.213.70)
- **Client**: Modified Fl-AIR charge-suspendue running on each drone
- **Communication**: Master drone sends TCP requests, all drones receive UDP responses
- **Tracking**: VRPN/OptiTrack for real-time position feedback (172.26.213.1)

## Prerequisites & Installation

### 1. SNOPT 7.x (Commercial License Required)
Request from: https://ccom.ucsd.edu/~optimizers/downloads/

```bash
# After downloading
unzip libsnopt7_cpp.zip.tgz -d $HOME/ # Or another directory
export SNOPT_ROOT=$HOME/SNOPT7
export SNOPT_LICENSE=$SNOPT_ROOT/snopt7.lic
```

### 2. VRPN (Motion Capture)
```bash
git clone --recursive https://github.com/vrpn/vrpn.git
cd vrpn && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SHARED_LIBS=ON
make -j$(nproc) && sudo make install
```

### 3. Fl-AIR Framework
Follow installation guide: https://gitlab.utc.fr/uav-hds/flair/flair-src/-/wikis/home

```bash
# After Fl-AIR installation, clone this repository
git clone https://github.com/Lambert00/maats_optimization.git
cd maats_optimization

# Replace existing charge-suspendue demo files with modified versions
rm -rf $FLAIR_ROOT/flair-build/build/demos/charge-suspendue/uav/src/*
rm -f $FLAIR_ROOT/flair-build/build/demos/charge-suspendue/uav/CMakeLists.txt
cp -r src/* $FLAIR_ROOT/flair-build/build/demos/charge-suspendue/uav/src/
cp CMakeLists.txt $FLAIR_ROOT/flair-build/build/demos/charge-suspendue/uav/

# Rebuild the modified demo
cd $FLAIR_ROOT/flair-build/build/demos/charge-suspendue/build
make install
```

## Building

### Server
```bash
cd server
# If SNOPT is not in $HOME/SNOPT7, specify location:
make SNOPT_ROOT=/path/to/your/snopt -j$(nproc)
# Or use default location:
make -j$(nproc)
```

## Network Configuration

| Component | IP Address | Port | Purpose |
|-----------|------------|------|---------|
| Drone_0 (Master) | 172.26.209.100 | - | Sends optimization requests |
| Drone_1 | 172.26.209.103 | - | Receives optimization results |
| VRPN Server | 172.26.213.1 | 3883 | Motion capture tracking |
| Optimization Server | 172.26.213.70 | 5555/5556 | TCP requests/UDP responses |

## Running Procedure

### Step 1: Start Drone Clients
**On Drone_0 (Master):**
```bash
cd charge_opt/
./ChargeSuspendue.sh
```

**On Drone_1:**
```bash
cd charge_opt/
./ChargeSuspendue.sh
```

### Step 2: Start Optimization Server
```bash
cd server
./MAATSTensionOptimizer -v debug -n 2 -m 5.0 -b 0.1
```

**Expected output:**
```
TCP server initialized on port 5555
UDP socket initialized for unicast communication
VRPN tracking started for 2 objects
Unicast test SUCCEEDED
```

### Step 3: Flight Sequence

#### 3.1 Takeoff (Optimization DISABLED)
1. Open Fl-AIR Ground Control Station
2. **Verify optimization shows "DISABLED"** in GUI (default state)
3. Arm and takeoff both drones
4. System operates in symmetric distribution mode

#### 3.2 Positioning Phase
1. Use "Goto Position" to move charge to (0,0,1.5m)
2. Ensure cables are not tangled
3. Verify VRPN tracking: "charge", "Drone_0", "Drone_1" all tracked
4. Check server shows "VRPN tracking: 2/2 objects tracked"

#### 3.3 Enable Optimization
1. **Click "Enable Optimization"** button in SharedParameters tab
   - OR use joystick: L1 + Triangle
2. GUI should show "Optimizer: Connected (Active)"
3. Server logs: "Received tension request from Drone_0"
4. Monitor optimization success messages

#### 3.4 Mission Execution
- Master drone sends force requests to server
- Server responds with individual tension/direction per drone
- Monitor cable separation (>36° enforced)
- Emergency disable: L1 + Triangle or GUI button

### Step 4: Shutdown
1. **Disable optimization first** (critical safety step)
2. Land drones via standard procedure
3. Stop server (Ctrl+C)
4. Check logs: `requests_*.csv`, `responses_*.csv`

## Server Parameters

| Parameter | Description | Recommended |
|-----------|-------------|-------------|
| `-v debug` | Log level (debug/info/warn/error) | debug |
| `-n 2` | Number of drones | 2 |
| `-m 5.0` | Cable separation weight | 5.0 |
| `-b 0.1` | Direction smoothness | 0.1 |
| `-r 0.35` | Max direction rate (rad/s) | 0.35 |

## GUI Controls

| Control | Action | Purpose |
|---------|--------|---------|
| "Enable Optimization" button | Toggle optimization | Enable/disable server optimization |
| L1 + Triangle | Toggle optimization | Joystick optimization control |
| R1 + Circle | Start circle | Begin circular trajectory |
| R1 + Square | Position hold | Maintain current position |
| L2/R2 | Start/Stop logging | Data logging control |

## Troubleshooting

### Server Connection Issues
```bash
# Check server listening
netstat -tlnp | grep 5555

# Test drone connectivity
ping 172.26.209.100
ping 172.26.209.103

# Verify VRPN
vrpn_print_devices Drone_0@172.26.213.1
```

### Optimization Not Working
1. **Check GUI status**: Must show "Optimizer: Connected (Active)"
2. **Master requests**: Server should log "Received tension request from Drone_0"
3. **Toggle optimization**: Try disable/enable via GUI
4. **Fallback check**: Look for "Falling back to symmetric distribution"

### Flight Instability
1. **Immediate**: Disable optimization via L1+Triangle or GUI
2. **Automatic**: System falls back to symmetric distribution
3. **Manual control**: Take over and land safely
4. **Logs**: Check server for optimization errors

## Safety Features

- **Automatic fallback**: Reverts to symmetric distribution if optimization fails
- **Cable collision avoidance**: Enforces >36° separation between cables
- **Direction rate limiting**: Prevents abrupt changes (5.7°/s max)
- **VRPN loss detection**: Emergency landing if tracking lost
- **Instant disable**: GUI or joystick emergency optimization disable

## Output Files

- `requests_<timestamp>.csv`: Optimization requests from master drone
- `responses_<timestamp>.csv`: Server responses to all drones
- Fl-AIR logs: Standard flight data and parameters

## Expected Performance

- Optimization frequency: 20-60 Hz
- Success rate: >95%
- Force equilibrium error: <3%
- Network latency: <10ms
- Cable separation: Always >36°
