/**
 * MAATSTensionOptimizer.cpp
 * 
 * Optimized Tension Controller for MAATS
 * Three-Phase Optimization Architecture
 */

#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>
#include <queue>
#include <map>
#include <unordered_map>
#include <deque>
#include <cstdio>
#include <cstring>
#include <csignal>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <getopt.h>
#include <fcntl.h>
#include <netdb.h>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <condition_variable>
#include <functional>
#include <memory>
#include <future>
#include <fstream>

// Eigen for matrix operations in tension-only optimization
#include <Eigen/Dense>

// VRPN includes
#include <vrpn_Tracker.h>
#include <vrpn_Connection.h>

// SNOPT includes for optimization
#include "snoptProblem.hpp"

typedef int    integer;
typedef double doublereal;

// Logger class for handling log messages with different levels
class Logger {
public:
    enum Level { DEBUG, INFO, WARN, ERROR, NONE };
    
    Logger(Level level = INFO) : level_(level) {}
    
    template<typename... Args>
    void debug(const char* format, Args... args) {
        if (level_ <= DEBUG) log("[DEBUG] ", format, args...);
    }
    
    template<typename... Args>
    void info(const char* format, Args... args) {
        if (level_ <= INFO) log("[INFO] ", format, args...);
    }
    
    template<typename... Args>
    void warn(const char* format, Args... args) {
        if (level_ <= WARN) log("[WARN] ", format, args...);
    }
    
    template<typename... Args>
    void error(const char* format, Args... args) {
        if (level_ <= ERROR) log("[ERROR] ", format, args...);
    }
    
    void setLevel(Level level) { level_ = level; }

private:
    Level level_;
    std::mutex mutex_;
    
    template<typename... Args>
    void log(const char* prefix, const char* format, Args... args) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Get current time
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        struct tm timeinfo;
        localtime_r(&time_t_now, &timeinfo);
        char timestr[20];
        strftime(timestr, sizeof(timestr), "%Y-%m-%d %H:%M:%S", &timeinfo);
        
        // Format and print log message
        std::cout << timestr << " " << prefix;
        
        if (sizeof...(args) == 0) {
            // Direct output if no format arguments
            std::cout << format;
        } else {
            char buffer[4096];
            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Wformat-security"
            snprintf(buffer, sizeof(buffer), format, args...);
            #pragma GCC diagnostic pop
            std::cout << buffer;
        }
        std::cout << std::endl;
    }
};

// Global logger
Logger logger;

// Vector3D class to handle 3D vectors
struct Vector3D {
    double x, y, z;
    
    Vector3D() : x(0.0), y(0.0), z(0.0) {}
    Vector3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    
    double norm() const {
        return std::sqrt(x*x + y*y + z*z);
    }
    
    void normalize() {
        double n = norm();
        if (n > 1e-10) {
            x /= n;
            y /= n;
            z /= n;
        } else {
            x = 0.0;
            y = 0.0;
            z = 1.0;
        }
    }
    
    Vector3D& operator=(const Vector3D& other) {
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }
    
    Vector3D operator+(const Vector3D& other) const {
        return Vector3D(x + other.x, y + other.y, z + other.z);
    }
    
    Vector3D operator-(const Vector3D& other) const {
        return Vector3D(x - other.x, y - other.y, z - other.z);
    }
    
    Vector3D operator*(double scalar) const {
        return Vector3D(x * scalar, y * scalar, z * scalar);
    }
    
    double dot(const Vector3D& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
};

// Simplified data structure for the SNOPT problem
struct TensionProblemData {
    Vector3D resultantForce;          // Desired resultant force vector
    int numCables;                    // Number of cables
    std::vector<Vector3D> prevDirections;    // Previous direction solution
    double mu;                        // Cable separation weight
    double directionBias;             // Continuity enforcement weight
};

// Global pointer and mutex for thread safety with SNOPT
static TensionProblemData* g_tensionProblemPtr = nullptr;
static std::mutex g_tensionProblemMutex;

// Magic numbers for message validation
#define TENSION_REQUEST_MAGIC    0xDECA0001  // Magic number for TCP optimization requests
#define TENSION_UNICAST_MAGIC    0xDECA0005  // Magic number for unicast messages
#define TENSION_HEARTBEAT_MAGIC  0xDECA0003  // Magic number for TCP heartbeat messages
#define TENSION_POSITION_MAGIC   0xDECA0004  // Magic number for position update messages

// Server version information
#define SERVER_VERSION_MAJOR 3
#define SERVER_VERSION_MINOR 0
#define SERVER_VERSION_PATCH 0

// Message formats
#pragma pack(push, 1)

// TCP request message format
struct TcpRequestMessage {
    uint32_t magic;
    uint32_t sequenceNumber;
    uint32_t droneId;
    uint64_t timestamp;
    float resultantForce[3];
    uint16_t checksum;            
};

// Unicast UDP message format
struct UdpUnicastMessage {
    uint32_t magic;
    uint32_t sequenceNumber;
    uint8_t droneIndex;
    float direction[3]; // Direction vector [x,y,z]
    float tension;  // Tension value in Newtons
    uint16_t checksum;
};

// Position data message format
struct PositionData {
    uint32_t magic;             
    uint32_t objectId;
    uint64_t timestamp;
    float position[3];  // Position vector (x, y, z)
    float orientation[4];   // Orientation quaternion (qw, qx, qy, qz)
    float velocity[3];  // Velocity vector
    bool tracked;   // Whether object is currently tracked
};

#pragma pack(pop)

inline uint64_t GetCurrentTimeMs() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

// CSV Logger for tension data
class CsvLogger {
public:
    CsvLogger() : requests_file_open_(false), responses_file_open_(false) {}
    
    bool openFiles(const std::string& timestamp_str) {
        std::string requests_filename = "requests_" + timestamp_str + ".csv";
        std::string responses_filename = "responses_" + timestamp_str + ".csv";
        
        requests_file_.open(requests_filename, std::ios::out);
        responses_file_.open(responses_filename, std::ios::out);
        
        if (requests_file_.is_open() && responses_file_.is_open()) {
            requests_file_open_ = true;
            responses_file_open_ = true;
            return true;
        }
        
        if (requests_file_.is_open()) {
            requests_file_.close();
        }
        if (responses_file_.is_open()) {
            responses_file_.close();
        }
        
        return false;
    }
    
    void logRequest(const TcpRequestMessage& request, uint64_t timestamp) {
        if (!requests_file_open_) return;
        
        std::lock_guard<std::mutex> lock(requests_mutex_);
        requests_file_ << timestamp << ","
                      << request.resultantForce[0] << ","
                      << request.resultantForce[1] << ","
                      << request.resultantForce[2] << "\n";
        requests_file_.flush();
    }
    
    void logResponse(const UdpUnicastMessage& response, uint64_t timestamp) {
        if (!responses_file_open_) return;
        
        std::lock_guard<std::mutex> lock(responses_mutex_);
        responses_file_ << timestamp << ","
                       << (int)response.droneIndex << ","
                       << response.direction[0] << ","
                       << response.direction[1] << ","
                       << response.direction[2] << ","
                       << response.tension << "\n";
        responses_file_.flush();
    }
    
    void close() {
        if (requests_file_open_) {
            std::lock_guard<std::mutex> lock(requests_mutex_);
            requests_file_.close();
            requests_file_open_ = false;
        }
        if (responses_file_open_) {
            std::lock_guard<std::mutex> lock(responses_mutex_);
            responses_file_.close();
            responses_file_open_ = false;
        }
    }
    
    ~CsvLogger() {
        close();
    }
    
private:
    std::ofstream requests_file_;
    std::ofstream responses_file_;
    bool requests_file_open_;
    bool responses_file_open_;
    std::mutex requests_mutex_;
    std::mutex responses_mutex_;
};

// Global CSV logger
CsvLogger csvLogger;

// SNOPT problem function
extern "C" {
void SNOPT_tensionUserFun(integer *Status, integer *n, doublereal x[],
                        integer *needF, integer *neF, doublereal F[],
                        integer *needG, integer *neG, doublereal G[],
                        char *cu, integer *lencu,
                        integer iu[], integer *leniu,
                        doublereal ru[], integer *lenru)
{
    if(*needF == 0) return;

    TensionProblemData& pd = *g_tensionProblemPtr;
    int c = pd.numCables;
    
    // Tensions
    std::vector<double> tensions(c);
    for (int i = 0; i < c; i++) {
        tensions[i] = x[i];
    }
    
    // Direction components
    std::vector<Vector3D> directions(c);
    for (int i = 0; i < c; i++) {
        directions[i].x = x[c + 3*i + 0];
        directions[i].y = x[c + 3*i + 1];
        directions[i].z = x[c + 3*i + 2];
    }
    
    // Objective: minimize sum of squared tensions
    double tensionSum = 0.0;
    for (int i = 0; i < c; i++) {
        tensionSum += tensions[i] * tensions[i];
    }
    
    // Direction proximity penalty
    double proximitySum = 0.0;
    for (int i = 0; i < c; i++) {
        for (int j = i+1; j < c; j++) {
            double dotProduct = directions[i].x * directions[j].x + 
                            directions[i].y * directions[j].y + 
                            directions[i].z * directions[j].z;
            proximitySum += dotProduct * dotProduct;
        }
    }
    
    // Direction continuity penalty
    double continuityPenalty = 0.0;
    if (!pd.prevDirections.empty()) {
        for (int i = 0; i < c && i < static_cast<int>(pd.prevDirections.size()); i++) {
            // Calculate dot product with previous direction
            double dot = directions[i].x * pd.prevDirections[i].x + 
                        directions[i].y * pd.prevDirections[i].y + 
                        directions[i].z * pd.prevDirections[i].z;
            
            // Basic penalty for deviation
            continuityPenalty += (1.0 - dot) * (1.0 - dot);
            
            // Add moderate penalty for sign flips
            if (pd.prevDirections[i].x * directions[i].x < 0 && 
                std::abs(pd.prevDirections[i].x) > 0.1) {
                continuityPenalty += 3.0 * std::abs(directions[i].x);
            }
            
            if (pd.prevDirections[i].y * directions[i].y < 0 && 
                std::abs(pd.prevDirections[i].y) > 0.1) {
                continuityPenalty += 3.0 * std::abs(directions[i].y);
            }
        }
    }
    
    // Combined objective
    F[0] = 0.5 * tensionSum + pd.mu * proximitySum + pd.directionBias * continuityPenalty;
    
    // Standard force equilibrium constraints
    double sumFx = 0.0, sumFy = 0.0, sumFz = 0.0;
    for (int i = 0; i < c; i++) {
        sumFx += tensions[i] * directions[i].x;
        sumFy += tensions[i] * directions[i].y;
        sumFz += tensions[i] * directions[i].z;
    }
    
    F[1] = sumFx - pd.resultantForce.x;
    F[2] = sumFy - pd.resultantForce.y;
    F[3] = sumFz - pd.resultantForce.z;
    
    // Direction normalization constraints
    for (int i = 0; i < c; i++) {
        double normSquared = directions[i].x * directions[i].x + 
                            directions[i].y * directions[i].y + 
                            directions[i].z * directions[i].z;
        F[4 + i] = normSquared - 1.0; // |alpha_i|^2 - 1 = 0
    }

    (void)Status; (void)needG; (void)neG; (void)G; 
    (void)cu; (void)lencu; (void)iu; (void)leniu; (void)ru; (void)lenru;
}
}

/**
 * Phase 2: Direction Rate Limiting
 * 
 * @param directions [in,out] Direction vectors to apply rate limiting to
 * @param directionsWereModified [out] Boolean flags indicating which directions were modified
 * @param prevDirections [in] Previous direction vectors
 * @param maxRateRad [in] Maximum allowed direction change rate in radians
 * @return true if any directions were modified
 */
bool ProcessRateLimitedDirections(
    std::vector<Vector3D>& directions,
    std::vector<bool>& directionsWereModified,
    const std::vector<Vector3D>& prevDirections,
    double maxRateRad)
{
    int numDrones = directions.size();
    bool anyDirectionModified = false;
    directionsWereModified.resize(numDrones, false);
    
    // Statistics tracking
    int directionsRateLimited = 0;
    double maxAngleBeforeLimiting = 0.0;
    double sumAngleBeforeLimiting = 0.0;
    double maxAngleAfterLimiting = 0.0;
    int validAngleCount = 0;
    
    // Phase 1: Apply rate limiting to raw directions
    for (int i = 0; i < numDrones; i++) {
        if (i >= (int)prevDirections.size()) continue;
        
        // Calculate angle between current and previous directions
        double dot = directions[i].dot(prevDirections[i]);
        dot = std::max(-1.0, std::min(1.0, dot)); // Clamp to valid range
        double angle = std::acos(dot);

        if (angle > 0.0001) {
            sumAngleBeforeLimiting += angle;
            maxAngleBeforeLimiting = std::max(maxAngleBeforeLimiting, angle);
            validAngleCount++;
        }
        
        // Apply rate limiting if angle change exceeds limit
        if (angle > maxRateRad) {
            directionsRateLimited++;
            anyDirectionModified = true;
            directionsWereModified[i] = true;
            
            logger.debug("Drone %d: Direction change %.2f° exceeds limit %.2f° - applying rate limiting",
                    i, angle * 180.0 / M_PI, maxRateRad * 180.0 / M_PI);
                    
            // Calculate interpolation factor
            double t = maxRateRad / angle;
            
            // SLERP interpolation
            double sinAngle = std::sin(angle);
            if (sinAngle > 1e-6) {
                double coeff1 = std::sin((1-t) * angle) / sinAngle;
                double coeff2 = std::sin(t * angle) / sinAngle;
                
                Vector3D newDir;
                newDir.x = coeff1 * prevDirections[i].x + coeff2 * directions[i].x;
                newDir.y = coeff1 * prevDirections[i].y + coeff2 * directions[i].y;
                newDir.z = coeff1 * prevDirections[i].z + coeff2 * directions[i].z;
                
                // Normalize
                double norm = newDir.norm();
                if (norm > 1e-10) {
                    newDir.x /= norm;
                    newDir.y /= norm;
                    newDir.z /= norm;
                }
                
                // Calculate angle after limiting for statistics
                double dotAfter = newDir.dot(prevDirections[i]);
                dotAfter = std::max(-1.0, std::min(1.0, dotAfter));
                double angleAfter = std::acos(dotAfter);
                maxAngleAfterLimiting = std::max(maxAngleAfterLimiting, angleAfter);
                
                // Log what changed
                logger.debug("Drone %d: Direction before limiting [%.3f, %.3f, %.3f]", 
                        i, directions[i].x, directions[i].y, directions[i].z);
                logger.debug("Drone %d: Direction after limiting  [%.3f, %.3f, %.3f]", 
                        i, newDir.x, newDir.y, newDir.z);
                        
                directions[i] = newDir;
            }
        }
        else {
            // Track the angle
            double dotAfter = directions[i].dot(prevDirections[i]);
            dotAfter = std::max(-1.0, std::min(1.0, dotAfter));
            double angleAfter = std::acos(dotAfter);
            maxAngleAfterLimiting = std::max(maxAngleAfterLimiting, angleAfter);
        }
    }
    
    // Log statistics about direction changes
    if (validAngleCount > 0) {
        double avgAngleBeforeLimiting = sumAngleBeforeLimiting / validAngleCount;
        logger.info("Direction statistics: %d/%d directions rate limited (%.1f%%)", 
                directionsRateLimited, numDrones, 
                100.0 * directionsRateLimited / (double)numDrones);
        logger.info("Direction angles: avg=%.2f°, max=%.2f° before limiting, max=%.2f° after limiting", 
                avgAngleBeforeLimiting * 180.0 / M_PI,
                maxAngleBeforeLimiting * 180.0 / M_PI, 
                maxAngleAfterLimiting * 180.0 / M_PI);
    }
    
    return anyDirectionModified;
}

/**
 * Phase 3: Tension-Only Optimization
 * 
 * @param tensions [in,out] Tension values to be re-optimized
 * @param directions [in] Direction vectors (after rate limiting)
 * @param targetForce [in] Desired resultant force vector
 * @return true if force equilibrium was maintained
 */
bool OptimizeTensionsOnly(
    std::vector<double>& tensions,
    const std::vector<Vector3D>& directions,
    const Vector3D& targetForce)
{
    int numDrones = directions.size();
    
    // Log the target force for debugging
    double targetForceMag = targetForce.norm();
    logger.debug("Tension re-optimization: Target force [%.3f, %.3f, %.3f], magnitude %.3f N", 
                targetForce.x, targetForce.y, targetForce.z, targetForceMag);
    
    // Skip re-optimization if target force is too small - prevents division by near-zero
    if (targetForceMag < 0.05) {
        logger.warn("Target force magnitude too small (%.5f N), skipping tension re-optimization", targetForceMag);
        return false;
    }
    
    if (numDrones <= 3) {
        // Create the force equilibrium matrix A
        Eigen::MatrixXd A(3, numDrones);
        Eigen::Vector3d b(targetForce.x, targetForce.y, targetForce.z);
        
        for (int i = 0; i < numDrones; i++) {
            A(0, i) = directions[i].x;
            A(1, i) = directions[i].y;
            A(2, i) = directions[i].z;
        }
        
        // Log direction vectors for debugging
        for (int i = 0; i < numDrones; i++) {
            logger.debug("Drone %d direction: [%.4f, %.4f, %.4f]", 
                        i, directions[i].x, directions[i].y, directions[i].z);
        }
        
        // Check matrix A conditioning
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A);
        double conditionNumber = svd.singularValues()(0) / 
                                svd.singularValues()(svd.singularValues().size()-1);
        
        int matrixRank = svd.rank();
        logger.debug("Direction matrix rank: %d/%d, condition number: %.2f", 
                    matrixRank, std::min(3, numDrones), conditionNumber);
        
        if (conditionNumber > 1000) {
            logger.warn("Matrix ill-conditioned (cond=%.2f), cable directions may be near-collinear", 
                        conditionNumber);
        }
        
        if (matrixRank < std::min(3, numDrones)) {
            logger.warn("Direction matrix is rank deficient (%d < %d), force space not fully spanned", 
                        matrixRank, std::min(3, numDrones));
        }
        
        // For 2 drones, we can use direct substitution and clamping
        if (numDrones == 2) {
            
            // Create a 3x2 system
            Eigen::MatrixXd ATA = A.transpose() * A;
            Eigen::VectorXd ATb = A.transpose() * b;
            
            // For a 2x2 system, we can compute the inverse directly
            double det = ATA(0,0) * ATA(1,1) - ATA(0,1) * ATA(1,0);
            logger.debug("2-drone system: ATA determinant = %.6f", det);
            
            if (fabs(det) < 1e-10) {
                // Nearly singular, distribute the load evenly
                logger.warn("Near-singular matrix (det=%.8f) in 2-drone system, using equal tension fallback", det);
                double avgTension = targetForceMag / 2.0;
                tensions[0] = avgTension;
                tensions[1] = avgTension;
                
                logger.debug("ATA matrix: [[%.4f, %.4f], [%.4f, %.4f]]", 
                        ATA(0,0), ATA(0,1), ATA(1,0), ATA(1,1));
            } else {
                // Compute the pseudo-inverse solution
                Eigen::VectorXd unconstrained(2);
                unconstrained(0) = (ATA(1,1) * ATb(0) - ATA(0,1) * ATb(1)) / det;
                unconstrained(1) = (ATA(0,0) * ATb(1) - ATA(1,0) * ATb(0)) / det;
                
                logger.debug("Unconstrained solution: [%.4f, %.4f]", unconstrained(0), unconstrained(1));
                
                // Apply non-negativity constraints
                bool anyNegative = false;
                if (unconstrained(0) < 0) {
                    logger.warn("Negative tension calculated for drone 0: %.4f N", unconstrained(0));
                    unconstrained(0) = 0.01;  // Small positive value
                    anyNegative = true;
                }
                if (unconstrained(1) < 0) {
                    logger.warn("Negative tension calculated for drone 1: %.4f N", unconstrained(1));
                    unconstrained(1) = 0.01;  // Small positive value
                    anyNegative = true;
                }
                
                // Scale to maintain force
                if (anyNegative) {
                    logger.debug("Adjusting for negative tensions and rescaling...");
                    
                    // Recompute the actual force
                    Vector3D actualForce;
                    actualForce.x = A(0,0) * unconstrained(0) + A(0,1) * unconstrained(1);
                    actualForce.y = A(1,0) * unconstrained(0) + A(1,1) * unconstrained(1);
                    actualForce.z = A(2,0) * unconstrained(0) + A(2,1) * unconstrained(1);
                    
                    double actualMag = actualForce.norm();
                    if (actualMag > 1e-6) {
                        double scale = targetForceMag / actualMag;
                        unconstrained *= scale;
                        logger.debug("After scaling: [%.4f, %.4f] (scale=%.4f)", 
                                unconstrained(0), unconstrained(1), scale);
                    } else {
                        logger.warn("Actual force magnitude too small (%.8f N) for rescaling", actualMag);
                    }
                }
                
                tensions[0] = unconstrained(0);
                tensions[1] = unconstrained(1);
            }
            
        } else if (numDrones == 3) {
            // For 3 drones
            Eigen::MatrixXd ATA = A.transpose() * A;
            Eigen::VectorXd ATb = A.transpose() * b;
            
            // Check ATA determinant
            double det = ATA.determinant();
            logger.debug("3-drone system: ATA determinant = %.6f", det);
            
            if (fabs(det) < 1e-9) {
                logger.warn("Near-singular ATA matrix (det=%.8f) in 3-drone system", det);
                // Log the ATA matrix for debugging
                logger.debug("ATA matrix:\n[[%.4f, %.4f, %.4f], [%.4f, %.4f, %.4f], [%.4f, %.4f, %.4f]]",
                        ATA(0,0), ATA(0,1), ATA(0,2),
                        ATA(1,0), ATA(1,1), ATA(1,2),
                        ATA(2,0), ATA(2,1), ATA(2,2));
            }
            
            // Compute the pseudo-inverse solution
            Eigen::VectorXd unconstrained(3);
            
            if (fabs(det) > 1e-9) {
                unconstrained = ATA.ldlt().solve(ATb);
                logger.debug("Used LDLT solver");
            } else {
                unconstrained = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
                logger.debug("Used SVD solver due to near-singular matrix");
            }
            
            logger.debug("Unconstrained solution: [%.4f, %.4f, %.4f]", 
                        unconstrained(0), unconstrained(1), unconstrained(2));
            
            // Apply non-negativity constraints
            bool anyNegative = false;
            for (int i = 0; i < 3; i++) {
                if (unconstrained(i) < 0) {
                    logger.warn("Negative tension calculated for drone %d: %.4f N", i, unconstrained(i));
                    unconstrained(i) = 0.01;
                    anyNegative = true;
                }
            }
            
            // If we had to adjust for non-negativity, scale to maintain force
            if (anyNegative) {
                logger.debug("Adjusting for negative tensions and rescaling...");
                
                Vector3D actualForce;
                for (int i = 0; i < 3; i++) {
                    actualForce.x += A(0,i) * unconstrained(i);
                    actualForce.y += A(1,i) * unconstrained(i);
                    actualForce.z += A(2,i) * unconstrained(i);
                }
                
                double actualMag = actualForce.norm();
                if (actualMag > 1e-6) {
                    double scale = targetForceMag / actualMag;
                    unconstrained *= scale;
                    logger.debug("After scaling: [%.4f, %.4f, %.4f] (scale=%.4f)", 
                            unconstrained(0), unconstrained(1), unconstrained(2), scale);
                } else {
                    logger.warn("Actual force magnitude too small (%.8f N) for rescaling", actualMag);
                }
            }
            
            // Copy the result
            for (int i = 0; i < 3; i++) {
                tensions[i] = unconstrained(i);
            }
        }
        
        // Verify force equilibrium
        Vector3D actualForce;
        for (int i = 0; i < numDrones; i++) {
            actualForce.x += tensions[i] * directions[i].x;
            actualForce.y += tensions[i] * directions[i].y;
            actualForce.z += tensions[i] * directions[i].z;
        }
        
        Vector3D errorVec = actualForce - targetForce;
        double errorMag = errorVec.norm();
        double relError = errorMag / targetForceMag;
        
        // Log the final tensions
        std::string tensionStr = "Final tensions: [";
        for (int i = 0; i < numDrones; i++) {
            tensionStr += std::to_string(tensions[i]);
            if (i < numDrones - 1) tensionStr += ", ";
        }
        tensionStr += "]";
        logger.debug("%s", tensionStr.c_str());
        
        logger.debug("Actual force after re-optimization: [%.3f, %.3f, %.3f], magnitude %.3f N", 
                    actualForce.x, actualForce.y, actualForce.z, actualForce.norm());
        logger.debug("Force equilibrium error: %.3f N (%.2f%%)", errorMag, relError * 100.0);
        
        // Check if force equilibrium is maintained
        double toleranceThreshold = 0.03;
        if (targetForceMag < 1.0) {
            toleranceThreshold = 0.05;
        }
        
        if (relError <= toleranceThreshold) {
            logger.debug("Force equilibrium maintained within %.1f%% tolerance", toleranceThreshold * 100.0);
            return true;
        } else {
            logger.warn("Force equilibrium error (%.2f%%) exceeds %.1f%% tolerance", 
                    relError * 100.0, toleranceThreshold * 100.0);
            return false;
        }
    }
    
    logger.warn("Tension-only optimization only implemented for 2-3 drones");
    return false;
}

/**
 * RAII wrapper for socket file descriptors
 * Automatically closes the socket on destruction
 */
class Socket {
public:
    Socket() : fd_(-1) {}
    explicit Socket(int fd) : fd_(fd) {}
    
    ~Socket() {
        close();
    }
    
    // Non-copyable
    Socket(const Socket&) = delete;
    Socket& operator=(const Socket&) = delete;
    
    // Movable
    Socket(Socket&& other) noexcept : fd_(other.fd_) {
        other.fd_ = -1;
    }
    
    Socket& operator=(Socket&& other) noexcept {
        if (this != &other) {
            close();
            fd_ = other.fd_;
            other.fd_ = -1;
        }
        return *this;
    }
    
    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
    
    bool valid() const { return fd_ >= 0; }
    int get() const { return fd_; }
    operator int() const { return fd_; }
    
    void set(int fd) {
        if (fd_ >= 0) {
            ::close(fd_);
        }
        fd_ = fd;
    }
    
    int release() {
        int temp = fd_;
        fd_ = -1;
        return temp;
    }
    
private:
    int fd_;
};

class TensionOptimizer {
public:
    struct InputData {
        Vector3D resultantForce;
        int numCables;
        std::vector<Vector3D> initialDirections;
        std::vector<Vector3D> prevDirections;      // Previous optimization result
        double mu;                                // Cable separation weight
        double directionBias;                     // Direction continuity weight
    };
    
    struct OutputData {
        bool success;
        double objectiveValue;
        std::vector<double> tensions;
        std::vector<Vector3D> directions;
        double solveTimeMs;
        std::string errorMessage;
    };
    
    OutputData optimize(const InputData& input) {
        // Lock the mutex
        std::lock_guard<std::mutex> lock(g_tensionProblemMutex);
        
        logger.debug("Starting SNOPT optimization with %d cables", input.numCables);
        
        auto startTime = std::chrono::high_resolution_clock::now();
        
        OutputData output;
        output.success = false;
        
        // Copy input to problem data for SNOPT user function
        TensionProblemData problemData;
        problemData.resultantForce = input.resultantForce;
        problemData.numCables = input.numCables;
        problemData.prevDirections = input.prevDirections;
        problemData.mu = input.mu;
        problemData.directionBias = input.directionBias;
        
        // Set global pointer for SNOPT function
        g_tensionProblemPtr = &problemData;
        
        int c = input.numCables;
        int n = c + 3*c;
        int nF = 1 + 3 + c; // 1 objective + 3 force equilibrium + c direction normalization
        
        // Initialize arrays for SNOPT
        std::vector<doublereal> x(n), xlow(n), xupp(n);
        std::vector<doublereal> F(nF), Flow(nF), Fupp(nF);
        std::vector<doublereal> xmul(n, 0), Fmul(nF, 0);
        std::vector<integer> xstate(n, 0), Fstate(nF, 0);
        
        // Calculate initial tension
        double forceMagnitude = input.resultantForce.norm();
        double initialTension = forceMagnitude / c;
        
        // Set initial values for tensions
        for (int i = 0; i < c; i++) {
            x[i] = initialTension;
        }
        
        // Set initial directions
        bool hasPreviousSolution = !input.prevDirections.empty() && 
                                input.prevDirections.size() >= static_cast<size_t>(c);
        
        for (int i = 0; i < c; i++) {
            Vector3D dir;
            
            if (hasPreviousSolution) {
                // Use previous solution as initial point for warm start
                dir = input.prevDirections[i];
            }
            else if (i < static_cast<int>(input.initialDirections.size())) {
                dir = input.initialDirections[i];
            } 
            else {
                // Create default directions if not provided
                double angle = 2.0 * M_PI * i / c;
                dir.x = 0.5 * cos(angle);
                dir.y = 0.5 * sin(angle);
                dir.z = 0.7071;
            }
            
            // Ensure normalized direction
            double norm = dir.norm();
            if (norm > 1e-10) {
                dir.x /= norm;
                dir.y /= norm;
                dir.z /= norm;
            } else {
                // Default direction if norm is too small
                dir.x = 0.0;
                dir.y = 0.0;
                dir.z = 1.0;
            }
            
            x[c + 3*i + 0] = dir.x;
            x[c + 3*i + 1] = dir.y;
            x[c + 3*i + 2] = dir.z;
        }
        
        // Set bounds for variables
        for (int i = 0; i < n; i++) {
            xlow[i] = -1e20;
            xupp[i] = 1e20;
        }
        
        // Tensions must be non-negative
        for (int i = 0; i < c; i++) {
            xlow[i] = 0.0;
        }
        
        // Set F bounds
        Flow[0] = -1e20;
        Fupp[0] = 1e20;
        
        // Force equilibrium constraints (equality)
        for (int i = 1; i <= 3; i++) {
            Flow[i] = 0.0;
            Fupp[i] = 0.0;
        }
        
        // Direction normalization constraints (equality)
        for (int i = 4; i < nF; i++) {
            Flow[i] = 0.0;
            Fupp[i] = 0.0;
        }
        
        // Initialize SNOPT problem
        snoptProblemA snProb;
        snProb.initialize("", 1);

        // Set SNOPT options
        snProb.setIntParameter("Major print level", 0);
        snProb.setIntParameter("Minor print level", 0);
        snProb.setIntParameter("Summary file", 0);
        snProb.setIntParameter("Derivative option", 0);  // Use finite differencing
        snProb.setIntParameter("Major iterations limit", 500);
        snProb.setRealParameter("Major feasibility tolerance", 1e-7);

        // Determine start type based on whether we have previous directions
        integer starttype = hasPreviousSolution ? 2 : 0;  // 2 for warm start, 0 for cold start
        integer iObj = 0;  // Index of objective function
        doublereal ObjAdd = 0.0;
        integer nS = 0, nInf = 0;
        doublereal sInf = 0.0;

        if (hasPreviousSolution) {
            logger.debug("Using warm start (starttype=2) with previous solution");
        } else {
            logger.debug("Using cold start (starttype=0)");
        }

        logger.debug("SNOPT setup complete, starting solve...");

        // Solve
        integer status = 0;
        try {
            status = snProb.solve(
                starttype,
                nF, n,
                ObjAdd, iObj,
                SNOPT_tensionUserFun,
                xlow.data(), xupp.data(),
                Flow.data(), Fupp.data(),
                x.data(), xstate.data(), xmul.data(),
                F.data(), Fstate.data(), Fmul.data(),
                nS, nInf, sInf
            );
            
            logger.debug("SNOPT solve returned with status %d", status);
        }
        catch (const std::exception& e) {
            logger.error("SNOPT exception: %s", e.what());
            output.errorMessage = "SNOPT exception: " + std::string(e.what());
            g_tensionProblemPtr = nullptr;
            return output;
        }
        catch (...) {
            logger.error("Unknown SNOPT exception");
            output.errorMessage = "Unknown SNOPT exception";
            g_tensionProblemPtr = nullptr;
            return output;
        }
        
        // Extract results
        output.objectiveValue = F[0];
        output.success = (status == 1);
        
        // Extract tensions and directions
        output.tensions.resize(c);
        output.directions.resize(c);
        
        for (int i = 0; i < c; i++) {
            output.tensions[i] = x[i];
            
            Vector3D dir;
            dir.x = x[c + 3*i + 0];
            dir.y = x[c + 3*i + 1];
            dir.z = x[c + 3*i + 2];
            
            // Normalize
            double norm = dir.norm();
            if (norm > 1e-10) {
                dir.x /= norm;
                dir.y /= norm;
                dir.z /= norm;
            } else {
                dir.x = 0.0;
                dir.y = 0.0;
                dir.z = 1.0;
            }
            
            output.directions[i] = dir;
        }
        
        // Set error message if optimization failed
        if (!output.success) {
            switch (status) {
                case 2:
                    output.errorMessage = "Optimal solution is infeasible";
                    break;
                case 3:
                    output.errorMessage = "Unbounded problem";
                    break;
                case 4:
                    output.errorMessage = "Maximum iterations reached";
                    break;
                default:
                    output.errorMessage = "Optimization failed with status " + std::to_string(status);
                    break;
            }
        }
        
        // Calculate solve time
        auto endTime = std::chrono::high_resolution_clock::now();
        output.solveTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
        
        logger.debug("SNOPT optimization completed in %fms with status %d", output.solveTimeMs, status);
        
        // Clear global pointer
        g_tensionProblemPtr = nullptr;
        
        return output;
    }
};

/**
 * VRPN client for tracking drone positions
 */
class VrpnTracker {
    public:
        VrpnTracker(const std::string& host, int port, const std::vector<std::string>& objectNames)
            : running_(false), 
            host_(host),
            port_(port)
        {
            // Reserve space for position data
            positionData_.resize(objectNames.size() + 1); // +1 for load
            
            // Initialize objects in tracking list
            trackingObjects_.push_back("charge"); // Load is always object 0
            for (const auto& name : objectNames) {
                trackingObjects_.push_back(name);
            }
        }
        
        ~VrpnTracker() {
            Stop();
        }
        
        bool Start() {
            if (running_) {
                return true;
            }
            
            // Connection string
            std::stringstream ss;
            ss << host_;
            if (port_ != 3883) { // Custom port
                ss << ":" << port_;
            }
            std::string connectionString = ss.str();
            
            logger.info("Connecting to VRPN server at %s", connectionString.c_str());
            
            // Create a connection to the VRPN server
            connection_ = vrpn_get_connection_by_name(connectionString.c_str());
            if (!connection_) {
                logger.error("Failed to connect to VRPN server at %s", connectionString.c_str());
                return false;
            }
            
            // Create a tracker object for each tracked object
            for (size_t i = 0; i < trackingObjects_.size(); i++) {
                // Create tracker
                vrpn_Tracker_Remote* tracker = new vrpn_Tracker_Remote(
                    trackingObjects_[i].c_str(), connection_);
                
                // Register callbacks with the object ID as user data
                tracker_data_t* userData = new tracker_data_t();
                userData->tracker = this;
                userData->objectId = i;
                
                // Register position callback
                tracker->register_change_handler(userData, VrpnTracker::HandleTrackerPosition, 0);
                
                // Register velocity callback for velocity data
                tracker->register_change_handler(userData, VrpnTracker::HandleTrackerVelocity, 0);
                
                trackers_.push_back(tracker);
                userDataList_.push_back(userData);
                
                // Initialize position data
                positionData_[i].magic = TENSION_POSITION_MAGIC;
                positionData_[i].objectId = i;
                positionData_[i].tracked = false;
            }
            
            // Start the tracking thread
            running_ = true;
            trackingThread_ = std::thread(&VrpnTracker::TrackingLoop, this);
            
            logger.info("VRPN tracking started for %zu objects", trackingObjects_.size());
            return true;
        }
        
        void Stop() {
            if (!running_) {
                return;
            }
            
            running_ = false;
            
            if (trackingThread_.joinable()) {
                trackingThread_.join();
            }
            
            // Clean up trackers
            for (auto tracker : trackers_) {
                delete tracker;
            }
            trackers_.clear();
            
            // Clean up user data
            for (auto data : userDataList_) {
                delete data;
            }
            userDataList_.clear();
            
            // Clean up connection
            if (connection_) {
                connection_->removeReference();
                connection_ = nullptr;
            }
            
            logger.info("VRPN tracking stopped");
        }
        
        PositionData GetPositionData(size_t objectId) const {
            std::lock_guard<std::mutex> lock(mutex_);
            
            if (objectId < positionData_.size()) {
                return positionData_[objectId];
            }
            
            // Return invalid data if objectId is out of range
            PositionData emptyData;
            memset(&emptyData, 0, sizeof(emptyData));
            emptyData.magic = TENSION_POSITION_MAGIC;
            emptyData.objectId = objectId;
            emptyData.tracked = false;
            
            return emptyData;
        }
        
        bool IsTracked(size_t objectId, uint64_t maxAgeMs = 500) const {
            std::lock_guard<std::mutex> lock(mutex_);
            
            if (objectId < positionData_.size()) {
                if (!positionData_[objectId].tracked) {
                    return false;
                }
                
                // Check if position data is fresh enough
                uint64_t currentTime = GetCurrentTimeMs();
                uint64_t dataAge = currentTime - positionData_[objectId].timestamp;
                
                return (dataAge <= maxAgeMs);
            }
            
            return false;
        }
        
        std::vector<PositionData> GetAllPositionData() const {
            std::lock_guard<std::mutex> lock(mutex_);
            return positionData_;
        }
        
        size_t GetNumObjects() const {
            return trackingObjects_.size();
        }
    
    private:
        // Structure to pass to VRPN callbacks
        struct tracker_data_t {
            VrpnTracker* tracker;
            size_t objectId;
        };
        
        // Callback for position updates
        static void HandleTrackerPosition(void* userData, const vrpn_TRACKERCB trackData) {
            tracker_data_t* data = static_cast<tracker_data_t*>(userData);
            VrpnTracker* self = data->tracker;
            size_t objectId = data->objectId;
            
            uint64_t currentTime = GetCurrentTimeMs();
            
            std::lock_guard<std::mutex> lock(self->mutex_);
            
            PositionData& posData = self->positionData_[objectId];
            posData.magic = TENSION_POSITION_MAGIC;
            posData.objectId = objectId;
            posData.timestamp = currentTime;
            posData.position[0] = trackData.pos[0];
            posData.position[1] = trackData.pos[1];
            posData.position[2] = trackData.pos[2];
            posData.orientation[0] = trackData.quat[0];
            posData.orientation[1] = trackData.quat[1];
            posData.orientation[2] = trackData.quat[2];
            posData.orientation[3] = trackData.quat[3];
            posData.tracked = true;
        }
        
        // Callback for velocity updates
        static void HandleTrackerVelocity(void* userData, const vrpn_TRACKERVELCB velocityData) {
            tracker_data_t* data = static_cast<tracker_data_t*>(userData);
            VrpnTracker* self = data->tracker;
            size_t objectId = data->objectId;
            
            std::lock_guard<std::mutex> lock(self->mutex_);
            
            PositionData& posData = self->positionData_[objectId];
            posData.velocity[0] = velocityData.vel[0];
            posData.velocity[1] = velocityData.vel[1];
            posData.velocity[2] = velocityData.vel[2];
        }
        
        void TrackingLoop() {
            while (running_) {
                // Process VRPN messages (non-blocking)
                if (connection_) {
                    connection_->mainloop();
                    
                    // Process each tracker
                    for (auto tracker : trackers_) {
                        if (tracker) {
                            tracker->mainloop();
                        }
                    }
                }
                
                // Sleep to avoid high CPU usage
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        
        // Use the global GetCurrentTimeMs function
        
        std::atomic<bool> running_;
        std::string host_;
        int port_;
        
        std::vector<std::string> trackingObjects_;
        std::vector<vrpn_Tracker_Remote*> trackers_;
        std::vector<tracker_data_t*> userDataList_;
        vrpn_Connection* connection_ = nullptr;
        
        std::thread trackingThread_;
        mutable std::mutex mutex_;
        std::vector<PositionData> positionData_;
    };

/**
 * Client record for TCP connections
 */
struct TcpClientRecord {
    Socket socket;
    struct sockaddr_in address;
    uint32_t droneId;
    uint64_t lastActivityTime;
    bool authorized;
    bool connected;
    
    TcpClientRecord(int sock, const struct sockaddr_in& addr)
        : socket(sock), address(addr), droneId(0), lastActivityTime(0), authorized(false), connected(true)
    {}
};

/**
 * Request record
 */
struct RequestRecord {
    uint32_t sequenceNumber;
    uint32_t droneId;
    uint64_t timestamp;
    
    RequestRecord(uint32_t seq, uint32_t drone, uint64_t time)
        : sequenceNumber(seq), droneId(drone), timestamp(time) {}
};

/**
 * Task queue with thread pool
 */
class TaskQueue {
public:
    TaskQueue(size_t numThreads) : running_(true) {
        for (size_t i = 0; i < numThreads; ++i) {
            workers_.emplace_back([this] {
                while (running_) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(mutex_);
                        condition_.wait(lock, [this] { 
                            return !running_ || !tasks_.empty(); 
                        });
                        
                        if (!running_ && tasks_.empty()) {
                            return;
                        }
                        
                        if (!tasks_.empty()) {
                            task = std::move(tasks_.front());
                            tasks_.pop();
                        }
                    }
                    
                    if (task) {
                        task();
                    }
                }
            });
        }
    }
    
    ~TaskQueue() {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            running_ = false;
        }
        
        condition_.notify_all();
        
        for (auto& worker : workers_) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }
    
    template<typename F, typename... Args>
    auto enqueue(F&& f, Args&&... args) -> std::future<typename std::result_of<F(Args...)>::type> {
        using return_type = typename std::result_of<F(Args...)>::type;
        
        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );
        
        std::future<return_type> res = task->get_future();
        
        {
            std::unique_lock<std::mutex> lock(mutex_);
            
            if (!running_) {
                throw std::runtime_error("Cannot enqueue on stopped TaskQueue");
            }
            
            tasks_.emplace([task]() { (*task)(); });
        }
        
        condition_.notify_one();
        return res;
    }
    
    size_t size() const {
        std::unique_lock<std::mutex> lock(mutex_);
        return tasks_.size();
    }
    
private:
    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::atomic<bool> running_;
};

/**
 * Main server class - Three-Phase Optimization Architecture
 */
class MAATSTensionOptimizer {
public:
    struct Config {
        // Network configuration
        std::string bindAddress;       // Address to bind the server to
        int tcpPort;                   // Port for TCP connections
        int udpPort;                   // Port for UDP unicast messages
        std::vector<std::string> droneAddresses; // IP addresses for each drone
        
        // VRPN configuration
        std::string vrpnHost;          // VRPN server host
        int vrpnPort;                  // VRPN server port
        int numDrones;                 // Number of drones to track
        
        // System configuration
        int threadPoolSize;            // Thread pool size
        Logger::Level logLevel;        // Logging level
        double optimizerMu;            // Mu parameter for optimization (cable separation)
        double directionBias;          // Direction persistence weight
        double maxDirectionRateRad;    // Maximum direction change rate (radians/sec)
        bool useVrpnPositions;         // Whether to use VRPN positions for optimization
        int tcpTimeout;                // TCP connection timeout (milliseconds)
        int optimizationTimeoutMs;     // Timeout for optimization (milliseconds)
        int udpRetries;                // Max retries for UDP transmission
        int udpRetryIntervalMs;        // Interval between UDP retries (milliseconds)
        int optimizationRateHz;        // Rate limit for optimization (max Hz)
        
        Config() 
            : bindAddress("0.0.0.0")
            , tcpPort(5555)
            , udpPort(5556)
            , vrpnHost("127.0.0.1")  // Simulator VRPN server
            , vrpnPort(3883)
            , numDrones(3)           // Default to 3 drones (optimized version)
            , threadPoolSize(4)
            , logLevel(Logger::INFO)
            , optimizerMu(5.0)
            , directionBias(0.1)
            , maxDirectionRateRad(0.35)
            , useVrpnPositions(true)
            , tcpTimeout(10000)
            , optimizationTimeoutMs(5000)
            , udpRetries(3)
            , udpRetryIntervalMs(5)
            , optimizationRateHz(60) // 60 Hz for 2 drones, 40 Hz for 3 drones
        {}
    };
    
    MAATSTensionOptimizer(const Config& config = Config())
        : config_(config)
        , running_(false)
        , taskQueue_(config.threadPoolSize)
        , requestCount_(0)
        , successCount_(0)
        , failureCount_(0)
        , vrpnTracker_(nullptr)
        , messagesSent_(0)
        , lastStatsTime_(0)
        , tcpServerSocket_(-1)
        , udpSocket_(-1)
        , clientChangeSignal_(false)
        , hasValidHistory_(false)
        , lastOptimizationTime_(0)
    {
        // Set logger level from config
        logger.setLevel(config.logLevel);
        
        logger.info("MAATSTensionOptimizer Three-Phase v%d.%d.%d initializing...",
                    SERVER_VERSION_MAJOR, SERVER_VERSION_MINOR, SERVER_VERSION_PATCH);
        
        // Generate default drone addresses if none provided
        if (config_.droneAddresses.empty()) {
            for (int i = 0; i < config_.numDrones; i++) {
                config_.droneAddresses.push_back("127.0.0.1"); // All drones on localhost for simulation
            }
        }
        
        // Ensure we have at least numDrones addresses
        while (config_.droneAddresses.size() < (size_t)config_.numDrones) {
            config_.droneAddresses.push_back("127.0.0.1"); // Fallback to master drone
        }
        
        // Initialize direction history and commanded directions
        rawDirections_.resize(config_.numDrones);
        commandedDirections_.resize(config_.numDrones);
        lastTensions_.resize(config_.numDrones, 0.0);
        
        // Create default directions for initialization
        for (int i = 0; i < config_.numDrones; i++) {
            double angle = 2.0 * M_PI * i / config_.numDrones;
            double thetaRad = 20.0 * M_PI / 180.0;
            
            Vector3D direction(
                -sin(thetaRad) * cos(angle),
                -sin(thetaRad) * sin(angle),
                cos(thetaRad)
            );
            
            // Normalize
            direction.normalize();
            rawDirections_[i] = direction;
            commandedDirections_[i] = direction;
        }
        
        // Adjust optimization rate based on number of drones
        if (config_.numDrones <= 2) {
            config_.optimizationRateHz = 60;
        } else if (config_.numDrones == 3) {
            config_.optimizationRateHz = 40;
        } else {
            config_.optimizationRateHz = 10;
        }
        
        // Log configuration
        logger.info("=====================================================");
        logger.info("THREE-PHASE CONFIGURATION");
        logger.info("Number of drones: %d", config_.numDrones);
        logger.info("Direction bias: %.2f", config_.directionBias);
        logger.info("Max direction rate: %.2f rad/s (~%.1f deg/s)", 
                config_.maxDirectionRateRad, config_.maxDirectionRateRad * 180.0 / M_PI);
        logger.info("Optimization rate: %d Hz", config_.optimizationRateHz);
        for (size_t i = 0; i < config_.droneAddresses.size(); i++) {
            logger.info("  Drone %zu: %s:%d", i, config_.droneAddresses[i].c_str(), config_.udpPort + i);
        }
        logger.info("=====================================================");
    }
    
    bool Start() {
        if (running_) {
            logger.warn("Server already running");
            return true;
        }
        
        // Initialize TCP socket
        if (!InitializeTcpSocket()) {
            return false;
        }
        
        // Initialize UDP socket for unicast
        if (!InitializeUdpSocket()) {
            CloseTcpSocket();
            return false;
        }
        
        // Initialize VRPN tracking
        if (config_.useVrpnPositions) {
            std::vector<std::string> trackingObjects;
            
            // Create object names
            for (int i = 0; i < config_.numDrones; i++) {
                std::stringstream ss;
                ss << "Drone_" << i;
                trackingObjects.push_back(ss.str());
            }
            
            // Create VrpnTracker
            vrpnTracker_ = std::unique_ptr<VrpnTracker>(new VrpnTracker(
                config_.vrpnHost, config_.vrpnPort, trackingObjects));
                
            if (!vrpnTracker_->Start()) {
                logger.error("Failed to start VRPN tracking");
                CloseUdpSocket();
                CloseTcpSocket();
                return false;
            }
        }
        
        // Start server threads
        running_ = true;
        
        // Start TCP thread
        tcpThread_ = std::thread(&MAATSTensionOptimizer::TcpServerLoop, this);
        
        // Start monitoring thread
        monitorThread_ = std::thread(&MAATSTensionOptimizer::MonitorLoop, this);
        
        logger.info("MAATSTensionOptimizer started (TCP: %s:%d, UDP: unicast to %d drones)",
                    config_.bindAddress.c_str(), config_.tcpPort, config_.numDrones);
        
        return true;
    }
    
    void Stop() {
        if (!running_) {
            return;
        }
        
        running_ = false;
        
        // Signal client change to wake up select
        {
            std::lock_guard<std::mutex> lock(clientChangeMutex_);
            clientChangeSignal_ = true;
        }
        clientChangeCondition_.notify_one();
        
        // Close TCP connections
        {
            std::lock_guard<std::mutex> lock(clientsMutex_);
            tcpClients_.clear();
        }
        
        // Close server sockets
        CloseTcpSocket();
        CloseUdpSocket();
        
        // Stop VRPN tracking
        if (vrpnTracker_) {
            vrpnTracker_->Stop();
        }
        
        // Wait for threads to finish
        if (tcpThread_.joinable()) {
            tcpThread_.join();
        }
        
        if (monitorThread_.joinable()) {
            monitorThread_.join();
        }
        
        logger.info("MAATSTensionOptimizer stopped");
    }
    
    void SetLogLevel(Logger::Level level) {
        logger.setLevel(level);
    }
    
    // Test unicast message to a specific drone
    bool TestUnicastMessage(int droneIndex) {
        if (droneIndex < 0 || droneIndex >= config_.numDrones) {
            logger.error("Invalid drone index for test: %d", droneIndex);
            return false;
        }
        
        logger.warn("==== TESTING UNICAST MESSAGE TO DRONE %d ====", droneIndex);
        
        // Create test message
        Vector3D testDir(0, 0, 1);
        double testTension = 1.0;
        
        // Send test message
        bool result = SendUnicastMessage(0xFFFFFFFF, droneIndex, testDir, testTension);
        
        if (result) {
            logger.warn("Test message to drone %d SUCCEEDED", droneIndex);
        } else {
            logger.error("Test message to drone %d FAILED", droneIndex);
        }
        
        return result;
    }
    
    // Test unicast to all drones
    bool TestAllDrones() {
        logger.warn("==== TESTING UNICAST TO ALL DRONES ====");
        
        bool allSuccess = true;
        for (int i = 0; i < config_.numDrones; i++) {
            if (!TestUnicastMessage(i)) {
                allSuccess = false;
            }
        }
        
        logger.warn("==== UNICAST TEST %s ====", allSuccess ? "SUCCEEDED" : "FAILED");
        return allSuccess;
    }

private:
    bool InitializeTcpSocket() {
        // Close existing socket if any
        CloseTcpSocket();
        
        // Create socket
        tcpServerSocket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (tcpServerSocket_ < 0) {
            logger.error("Failed to create TCP socket: %s", strerror(errno));
            return false;
        }
        
        // Set socket options
        int opt = 1;
        if (setsockopt(tcpServerSocket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            logger.error("Failed to set SO_REUSEADDR on TCP socket: %s", strerror(errno));
            close(tcpServerSocket_);
            tcpServerSocket_ = -1;
            return false;
        }
        
        // Bind socket to address and port
        struct sockaddr_in serverAddr;
        memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(config_.tcpPort);
        
        if (config_.bindAddress == "0.0.0.0" || config_.bindAddress == "any") {
            serverAddr.sin_addr.s_addr = INADDR_ANY;
        } else {
            if (inet_pton(AF_INET, config_.bindAddress.c_str(), &serverAddr.sin_addr) <= 0) {
                logger.error("Invalid TCP bind address: %s", config_.bindAddress.c_str());
                close(tcpServerSocket_);
                tcpServerSocket_ = -1;
                return false;
            }
        }
        
        if (bind(tcpServerSocket_, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
            logger.error("Failed to bind TCP socket: %s", strerror(errno));
            close(tcpServerSocket_);
            tcpServerSocket_ = -1;
            return false;
        }
        
        // Listen for connections
        if (listen(tcpServerSocket_, 10) < 0) {
            logger.error("Failed to listen on TCP socket: %s", strerror(errno));
            close(tcpServerSocket_);
            tcpServerSocket_ = -1;
            return false;
        }
        
        // Set the server socket to non-blocking mode
        int flags = fcntl(tcpServerSocket_, F_GETFL, 0);
        fcntl(tcpServerSocket_, F_SETFL, flags | O_NONBLOCK);
        
        logger.info("TCP server initialized on port %d", config_.tcpPort);
        return true;
    }
    
    void CloseTcpSocket() {
        if (tcpServerSocket_ >= 0) {
            close(tcpServerSocket_);
            tcpServerSocket_ = -1;
        }
    }
    
    bool InitializeUdpSocket() {
        // Close existing socket if any
        CloseUdpSocket();
        
        logger.info("Initializing UDP socket for unicast...");
        
        // Create socket
        udpSocket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udpSocket_ < 0) {
            logger.error("Failed to create UDP socket: %s", strerror(errno));
            return false;
        }
        
        int opt = 1;
        if (setsockopt(udpSocket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            logger.error("Failed to set SO_REUSEADDR on UDP socket: %s", strerror(errno));
            close(udpSocket_);
            udpSocket_ = -1;
            return false;
        }
        
        // Test connectivity
        if (!config_.droneAddresses.empty()) {
            struct sockaddr_in test_addr;
            memset(&test_addr, 0, sizeof(test_addr));
            test_addr.sin_family = AF_INET;
            // Use calculatedPort for drone 0
            int testCalculatedPort = config_.udpPort + 0; // Drone 0
            test_addr.sin_port = htons(testCalculatedPort);
            
            if (inet_pton(AF_INET, config_.droneAddresses[0].c_str(), &test_addr.sin_addr) <= 0) {
                logger.warn("Invalid drone address: %s", config_.droneAddresses[0].c_str());
            } else {
                // Test socket functionality with a dummy send
                char test_msg[] = "UDP_TEST";
                ssize_t test_result = sendto(udpSocket_, test_msg, sizeof(test_msg), 0,
                                (struct sockaddr*)&test_addr, sizeof(test_addr));
                                
                if (test_result < 0) {
                    logger.warn("UDP test send failed: %s", strerror(errno));
                    logger.warn("This may indicate a network configuration issue");
                } else {
                    logger.info("UDP test send successful (%zd bytes sent to %s:%d)", 
                            test_result, config_.droneAddresses[0].c_str(), testCalculatedPort);
                }
            }
        }
        
        logger.info("UDP socket initialized for unicast communication");
        return true;
    }
    
    void CloseUdpSocket() {
        if (udpSocket_ >= 0) {
            close(udpSocket_);
            udpSocket_ = -1;
        }
    }
    
    void TcpServerLoop() {
        fd_set master_set, read_fds;
        struct timeval tv;
        
        FD_ZERO(&master_set);
        FD_SET(tcpServerSocket_, &master_set);
        int max_fd = tcpServerSocket_;
        
        logger.info("TCP server loop started");
        
        // Create a pipe for waking up select
        int pipefds[2];
        if (pipe(pipefds) == -1) {
            logger.error("Failed to create pipe: %s", strerror(errno));
            return;
        }
        
        int pipeReadFd = pipefds[0];
        int pipeWriteFd = pipefds[1];
        
        // Set pipe to non-blocking mode
        int flags = fcntl(pipeReadFd, F_GETFL, 0);
        fcntl(pipeReadFd, F_SETFL, flags | O_NONBLOCK);
        
        flags = fcntl(pipeWriteFd, F_GETFL, 0);
        fcntl(pipeWriteFd, F_SETFL, flags | O_NONBLOCK);
        
        // Add pipe read end to master set
        FD_SET(pipeReadFd, &master_set);
        if (pipeReadFd > max_fd) {
            max_fd = pipeReadFd;
        }
        
        // Start client change monitor thread
        std::thread changeMonitorThread([this, pipeWriteFd]() {
            while (running_) {
                std::unique_lock<std::mutex> lock(clientChangeMutex_);
                clientChangeCondition_.wait(lock, [this]() { 
                    return clientChangeSignal_ || !running_; 
                });
                
                if (!running_) break;
                
                // Signal main thread by writing to pipe
                if (clientChangeSignal_) {
                    char c = 1;
                    if (write(pipeWriteFd, &c, 1) < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                        logger.error("Failed to write to pipe: %s", strerror(errno));
                    }
                    clientChangeSignal_ = false;
                }
            }
        });
        
        while (running_) {
            // Update the master_set
            std::vector<TcpClientRecord*> clients;
            {
                std::lock_guard<std::mutex> lock(clientsMutex_);
                for (auto& client : tcpClients_) {
                    clients.push_back(&client);
                }
            }
            
            // Update master_set with current client sockets
            FD_ZERO(&master_set);
            FD_SET(tcpServerSocket_, &master_set);
            FD_SET(pipeReadFd, &master_set);
            max_fd = std::max(tcpServerSocket_, pipeReadFd);
            
            for (auto client : clients) {
                if (client->connected && client->socket.valid()) {
                    int fd = client->socket.get();
                    FD_SET(fd, &master_set);
                    if (fd > max_fd) {
                        max_fd = fd;
                    }
                }
            }
            
            // Copy the master set
            read_fds = master_set;
            
            // Set timeout for select
            tv.tv_sec = 0;
            tv.tv_usec = 200000;
            
            // Wait for activity
            int activity = select(max_fd + 1, &read_fds, NULL, NULL, &tv);
            
            if (activity < 0) {
                if (errno == EINTR) {
                    continue;  // Interrupted system call, just retry
                }
                logger.error("Select error: %s", strerror(errno));
                std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Avoid tight loop on errors
                continue;
            }
            
            // Check if there's activity on the pipe
            if (FD_ISSET(pipeReadFd, &read_fds)) {
                // Clear the pipe data
                char buffer[256];
                while (read(pipeReadFd, buffer, sizeof(buffer)) > 0) {
                    // Drain the pipe
                }
            }
            
            // Check for new connections
            if (activity > 0 && FD_ISSET(tcpServerSocket_, &read_fds)) {
                struct sockaddr_in clientAddr;
                socklen_t addrLen = sizeof(clientAddr);
                
                int clientSocket = accept(tcpServerSocket_, (struct sockaddr*)&clientAddr, &addrLen);
                
                if (clientSocket < 0) {
                    if (errno != EAGAIN && errno != EWOULDBLOCK) {
                        logger.error("Accept error: %s", strerror(errno));
                    }
                } else {
                    // Log the new connection
                    char clientIp[INET_ADDRSTRLEN];
                    inet_ntop(AF_INET, &clientAddr.sin_addr, clientIp, sizeof(clientIp));
                    int clientPort = ntohs(clientAddr.sin_port);
                    
                    logger.info("New TCP connection from %s:%d", clientIp, clientPort);
                    
                    // Set non-blocking mode
                    int flags = fcntl(clientSocket, F_GETFL, 0);
                    fcntl(clientSocket, F_SETFL, flags | O_NONBLOCK);
                    
                    // Add to client list
                    {
                        std::lock_guard<std::mutex> lock(clientsMutex_);
                        tcpClients_.emplace_back(clientSocket, clientAddr);
                        
                        // Set last activity time
                        tcpClients_.back().lastActivityTime = GetCurrentTimeMs();
                    }
                    
                    // Signal client change
                    {
                        std::lock_guard<std::mutex> lock(clientChangeMutex_);
                        clientChangeSignal_ = true;
                    }
                    clientChangeCondition_.notify_one();
                }
            }
            
            // Process activity on client sockets
            for (auto client : clients) {
                if (!client->connected || !client->socket.valid()) {
                    continue;
                }
                
                int clientSocket = client->socket.get();
                
                if (activity > 0 && FD_ISSET(clientSocket, &read_fds)) {
                    if (!HandleTcpClientData(clientSocket, *client)) {
                        // Handle connection close
                        logger.info("Client disconnected");
                        
                        // Mark client as disconnected and signal change
                        {
                            std::lock_guard<std::mutex> lock(clientsMutex_);
                            client->connected = false;
                        }
                        
                        {
                            std::lock_guard<std::mutex> lock(clientChangeMutex_);
                            clientChangeSignal_ = true;
                        }
                        clientChangeCondition_.notify_one();
                    } else {
                        // Update activity time
                        client->lastActivityTime = GetCurrentTimeMs();
                    }
                }
            }
            
            // Clean up disconnected clients
            {
                std::lock_guard<std::mutex> lock(clientsMutex_);
                
                // Remove clients that are not connected
                for (auto it = tcpClients_.begin(); it != tcpClients_.end(); ) {
                    if (!it->connected) {
                        it = tcpClients_.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
        }
        
        // Clean up pipe
        close(pipeReadFd);
        close(pipeWriteFd);
        
        // Join change monitor thread
        if (changeMonitorThread.joinable()) {
            changeMonitorThread.join();
        }
        
        logger.info("TCP server loop stopped");
    }
    
    bool HandleTcpClientData(int clientSocket, TcpClientRecord& client) {
        char buffer[512]; // Buffer for message
        
        // Try to receive data
        ssize_t bytesRead = recv(clientSocket, buffer, sizeof(buffer), 0);
        
        if (bytesRead <= 0) {
            if (bytesRead == 0) {
                // Connection closed by client
                char clientIp[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &client.address.sin_addr, clientIp, sizeof(clientIp));
                logger.info("TCP client disconnected: %s:%d", 
                            clientIp, ntohs(client.address.sin_port));
                return false;
            } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
                // Error
                logger.error("TCP receive error: %s", strerror(errno));
                return false;
            }
            return true;
        }
        
        // Message magic number check and dispatch
        if (bytesRead >= 4) {
            uint32_t magic = *((uint32_t*)buffer);
            
            switch (magic) {
                case TENSION_REQUEST_MAGIC:
                    if (static_cast<size_t>(bytesRead) >= sizeof(TcpRequestMessage)) {
                        HandleTensionRequestMessage(clientSocket, client, 
                            *((TcpRequestMessage*)buffer));
                    }
                    break;
                    
                case TENSION_HEARTBEAT_MAGIC:
                    // Record the activity
                    logger.debug("Heartbeat from client %d", client.droneId);
                    break;
                    
                default:
                    // Unknown message type
                    logger.warn("Unknown TCP message magic: 0x%08X", magic);
                    break;
            }
        }
        
        return true;
    }
    
    void HandleTensionRequestMessage(int clientSocket, TcpClientRecord& client, 
                                    const TcpRequestMessage& request) {
        // Validate message
        if (request.magic != TENSION_REQUEST_MAGIC) {
            logger.warn("Invalid magic number in tension request message");
            return;
        }
        
        // Check if this request is a duplicate
        if (IsDuplicateRequest(request.sequenceNumber, request.droneId, request.timestamp)) {
            logger.debug("Duplicate request from drone %d (seq=%u)", 
                        request.droneId, request.sequenceNumber);
            return;
        }
        
        // Save drone ID
        client.droneId = request.droneId;
        
        // Log the request
        char clientIp[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client.address.sin_addr, clientIp, sizeof(clientIp));
        
        logger.info("Received tension request from %s:%d (seq=%u, drone=%u)",
                    clientIp, ntohs(client.address.sin_port),
                    request.sequenceNumber, request.droneId);
        
        // Log force vector for debugging
        logger.debug("Request details - resultant force: [%.3f, %.3f, %.3f]",
                    request.resultantForce[0], request.resultantForce[1], 
                    request.resultantForce[2]);
        
        // Get current time for rate limiting
        uint64_t currentTime = GetCurrentTimeMs();
        
        // Log request to CSV file
        csvLogger.logRequest(request, currentTime);
        uint64_t timeSinceLastOpt = currentTime - lastOptimizationTime_;
        uint64_t minInterval = 1000 / config_.optimizationRateHz; // Minimum ms between optimizations
        
        if (timeSinceLastOpt < minInterval) {
            logger.debug("Rate limiting optimization (last was %llu ms ago, min interval %llu ms)",
                    (unsigned long long)timeSinceLastOpt, (unsigned long long)minInterval);
            
            // Use last solution
            if (hasValidHistory_) {
                logger.debug("Using previous solution due to rate limiting");
                SendLastSolution(request.sequenceNumber);
                return;
            }
        }
        
        // Update optimization timestamp
        lastOptimizationTime_ = currentTime;
        
        // Increment request counter
        requestCount_++;
        
        // Process the request in the thread pool
        auto future = taskQueue_.enqueue(
            [this, request]() {
                return ProcessTensionRequest(request);
            }
        );
        
        // Wait for optimization
        std::future_status status = future.wait_for(std::chrono::milliseconds(config_.optimizationTimeoutMs));
        
        if (status == std::future_status::timeout) {
            logger.error("Optimization timed out after %d ms (seq=%u)",
                        config_.optimizationTimeoutMs, request.sequenceNumber);
            
            // Use previous solution or fallback
            if (hasValidHistory_) {
                SendLastSolution(request.sequenceNumber);
            } else {
                // Create fallback result with symmetric distribution
                if (!SendFallbackSymmetricDistribution(request.sequenceNumber)) {
                    logger.error("Failed to send fallback distribution");
                }
            }
            
            // Update failure counter
            failureCount_++;
        }
    }
    
    bool ProcessTensionRequest(const TcpRequestMessage& request) {
        try {
            // Get number of drones from config
            int numDrones = config_.numDrones;
            
            // Create vector for the resultant force
            Vector3D resultantForce(
                request.resultantForce[0],
                request.resultantForce[1],
                request.resultantForce[2]
            );
            
            // Save for verification
            {
                std::lock_guard<std::mutex> lock(solutionHistoryMutex_);
                lastResultantForce_ = resultantForce;
            }
            
            // Get drone positions from VRPN if available
            std::vector<Vector3D> initialDirections;
            bool vrpnPositionsAvailable = false;
            
            if (config_.useVrpnPositions && vrpnTracker_) {
                vrpnPositionsAvailable = GetVrpnDirections(initialDirections);
            }
            
            if (!vrpnPositionsAvailable) {
                // Use default directions based on symmetric distribution
                for (int i = 0; i < numDrones; i++) {
                    double angle = 2.0 * M_PI * i / numDrones;
                    double thetaRad = 20.0 * M_PI / 180.0;
                    
                    Vector3D direction(
                        -sin(thetaRad) * cos(angle),
                        -sin(thetaRad) * sin(angle),
                        cos(thetaRad)
                    );
                    
                    // Normalize
                    direction.normalize();
                    initialDirections.push_back(direction);
                }
            }
            
            // Get previous solution for continuity
            std::vector<Vector3D> prevDirections;
            
            {
                std::lock_guard<std::mutex> lock(solutionHistoryMutex_);
                if (hasValidHistory_) {
                    prevDirections = commandedDirections_;
                } else {
                    // If no history, use initial directions
                    prevDirections = initialDirections;
                }
            }
            
            // Phase 1: Primary optimization with SNOPT
            // Prepare optimization input data with simplified approach
            TensionOptimizer::InputData optimInput;
            optimInput.resultantForce = resultantForce;
            optimInput.numCables = numDrones;
            optimInput.initialDirections = initialDirections;
            optimInput.prevDirections = prevDirections;
            optimInput.mu = config_.optimizerMu;
            optimInput.directionBias = config_.directionBias;
            
            // Log optimization details
            logger.info("Starting optimization (seq=%u, drones=%d, force=[%.2f, %.2f, %.2f])",
                    request.sequenceNumber, numDrones,
                    resultantForce.x, resultantForce.y, resultantForce.z);
                    
            // Create optimizer and run optimization
            TensionOptimizer optimizer;
            TensionOptimizer::OutputData optimResult = optimizer.optimize(optimInput);
            
            if (optimResult.success) {
                // Log success
                logger.info("Optimization succeeded (seq=%u), objective=%.6f, time=%.2fms",
                        request.sequenceNumber, optimResult.objectiveValue, optimResult.solveTimeMs);
                
                // Update success counter
                successCount_++;
                
                // Phase 2: Post-processing to enforce rate limiting 
                std::vector<Vector3D> limitedDirections = optimResult.directions;
                std::vector<bool> directionsWereModified;
                
                // Get previous directions for rate limiting
                std::vector<Vector3D> prevDirections;
                {
                    std::lock_guard<std::mutex> lock(solutionHistoryMutex_);
                    if (hasValidHistory_) {
                        prevDirections = commandedDirections_;
                    }
                }
                
                // Apply rate limiting and track which directions were modified
                bool anyDirectionModified = ProcessRateLimitedDirections(
                    limitedDirections,
                    directionsWereModified,
                    prevDirections,
                    config_.maxDirectionRateRad
                );
                
                // Start with the original optimized tensions
                std::vector<double> adjustedTensions = optimResult.tensions;
                
                // Phase 3: If any directions were rate-limited, re-optimize tensions
                bool preservedEquilibrium = true;
                if (anyDirectionModified) {
                    logger.info("Re-optimizing tensions after direction rate limiting");
                    
                    // Use the new analytical tension-only optimization for 2-3 drones
                    preservedEquilibrium = OptimizeTensionsOnly(
                        adjustedTensions,
                        limitedDirections,
                        resultantForce
                    );
                    
                    if (!preservedEquilibrium) {
                        logger.warn("Could not maintain force equilibrium after re-optimizing tensions. Using raw optimization result.");
                        // Fall back to raw optimization result
                        limitedDirections = optimResult.directions;
                        adjustedTensions = optimResult.tensions;
                    }
                }
                
                // Check for unsafe configurations before sending
                bool configurationSafe = true;
                double maxDotProduct = 0.0;
                int drone1 = -1, drone2 = -1;
                
                // Calculate dot products between all direction vectors
                for (int i = 0; i < numDrones && i < (int)limitedDirections.size() && configurationSafe; i++) {
                    for (int j = i+1; j < numDrones && j < (int)limitedDirections.size(); j++) {
                        // Calculate dot product between direction vectors
                        double dotProduct = 
                            limitedDirections[i].dot(limitedDirections[j]);
                        
                        // Keep track of maximum dot product for logging
                        if (dotProduct > maxDotProduct) {
                            maxDotProduct = dotProduct;
                            drone1 = i;
                            drone2 = j;
                        }
                        
                        // Check if dot product exceeds safety threshold (0.8 ≈ 35 degrees)
                        if (dotProduct > 0.8) {
                            logger.warn("Unsafe cable configuration detected! (seq=%u): "
                                        "Dot product between drones %d and %d is %.4f (exceeds 0.8)",
                                        request.sequenceNumber, i, j, dotProduct);
                            configurationSafe = false;
                            break;
                        }
                    }
                }
                
                // Log the maximum dot product for debugging
                if (drone1 >= 0 && drone2 >= 0) {
                    logger.debug("Maximum dot product (seq=%u): %.4f between drones %d and %d",
                                request.sequenceNumber, maxDotProduct, drone1, drone2);
                }
                
                // Store the processed solution
                {
                    std::lock_guard<std::mutex> lock(solutionHistoryMutex_);
                    rawDirections_ = optimResult.directions;  // Save raw directions
                    commandedDirections_ = limitedDirections; // Save rate-limited directions
                    lastTensions_ = adjustedTensions;         // Save adjusted tensions
                    lastResultantForce_ = resultantForce;
                    lastSequence_ = request.sequenceNumber;
                    hasValidHistory_ = true;
                }
                
                // Send configuration to drones if safe, otherwise use symmetric distribution
                if (configurationSafe) {
                    // Send unicast messages to each drone
                    bool allSent = true;
                    for (int i = 0; i < numDrones && i < (int)limitedDirections.size(); i++) {
                        if (!SendUnicastMessage(request.sequenceNumber, i, 
                                            limitedDirections[i],
                                            adjustedTensions[i])) {
                            logger.error("Failed to send unicast to drone %d", i);
                            allSent = false;
                        }
                    }
                    
                    return allSent;
                } else {
                    // Configuration unsafe
                    logger.warn("Using symmetric distribution due to unsafe configuration (seq=%u)",
                            request.sequenceNumber);
                    return SendFallbackSymmetricDistribution(request.sequenceNumber);
                }
            } else {
                // Log failure
                logger.error("Optimization failed (seq=%u): %s",
                        request.sequenceNumber, optimResult.errorMessage.c_str());
                
                // Update failure counter
                failureCount_++;
                
                // Try to use previous solution or fallback to symmetric distribution
                if (hasValidHistory_) {
                    return SendLastSolution(request.sequenceNumber);
                } else {
                    return SendFallbackSymmetricDistribution(request.sequenceNumber);
                }
            }
        }
        catch (const std::exception& e) {
            logger.error("Exception in ProcessTensionRequest: %s", e.what());
            failureCount_++;
            
            if (hasValidHistory_) {
                return SendLastSolution(request.sequenceNumber);
            } else {
                return SendFallbackSymmetricDistribution(request.sequenceNumber);
            }
        }
    }
    
    bool SendLastSolution(uint32_t sequenceNumber) {
        logger.info("Sending last valid solution (seq=%u)", sequenceNumber);
        
        std::vector<Vector3D> directions;
        std::vector<double> tensions;
        
        {
            std::lock_guard<std::mutex> lock(solutionHistoryMutex_);
            directions = commandedDirections_;
            tensions = lastTensions_;
        }
        
        if (directions.empty() || tensions.empty()) {
            logger.error("No valid history to send");
            return SendFallbackSymmetricDistribution(sequenceNumber);
        }
        
        // Send to all drones
        bool allSent = true;
        for (int i = 0; i < config_.numDrones && i < (int)directions.size(); i++) {
            if (!SendUnicastMessage(sequenceNumber, i, directions[i], tensions[i])) {
                logger.error("Failed to send last solution to drone %d", i);
                allSent = false;
            }
        }
        
        return allSent;
    }
    
    bool SendUnicastMessage(uint32_t sequenceNumber, int droneIndex, 
                        const Vector3D& direction, double tension) {
        if (udpSocket_ < 0) {
            logger.error("Cannot send unicast: UDP socket not open");
            return false;
        }
        
        if (droneIndex < 0 || droneIndex >= config_.numDrones || 
            droneIndex >= (int)config_.droneAddresses.size()) {
            logger.error("Invalid drone index %d (max %d)", 
                    droneIndex, (int)config_.droneAddresses.size() - 1);
            return false;
        }

        UdpUnicastMessage msg;
        memset(&msg, 0, sizeof(msg));
        
        // Set fields directly from inputs
        msg.magic = TENSION_UNICAST_MAGIC;
        msg.sequenceNumber = sequenceNumber;
        msg.droneIndex = droneIndex;
        msg.direction[0] = direction.x;
        msg.direction[1] = direction.y;
        msg.direction[2] = direction.z;
        msg.tension = tension;
        
        // Calculate checksum
        msg.checksum = CalculateChecksum(&msg, sizeof(msg) - sizeof(msg.checksum));
        
        // Set up destination address
        struct sockaddr_in destAddr;
        memset(&destAddr, 0, sizeof(destAddr));
        destAddr.sin_family = AF_INET;
        
        int calculatedPort = config_.udpPort + droneIndex; // Different port for each drone
        destAddr.sin_port = htons(calculatedPort);
        
        if (inet_pton(AF_INET, config_.droneAddresses[droneIndex].c_str(), &destAddr.sin_addr) <= 0) {
            logger.error("Invalid drone address: %s", config_.droneAddresses[droneIndex].c_str());
            return false;
        }
        
        // Try sending
        bool sent = false;
        int retries = 0;
        
        while (!sent && retries <= config_.udpRetries) {
            // Log destination with custom port for each drone
            logger.debug("Sending unicast to drone %d at %s:%d (seq=%u, retry=%d/%d)",
                    droneIndex, config_.droneAddresses[droneIndex].c_str(),
                    calculatedPort, sequenceNumber, retries, config_.udpRetries);
            
            // Send the message
            ssize_t bytesSent = sendto(udpSocket_, &msg, sizeof(msg), 0,
                                    (struct sockaddr*)&destAddr, sizeof(destAddr));
            
            if (bytesSent < 0) {
                logger.error("Failed to send unicast to drone %d: %s", 
                        droneIndex, strerror(errno));
            } else if (static_cast<size_t>(bytesSent) != sizeof(msg)) {
                logger.warn("Partial unicast sent to drone %d: %zd of %zu bytes",
                        droneIndex, bytesSent, sizeof(msg));
            } else {
                // Full message sent
                logger.debug("Unicast sent to drone %d: direction=[%.3f, %.3f, %.3f], tension=%.3f",
                        droneIndex, direction.x, direction.y, direction.z, tension);
                
                // Log response to CSV file
                csvLogger.logResponse(msg, GetCurrentTimeMs());
                
                sent = true;
                messagesSent_++;
                break;
            }
            
            // Retry if needed
            retries++;
            if (retries <= config_.udpRetries) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(config_.udpRetryIntervalMs));
            }
        }
        
        return sent;
    }
    
    bool SendFallbackSymmetricDistribution(uint32_t sequenceNumber) {
        logger.warn("Sending fallback flag to trigger client-side symmetric distribution (seq=%u)", sequenceNumber);
        
        int numDrones = config_.numDrones;
        bool allSent = true;
        
        Vector3D fallbackFlag(0, 0, -999);  // Special flag value (-999 is recognized by clients)
        double fallbackTension = -1;
        
        // Send fallback flag to all drones
        for (int i = 0; i < numDrones; i++) {
            if (!SendUnicastMessage(sequenceNumber, i, fallbackFlag, fallbackTension)) {
                logger.error("Failed to send fallback flag to drone %d", i);
                allSent = false;
            } else {
                logger.info("Sent fallback flag to drone %d - will use client-side symmetric distribution", i);
            }
        }
        
        return allSent;
    }
    
    void MonitorLoop() {
        logger.info("Monitor loop started");
        
        while (running_) {
            // Current time
            uint64_t currentTime = GetCurrentTimeMs();
            
            // Check client timeouts
            CheckClientTimeouts(currentTime);
            
            // Log statistics periodically
            if (currentTime - lastStatsTime_ > 10000) {
                LogStatistics();
                lastStatsTime_ = currentTime;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        
        logger.info("Monitor loop stopped");
    }
    
    void CheckClientTimeouts(uint64_t currentTime) {
        std::lock_guard<std::mutex> lock(clientsMutex_);
        
        // Check each client for timeout
        for (auto it = tcpClients_.begin(); it != tcpClients_.end(); ) {
            if (!it->connected) {
                it = tcpClients_.erase(it);
                continue;
            }
            
            uint64_t inactiveTime = currentTime - it->lastActivityTime;
            
            if (inactiveTime > (uint64_t)config_.tcpTimeout) {
                // Client has timed out
                char clientIp[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &it->address.sin_addr, clientIp, sizeof(clientIp));
                
                logger.info("TCP client timeout: %s:%d (inactive for %llu ms)",
                            clientIp, ntohs(it->address.sin_port),
                            (unsigned long long)inactiveTime);
                
                // Mark as disconnected for removal
                it->connected = false;
                
                // Signal client change
                {
                    std::lock_guard<std::mutex> changeLock(clientChangeMutex_);
                    clientChangeSignal_ = true;
                }
                clientChangeCondition_.notify_one();
                
                ++it;
            } else {
                ++it;
            }
        }
    }
    
    void LogStatistics() {
        // Client count
        size_t numClients;
        {
            std::lock_guard<std::mutex> lock(clientsMutex_);
            numClients = tcpClients_.size();
        }
        
        // Task queue size
        size_t queueSize = taskQueue_.size();
        
        // Log statistics
        logger.info("Stats: clients=%zu, queue=%zu, processed=%llu (success=%llu, fail=%llu), sent=%llu", 
                    numClients, queueSize,
                    (unsigned long long)requestCount_,
                    (unsigned long long)successCount_,
                    (unsigned long long)failureCount_,
                    (unsigned long long)messagesSent_);
        
        // VRPN status if enabled
        if (vrpnTracker_) {
            int trackedCount = 0;
            for (size_t i = 0; i < vrpnTracker_->GetNumObjects(); i++) {
                if (vrpnTracker_->IsTracked(i)) {
                    trackedCount++;
                }
            }
            
            logger.info("VRPN: %d/%zu objects tracked", 
                        trackedCount, vrpnTracker_->GetNumObjects());
        }
        
        // Log current solution state
        {
            std::lock_guard<std::mutex> lock(solutionHistoryMutex_);
            if (hasValidHistory_) {
                logger.info("Current force: [%.3f, %.3f, %.3f]",
                        lastResultantForce_.x, lastResultantForce_.y, lastResultantForce_.z);
                        
                // Calculate actual force from current solution
                Vector3D actualForce;
                for (size_t i = 0; i < commandedDirections_.size() && i < lastTensions_.size(); i++) {
                    actualForce.x += lastTensions_[i] * commandedDirections_[i].x;
                    actualForce.y += lastTensions_[i] * commandedDirections_[i].y;
                    actualForce.z += lastTensions_[i] * commandedDirections_[i].z;
                }
                
                // Calculate error if we have a valid force
                if (lastResultantForce_.norm() > 1e-6) {
                    Vector3D errorVec = actualForce - lastResultantForce_;
                    double errorMag = errorVec.norm();
                    double relError = errorMag / lastResultantForce_.norm();
                    
                    logger.info("Actual force: [%.3f, %.3f, %.3f], error: %.3f N (%.2f%%)",
                            actualForce.x, actualForce.y, actualForce.z,
                            errorMag, relError * 100.0);
                }
            }
        }
    }
    
    bool GetVrpnDirections(std::vector<Vector3D>& directions) {
        if (!vrpnTracker_) {
            return false;
        }
        
        // Get load position
        PositionData loadData = vrpnTracker_->GetPositionData(0); // Load is object 0
        
        if (!loadData.tracked) {
            logger.warn("Load is not tracked, cannot calculate VRPN directions");
            return false;
        }
        
        // Get drone positions
        bool allDronesTracked = true;
        std::vector<PositionData> droneData;
        
        for (size_t i = 1; i < vrpnTracker_->GetNumObjects(); i++) {
            PositionData data = vrpnTracker_->GetPositionData(i);
            droneData.push_back(data);
            
            if (!data.tracked) {
                allDronesTracked = false;
                logger.warn("Drone %zu is not tracked", i - 1);
            }
        }
        
        if (!allDronesTracked) {
            return false;
        }
        
        // Calculate directions from drones to load (unit vectors)
        directions.clear();
        
        logger.debug("============= VRPN DIRECTION VECTORS =============");
        logger.debug("Load position: [%.3f, %.3f, %.3f]",
                    loadData.position[0], loadData.position[1], loadData.position[2]);
        
        for (size_t i = 0; i < droneData.size(); i++) {
            const auto& drone = droneData[i];
            
            // Calculate vector FROM drone TO load
            Vector3D dirVec(
                loadData.position[0] - drone.position[0],
                loadData.position[1] - drone.position[1],
                loadData.position[2] - drone.position[2]
            );
            
            // Normalize to get unit vector
            double norm = dirVec.norm();
            if (norm > 1e-10) {
                dirVec.x /= norm;
                dirVec.y /= norm;
                dirVec.z /= norm;
            } else {
                // Default vertical direction if vectors too close
                dirVec.x = 0.0;
                dirVec.y = 0.0;
                dirVec.z = 1.0;
            }
            
            directions.push_back(dirVec);
            
            logger.debug("Drone %zu position: [%.3f, %.3f, %.3f]",
                        i, drone.position[0], drone.position[1], drone.position[2]);
            logger.debug("Drone %zu direction: [%.3f, %.3f, %.3f]",
                        i, dirVec.x, dirVec.y, dirVec.z);
        }
        logger.debug("==================================================");
        
        return true;
    }
    
    bool IsDuplicateRequest(uint32_t sequenceNumber, uint32_t droneId, uint64_t timestamp) {
        std::lock_guard<std::mutex> lock(recentRequestsMutex_);
        
        // Check if this request is already in our recent requests list
        for (const auto& req : recentRequests_) {
            if (req.sequenceNumber == sequenceNumber && req.droneId == droneId) {
                return true;
            }
        }
        
        // Not a duplicate, add to recent requests
        recentRequests_.emplace_front(sequenceNumber, droneId, timestamp);
        
        // Limit list size
        const size_t MAX_REQUESTS = 100;
        if (recentRequests_.size() > MAX_REQUESTS) {
            recentRequests_.pop_back();
        }
        
        return false;
    }
    
    uint16_t CalculateChecksum(const void* data, size_t length) {
        const uint8_t* buffer = reinterpret_cast<const uint8_t*>(data);
        uint16_t checksum = 0;
        
        // XOR-based checksum calculation in network byte order
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
    
    // Configuration and state
    Config config_;
    
    // Sockets
    int tcpServerSocket_;
    int udpSocket_;
    
    // Running flag and threads
    std::atomic<bool> running_;
    std::thread tcpThread_;
    std::thread monitorThread_;
    
    // Client management
    std::mutex clientsMutex_;
    std::vector<TcpClientRecord> tcpClients_;
    std::mutex clientChangeMutex_;
    std::condition_variable clientChangeCondition_;
    bool clientChangeSignal_;
    
    // Task queue for processing requests
    TaskQueue taskQueue_;
    
    // Request tracking for deduplication
    std::mutex recentRequestsMutex_;
    std::deque<RequestRecord> recentRequests_;
    
    // Statistics
    std::atomic<uint64_t> requestCount_;
    std::atomic<uint64_t> successCount_;
    std::atomic<uint64_t> failureCount_;
    
    // VRPN tracker
    std::unique_ptr<VrpnTracker> vrpnTracker_;
    
    // Solution history
    std::mutex solutionHistoryMutex_;
    std::vector<Vector3D> rawDirections_;       // Raw output from optimizer
    std::vector<Vector3D> commandedDirections_; // Rate-limited directions sent to drones
    std::vector<double> lastTensions_;          // Last tensions sent to drones
    Vector3D lastResultantForce_;               // Last requested force
    uint32_t lastSequence_ = 0;                 // Last processed sequence number
    bool hasValidHistory_;                      // Whether we have valid history
    
    // Rate limiting
    uint64_t lastOptimizationTime_;             // Time of last optimization
    
    // More statistics
    std::atomic<uint64_t> messagesSent_;
    uint64_t lastStatsTime_;
};

// Signal handler
std::atomic<bool> g_running(true);

void signal_handler(int signal) {
    logger.info("Received signal %d, shutting down...", signal);
    g_running = false;
    
    // Close the CSV log files
    csvLogger.close();
}

// Main function
int main(int argc, char* argv[]) {
    // Default parameters
    MAATSTensionOptimizer::Config config;
    
    // Parse command line arguments
    int opt;
    while ((opt = getopt(argc, argv, "d:t:u:v:n:p:r:s:m:b:f:a:z:h")) != -1) {
        switch (opt) {
            case 'd':
                config.droneAddresses.push_back(optarg);
                break;
            case 't':
                config.tcpPort = std::stoi(optarg);
                break;
            case 'u':
                config.udpPort = std::stoi(optarg);
                break;
            case 'v':
                {
                    std::string level(optarg);
                    if (level == "debug") config.logLevel = Logger::DEBUG;
                    else if (level == "info") config.logLevel = Logger::INFO;
                    else if (level == "warn") config.logLevel = Logger::WARN;
                    else if (level == "error") config.logLevel = Logger::ERROR;
                    else if (level == "none") config.logLevel = Logger::NONE;
                }
                break;
            case 'n':
                config.numDrones = std::stoi(optarg);
                break;
            case 'p':
                config.vrpnPort = std::stoi(optarg);
                break;
            case 'r':
                config.maxDirectionRateRad = std::stod(optarg);
                break;
            case 's':
                config.vrpnHost = optarg;
                break;
            case 'm':
                config.optimizerMu = std::stod(optarg);
                break;
            case 'b':
                config.directionBias = std::stod(optarg);
                break;
            case 'f':
                config.threadPoolSize = std::stoi(optarg);
                break;
            case 'a':
                config.threadPoolSize = std::stoi(optarg);
                break;
            case 'z':
                config.optimizationRateHz = std::stoi(optarg);
                break;
            case 'h':
                std::cout << "MAATSTensionOptimizer Three-Phase v" 
                        << SERVER_VERSION_MAJOR << "."
                        << SERVER_VERSION_MINOR << "."
                        << SERVER_VERSION_PATCH << std::endl;
                std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
                std::cout << "Options:" << std::endl;
                std::cout << "  -d addr     Drone IP address (can be specified multiple times)" << std::endl;
                std::cout << "  -t port     TCP port (default: " << config.tcpPort << ")" << std::endl;
                std::cout << "  -u port     UDP port (default: " << config.udpPort << ")" << std::endl;
                std::cout << "  -v level    Log level (debug, info, warn, error, none) (default: info)" << std::endl;
                std::cout << "  -n num      Number of drones (default: " << config.numDrones << ")" << std::endl;
                std::cout << "  -p port     VRPN port (default: " << config.vrpnPort << ")" << std::endl;
                std::cout << "  -r rate     Max direction change rate rad/s (default: " << config.maxDirectionRateRad << ")" << std::endl;
                std::cout << "  -s host     VRPN server host (default: " << config.vrpnHost << ")" << std::endl;
                std::cout << "  -m value    Optimizer mu parameter (default: " << config.optimizerMu << ")" << std::endl;
                std::cout << "  -b value    Direction continuity bias (default: " << config.directionBias << ")" << std::endl;
                std::cout << "  -a num      Thread pool size (default: " << config.threadPoolSize << ")" << std::endl;
                std::cout << "  -z rate     Optimization rate in Hz (default: auto-selected based on drone count)" << std::endl;
                std::cout << "  -h          Show this help message" << std::endl;
                return 0;
            default:
                std::cerr << "Unknown option: " << opt << std::endl;
                return 1;
        }
    }
    
    // Setup signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Set logger level from config
    logger.setLevel(config.logLevel);
    
    // Open CSV log files
    std::string timestamp_str = std::to_string(GetCurrentTimeMs());
    if (csvLogger.openFiles(timestamp_str)) {
        logger.info("CSV logging enabled to requests_%s.csv and responses_%s.csv", 
                  timestamp_str.c_str(), timestamp_str.c_str());
    } else {
        logger.error("Failed to open CSV log files");
    }
    
    // Create and start server
    MAATSTensionOptimizer server(config);
    
    if (server.Start()) {
        logger.info("MAATSTensionOptimizer v%d.%d.%d started", 
                SERVER_VERSION_MAJOR, SERVER_VERSION_MINOR, SERVER_VERSION_PATCH);
        
        // Log network configuration
        logger.warn("============= THREE-PHASE CONFIGURATION =============");
        logger.warn("TCP listening on: %s:%d", config.bindAddress.c_str(), config.tcpPort);
        logger.warn("UDP unicast port: %d", config.udpPort);
        logger.warn("Number of drones: %d", config.numDrones);
        logger.warn("Direction bias: %.2f", config.directionBias);
        logger.warn("Max direction rate: %.2f rad/s (~%.1f deg/s)", 
                config.maxDirectionRateRad, config.maxDirectionRateRad * 180.0 / M_PI);
        
        // Optimization rate is auto-selected based on drone count if not specified
        if (config.numDrones <= 2) {
            logger.warn("Optimization rate: %d Hz (optimized for 2 drones)", config.optimizationRateHz);
        } else if (config.numDrones == 3) {
            logger.warn("Optimization rate: %d Hz (optimized for 3 drones)", config.optimizationRateHz);
        } else {
            logger.warn("Optimization rate: %d Hz", config.optimizationRateHz);
        }
        
        // Test unicast to all drones
        logger.warn("Testing unicast to all drones...");
        bool testResult = server.TestAllDrones();
        logger.warn("Unicast test %s", testResult ? "SUCCEEDED" : "FAILED");
        logger.warn("=========================================================");
        
        logger.info("Press Ctrl+C to stop");
        
        // Main loop
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Stop server
        server.Stop();
    } else {
        logger.error("Failed to start server");
        return 1;
    }
    
    // Close CSV log files
    csvLogger.close();
    
    return 0;
}
