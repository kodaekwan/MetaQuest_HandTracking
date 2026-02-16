/**
 * @file zed_shared_memory.h
 * @brief ZED Camera Manager - Shared Memory Definitions
 * 
 * This header defines the shared memory structures for communication
 * between the C++ camera manager and Python interface.
 * 
 * Author: Daekwan Ko (kodaekwan)
 * Interactive Robotics Lab, Dongguk University
 */

#ifndef ZED_SHARED_MEMORY_H
#define ZED_SHARED_MEMORY_H

#include <cstdint>
#include <cstring>
#include <atomic>

// ============================================
// Configuration Constants
// ============================================
#define SHM_NAME_STATUS "/zed_status"
#define SHM_NAME_FRAME "/zed_frame"
#define SHM_NAME_COMMAND "/zed_command"

#define MAX_CAMERAS 4  // ZED typically supports fewer cameras
#define SERIAL_NUMBER_LEN 32
#define CAMERA_NAME_LEN 64
#define PATH_LEN 256
#define QR_DATA_LEN 512
#define TIMESTAMP_LEN 64
#define IP_ADDR_LEN 32

// Maximum frame dimensions (stereo side-by-side)
#define MAX_FRAME_WIDTH 2560   // 1280 * 2 for stereo
#define MAX_FRAME_HEIGHT 720
#define MAX_FRAME_CHANNELS 3

// ============================================
// Command Types
// ============================================
enum class CommandType : uint8_t {
    CMD_NONE = 0,
    CMD_START_PREVIEW,
    CMD_STOP_PREVIEW,
    CMD_START_RECORDING,
    CMD_STOP_RECORDING,
    CMD_START_STREAMING,
    CMD_STOP_STREAMING,
    CMD_SET_RESOLUTION,
    CMD_SET_FPS,
    CMD_SET_STREAM_PARAMS,
    CMD_SET_STEREO_PARAMS,
    CMD_SHUTDOWN
};

// ============================================
// Camera Status Enum
// ============================================
enum class CameraState : uint8_t {
    STATE_DISCONNECTED = 0,
    STATE_CONNECTED,
    STATE_PREVIEW,
    STATE_STREAMING,
    STATE_RECORDING,
    STATE_STREAMING_RECORDING,
    STATE_ERROR
};

// ============================================
// Stream Configuration (for XR devices)
// ============================================
struct StreamConfig {
    char target_ip[IP_ADDR_LEN];
    uint16_t target_port;
    uint16_t width;
    uint16_t height;
    uint16_t max_payload;
    uint8_t jpeg_quality;
    uint8_t fps;
    bool streaming_enabled;
    
    void clear() {
        memset(this, 0, sizeof(StreamConfig));
        target_port = 9003;
        width = 640;
        height = 480;
        max_payload = 1400;
        jpeg_quality = 50;
        fps = 30;
        streaming_enabled = false;
    }
};

// ============================================
// Single Camera Status Structure
// ============================================
struct CameraStatus {
    char serial_number[SERIAL_NUMBER_LEN];     // Camera serial number
    char camera_name[CAMERA_NAME_LEN];          // Camera display name
    CameraState state;                          // Current camera state
    
    // Camera properties
    uint32_t width;                             // Single eye width
    uint32_t height;
    uint32_t fps;
    
    // Flags
    bool is_connected;
    bool is_previewing;
    bool is_recording;
    bool is_streaming;
    
    // Timestamps
    int64_t camera_timestamp_us;                // ZED hardware timestamp (microseconds)
    int64_t system_timestamp_us;                // System timestamp
    char global_time_str[TIMESTAMP_LEN];        // Formatted global time string
    
    // QR Recognition
    bool qr_detected;
    char qr_data[QR_DATA_LEN];                  // QR code content
    int64_t qr_timestamp_us;                    // QR timestamp (0 if not detected)
    char qr_pc_time[TIMESTAMP_LEN];             // PC time from QR
    char qr_robot_time[TIMESTAMP_LEN];          // Robot time from QR
    
    // Streaming info
    StreamConfig stream_config;
    
    // Recording
    char recording_path[PATH_LEN];
    uint64_t frames_recorded;
    
    void clear() {
        memset(this, 0, sizeof(CameraStatus));
        state = CameraState::STATE_DISCONNECTED;
        stream_config.clear();
    }
};

// ============================================
// Global Status Structure (for all cameras)
// ============================================
struct GlobalStatus {
    std::atomic<uint64_t> update_counter;       // Incremented on each update
    uint32_t num_cameras;
    CameraStatus cameras[MAX_CAMERAS];
    
    // System info
    char last_error[256];
    bool manager_running;
    
    // Control server port
    uint16_t control_port;
    
    void init() {
        update_counter.store(0);
        num_cameras = 0;
        memset(last_error, 0, sizeof(last_error));
        manager_running = false;
        control_port = 0;
        for (int i = 0; i < MAX_CAMERAS; i++) {
            cameras[i].clear();
        }
    }
};

// ============================================
// Frame Data Structure
// ============================================
struct FrameHeader {
    std::atomic<uint64_t> frame_counter;        // Frame sequence number
    uint32_t camera_index;                      // Camera index
    uint32_t width;                             // Total width (stereo side-by-side)
    uint32_t height;
    uint32_t channels;                          // 3 for BGR
    uint32_t stride;                            // Row stride in bytes
    
    // Timestamps
    int64_t camera_timestamp_us;                // Hardware timestamp
    int64_t system_timestamp_us;                // System time when captured
    char global_time_str[TIMESTAMP_LEN];
    
    // QR info captured with this frame
    bool qr_detected;
    char qr_pc_time[TIMESTAMP_LEN];
    char qr_robot_time[TIMESTAMP_LEN];
    
    // Last detected QR time (for robot sync interpolation)
    int64_t last_qr_pc_timestamp_us;            // Last QR PC time in microseconds
    int64_t last_qr_robot_timestamp_us;         // Last QR Robot time in microseconds
    int64_t last_qr_cam_timestamp_us;           // Camera HW timestamp at QR detection
    char last_qr_pc_time[TIMESTAMP_LEN];        // Last QR PC time string
    char last_qr_robot_time[TIMESTAMP_LEN];     // Last QR Robot time string
    
    void init() {
        frame_counter.store(0);
        camera_index = 0;
        width = 0;
        height = 0;
        channels = 3;
        stride = 0;
        camera_timestamp_us = 0;
        system_timestamp_us = 0;
        memset(global_time_str, 0, sizeof(global_time_str));
        qr_detected = false;
        memset(qr_pc_time, 0, sizeof(qr_pc_time));
        memset(qr_robot_time, 0, sizeof(qr_robot_time));
        last_qr_pc_timestamp_us = 0;
        last_qr_robot_timestamp_us = 0;
        last_qr_cam_timestamp_us = 0;
        memset(last_qr_pc_time, 0, sizeof(last_qr_pc_time));
        memset(last_qr_robot_time, 0, sizeof(last_qr_robot_time));
    }
};

// Total frame shared memory size (header + max frame data)
constexpr size_t FRAME_DATA_SIZE = MAX_FRAME_WIDTH * MAX_FRAME_HEIGHT * MAX_FRAME_CHANNELS;
constexpr size_t FRAME_SHM_SIZE = sizeof(FrameHeader) + FRAME_DATA_SIZE;

// ============================================
// Command Structure
// ============================================
struct Command {
    std::atomic<uint64_t> command_counter;      // Incremented for each new command
    uint64_t last_processed;                    // Last processed command counter
    
    CommandType type;
    uint32_t camera_index;
    
    // Parameters
    union {
        struct {
            uint32_t width;
            uint32_t height;
        } resolution;
        struct {
            uint32_t fps;
        } framerate;
        struct {
            char path[PATH_LEN];
            char filename[CAMERA_NAME_LEN];
        } recording;
        struct {
            char target_ip[IP_ADDR_LEN];
            uint16_t target_port;
            uint16_t width;
            uint16_t height;
            uint8_t jpeg_quality;
        } streaming;
        struct {
            float focus;
            float quad;
            float zoom;
            bool add_focus;
        } stereo_params;
    } params;
    
    void init() {
        command_counter.store(0);
        last_processed = 0;
        type = CommandType::CMD_NONE;
        camera_index = 0;
        memset(&params, 0, sizeof(params));
    }
};

// Helper function to get frame data pointer
inline uint8_t* getFrameDataPtr(void* shm_ptr) {
    return static_cast<uint8_t*>(shm_ptr) + sizeof(FrameHeader);
}

inline const uint8_t* getFrameDataPtr(const void* shm_ptr) {
    return static_cast<const uint8_t*>(shm_ptr) + sizeof(FrameHeader);
}

#endif // ZED_SHARED_MEMORY_H
