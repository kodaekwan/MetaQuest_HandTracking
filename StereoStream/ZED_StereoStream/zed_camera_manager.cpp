/**
 * @file zed_camera_manager.cpp
 * @brief ZED Camera Manager with XR Streaming, QR Recognition and Time Sync
 * 
 * Features:
 * - ZED stereo camera support
 * - UDP streaming to Apple Vision Pro / Meta Quest
 * - TCP control server for Python interface
 * - Video recording with CSV metadata
 * - QR code recognition for robot time sync
 * - POSIX shared memory for real-time frame sharing
 * - Dual time interpolation (system/camera based)
 * 
 * Author: Daekwan Ko (kodaekwan)
 * Interactive Robotics Lab, Dongguk University
 * 
 * Build (in Docker):
 *   cmake . && make
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <csignal>
#include <algorithm>
#include <cstdlib>

// Network includes
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

// ZED SDK
#include <sl/Camera.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// ZBar for QR scanning
#include <zbar.h>

// Shared Memory
#include <sys/mman.h>
#include <sys/stat.h>

#include "zed_shared_memory.h"

namespace fs = std::filesystem;
using namespace std;
using namespace sl;

// ============================================
// Global signal handler
// ============================================
volatile sig_atomic_t g_shutdown_requested = 0;

void signalHandler(int signum) {
    g_shutdown_requested = 1;
}

// ============================================
// Simple JSON Parser
// ============================================
class SimpleJson {
public:
    map<string, string> data;
    
    static SimpleJson parse(const string& json_str) {
        SimpleJson result;
        string s = json_str;
        // Remove whitespace
        s.erase(remove(s.begin(), s.end(), '\n'), s.end());
        s.erase(remove(s.begin(), s.end(), '\r'), s.end());
        s.erase(remove(s.begin(), s.end(), '\t'), s.end());
        
        if (s.front() == '{') s = s.substr(1);
        if (s.back() == '}') s = s.substr(0, s.length() - 1);
        
        size_t pos = 0;
        while (pos < s.length()) {
            size_t key_start = s.find('"', pos);
            if (key_start == string::npos) break;
            size_t key_end = s.find('"', key_start + 1);
            if (key_end == string::npos) break;
            string key = s.substr(key_start + 1, key_end - key_start - 1);
            
            size_t colon = s.find(':', key_end);
            if (colon == string::npos) break;
            
            size_t val_start = colon + 1;
            while (val_start < s.length() && s[val_start] == ' ') val_start++;
            
            string value;
            if (s[val_start] == '"') {
                size_t val_end = s.find('"', val_start + 1);
                value = s.substr(val_start + 1, val_end - val_start - 1);
                pos = val_end + 1;
            } else {
                size_t val_end = s.find_first_of(",}", val_start);
                if (val_end == string::npos) val_end = s.length();
                value = s.substr(val_start, val_end - val_start);
                // Trim spaces
                value.erase(0, value.find_first_not_of(" "));
                value.erase(value.find_last_not_of(" ") + 1);
                pos = val_end;
            }
            
            result.data[key] = value;
            if (pos < s.length() && s[pos] == ',') pos++;
        }
        return result;
    }
    
    string get(const string& key, const string& default_val = "") const {
        auto it = data.find(key);
        return (it != data.end()) ? it->second : default_val;
    }
    
    int getInt(const string& key, int default_val = 0) const {
        auto it = data.find(key);
        if (it != data.end()) {
            try { return stoi(it->second); } catch (...) {}
        }
        return default_val;
    }
    
    bool getBool(const string& key, bool default_val = false) const {
        auto it = data.find(key);
        if (it != data.end()) {
            return it->second == "true" || it->second == "1";
        }
        return default_val;
    }
    
    float getFloat(const string& key, float default_val = 0.0f) const {
        auto it = data.find(key);
        if (it != data.end()) {
            try { return stof(it->second); } catch (...) {}
        }
        return default_val;
    }
    
    bool hasKey(const string& key) const {
        return data.find(key) != data.end();
    }
    
    // Helper to format float without trailing zeros
    static string formatFloat(float val) {
        stringstream ss;
        ss << std::fixed << std::setprecision(2) << val;
        string s = ss.str();
        // Remove trailing zeros after decimal point
        size_t dot = s.find('.');
        if (dot != string::npos) {
            size_t last = s.find_last_not_of('0');
            if (last != string::npos && last > dot) {
                s = s.substr(0, last + 1);
            } else if (last == dot) {
                s = s.substr(0, dot + 2);  // Keep at least one decimal
            }
        }
        return s;
    }
    
    static string stringify(const map<string, string>& obj) {
        stringstream ss;
        ss << "{";
        bool first = true;
        for (const auto& kv : obj) {
            if (!first) ss << ",";
            first = false;
            ss << "\"" << kv.first << "\":";
            if (kv.second == "true" || kv.second == "false" ||
                (!kv.second.empty() && (isdigit(kv.second[0]) || kv.second[0] == '-'))) {
                ss << kv.second;
            } else {
                ss << "\"" << kv.second << "\"";
            }
        }
        ss << "}";
        return ss.str();
    }
};

// ============================================
// Configuration Structure
// ============================================
struct ZedConfig {
    // Camera settings
    string serial;
    string name = "zed_camera";
    int width = 1280;
    int height = 720;
    int fps = 30;
    int depth_mode = 0;  // 0=NONE, 1=PERFORMANCE, etc.
    
    // Stream settings
    string stream_target_ip;
    int stream_port = 9003;
    int stream_width = 640;
    int stream_height = 480;
    int stream_quality = 50;
    bool auto_stream = false;  // Auto-start streaming on startup
    
    // Stereo params for XR devices
    int stereo_params_port = 9004;
    float stereo_focus = 0.5f;
    float stereo_quad = 1.8f;
    float stereo_zoom = 1.0f;
    bool stereo_add_focus = false;
    
    // Recording settings
    string output_folder = "./recordings";
    
    // Preview
    bool enable_preview = false;
    
    // Control server
    int control_port = 0;  // 0 = auto assign
    
    bool load(const string& filepath) {
        ifstream file(filepath);
        if (!file.is_open()) return false;
        
        string content((istreambuf_iterator<char>(file)),
                       istreambuf_iterator<char>());
        file.close();
        
        SimpleJson json = SimpleJson::parse(content);
        
        if (json.hasKey("serial")) serial = json.get("serial");
        if (json.hasKey("name")) name = json.get("name");
        if (json.hasKey("width")) width = json.getInt("width", 1280);
        if (json.hasKey("height")) height = json.getInt("height", 720);
        if (json.hasKey("fps")) fps = json.getInt("fps", 30);
        if (json.hasKey("depth_mode")) depth_mode = json.getInt("depth_mode", 0);
        
        if (json.hasKey("stream_target_ip")) stream_target_ip = json.get("stream_target_ip");
        if (json.hasKey("stream_port")) stream_port = json.getInt("stream_port", 9003);
        if (json.hasKey("stream_width")) stream_width = json.getInt("stream_width", 640);
        if (json.hasKey("stream_height")) stream_height = json.getInt("stream_height", 480);
        if (json.hasKey("stream_quality")) stream_quality = json.getInt("stream_quality", 50);
        if (json.hasKey("auto_stream")) auto_stream = json.getBool("auto_stream", false);
        
        if (json.hasKey("stereo_params_port")) stereo_params_port = json.getInt("stereo_params_port", 9004);
        if (json.hasKey("stereo_focus")) stereo_focus = json.getFloat("stereo_focus", 0.5f);
        if (json.hasKey("stereo_quad")) stereo_quad = json.getFloat("stereo_quad", 1.8f);
        if (json.hasKey("stereo_zoom")) stereo_zoom = json.getFloat("stereo_zoom", 1.0f);
        if (json.hasKey("stereo_add_focus")) stereo_add_focus = json.getBool("stereo_add_focus", false);
        
        if (json.hasKey("output_folder")) output_folder = json.get("output_folder");
        if (json.hasKey("enable_preview")) enable_preview = json.getBool("enable_preview", false);
        if (json.hasKey("control_port")) control_port = json.getInt("control_port", 0);
        
        return true;
    }
};

// ============================================
// QR Code Scanner (using ZBar) - optimized
// ============================================
class QRScanner {
public:
    QRScanner() {
        // Enable QR decoding and reduce density for speed
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_X_DENSITY, 2);
        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_Y_DENSITY, 2);
    }

    // Optimized scan with optional downsampling (ratio 0.0-1.0)
    bool scan(const cv::Mat& frame, string& result, float downsample_ratio = 1.0f) {
        cv::Mat gray, down;

        if (frame.channels() == 3) cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        else if (frame.channels() == 4) cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
        else gray = frame;

        if (downsample_ratio > 0.0f && downsample_ratio < 1.0f) {
            cv::resize(gray, down, cv::Size(), downsample_ratio, downsample_ratio, cv::INTER_LINEAR);
        } else {
            down = gray;
        }

        zbar::Image image(down.cols, down.rows, "Y800", down.data, down.cols * down.rows);
        int n = scanner_.scan(image);
        if (n > 0) {
            for (auto symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
                if (symbol->get_type() == zbar::ZBAR_QRCODE) {
                    result = symbol->get_data();
                    return true;
                }
            }
        }
        return false;
    }

    // Parse QR content for PC and Robot fields (JSON-ish string)
    bool parseTimeQR(const string& qr_content, string& pc_time, string& robot_time) {
        try {
            size_t pc_pos = qr_content.find("\"PC\"");
            size_t robot_pos = qr_content.find("\"Robot\"");

            if (pc_pos != string::npos) {
                size_t quote1 = qr_content.find('"', pc_pos + 4);
                if (quote1 != string::npos) {
                    size_t quote2 = qr_content.find('"', quote1 + 1);
                    if (quote2 != string::npos) pc_time = qr_content.substr(quote1 + 1, quote2 - quote1 - 1);
                }
            }

            if (robot_pos != string::npos) {
                size_t quote1 = qr_content.find('"', robot_pos + 7);
                if (quote1 != string::npos) {
                    size_t quote2 = qr_content.find('"', quote1 + 1);
                    if (quote2 != string::npos) robot_time = qr_content.substr(quote1 + 1, quote2 - quote1 - 1);
                }
            }

            return !pc_time.empty();
        } catch (...) {
            return false;
        }
    }

private:
    zbar::ImageScanner scanner_;
};

// ============================================
// Time Utilities
// ============================================
int64_t getCurrentTimestampUs() {
    auto now = chrono::system_clock::now();
    auto us = chrono::duration_cast<chrono::microseconds>(now.time_since_epoch());
    return us.count();
}

string formatTimestamp(int64_t timestamp_us) {
    auto seconds = timestamp_us / 1000000;
    auto micros = timestamp_us % 1000000;
    time_t time = static_cast<time_t>(seconds);
    tm* tm_info = localtime(&time);
    
    char buffer[64];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", tm_info);
    
    stringstream ss;
    ss << buffer << "." << setfill('0') << setw(6) << micros;
    return ss.str();
}

int64_t parseTimestampToUs(const string& timestamp) {
    // Parse format: "YYYY-MM-DD HH:MM:SS.ffffff" or with timezone
    tm tm_info = {};
    int micros = 0;
    
    size_t dot_pos = timestamp.find('.');
    if (dot_pos != string::npos) {
        string dt_part = timestamp.substr(0, dot_pos);
        string us_part = timestamp.substr(dot_pos + 1);
        
        // Remove timezone if present
        size_t tz_pos = us_part.find('+');
        if (tz_pos == string::npos) tz_pos = us_part.find('-');
        if (tz_pos != string::npos) {
            us_part = us_part.substr(0, tz_pos);
        }
        
        strptime(dt_part.c_str(), "%Y-%m-%d %H:%M:%S", &tm_info);
        
        // Parse microseconds (pad/truncate to 6 digits)
        while (us_part.length() < 6) us_part += "0";
        if (us_part.length() > 6) us_part = us_part.substr(0, 6);
        micros = stoi(us_part);
    } else {
        strptime(timestamp.c_str(), "%Y-%m-%d %H:%M:%S", &tm_info);
    }
    
    time_t local_time = mktime(&tm_info);
    int64_t result_local = static_cast<int64_t>(local_time) * 1000000 + micros;

    // Try additional timezone interpretations (UTC and Asia/Seoul) when
    // possible, and pick the one closest to current system time. This helps
    // when the QR PC timestamps are in KST while the container/system is UTC.
#if defined(_GNU_SOURCE) || defined(__unix__) || defined(__APPLE__)
    int64_t now_us = getCurrentTimestampUs();

    // UTC interpretation
    time_t utc_time = timegm(&tm_info);
    int64_t result_utc = static_cast<int64_t>(utc_time) * 1000000 + micros;

    // KST (Asia/Seoul) interpretation: PC time is KST (UTC+9). To convert the
    // wall-clock KST `tm_info` to epoch (UTC), subtract 9 hours from the UTC
    // interpretation computed by timegm.
    const int64_t KST_OFFSET_S = 0LL;
    int64_t result_kst = static_cast<int64_t>(utc_time - KST_OFFSET_S) * 1000000 + micros;

    int64_t diff_local = llabs(result_local - now_us);
    int64_t diff_utc = llabs(result_utc - now_us);
    int64_t diff_kst = llabs(result_kst - now_us);

    // Choose the interpretation with the smallest absolute difference to now
    if (diff_kst <= diff_utc && diff_kst <= diff_local) return result_kst;
    if (diff_utc <= diff_local && diff_utc <= diff_kst) return result_utc;
    return result_local;
#else
    return result_local;
#endif
}

// ============================================
// UDP Image Sender (for XR streaming)
// ============================================
class UdpImageSender {
public:
    UdpImageSender() : sock_fd_(-1), frame_id_(0), connected_(false), stop_flag_(true) {}
    
    ~UdpImageSender() { close(); }
    
    bool open(const string& ip, int port, int width, int height, int quality, int max_payload = 1400) {
        target_ip_ = ip;
        target_port_ = port;
        width_ = width;
        height_ = height;
        quality_ = quality;
        max_payload_ = max_payload;
        
        sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd_ < 0) {
            cerr << "[UDP Error] Failed to create socket" << endl;
            return false;
        }
        
        int buf_size = 4 * 1024 * 1024;
        setsockopt(sock_fd_, SOL_SOCKET, SO_SNDBUF, &buf_size, sizeof(buf_size));
        
        memset(&target_addr_, 0, sizeof(target_addr_));
        target_addr_.sin_family = AF_INET;
        target_addr_.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &target_addr_.sin_addr);
        
        if (::connect(sock_fd_, (struct sockaddr*)&target_addr_, sizeof(target_addr_)) < 0) {
            cerr << "[UDP Error] Failed to connect" << endl;
            ::close(sock_fd_);
            sock_fd_ = -1;
            return false;
        }
        
        connected_ = true;
        stop_flag_ = false;
        worker_thread_ = thread(&UdpImageSender::workerLoop, this);
        
        cout << "[UDP] Streaming to " << ip << ":" << port << " @ " << width << "x" << height << endl;
        return true;
    }
    
    void close() {
        stop_flag_ = true;
        queue_cv_.notify_one();
        
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
        
        if (sock_fd_ >= 0) {
            ::close(sock_fd_);
            sock_fd_ = -1;
        }
        connected_ = false;
    }
    
    void sendImage(const cv::Mat& img) {
        if (stop_flag_ || !connected_) return;
        
        lock_guard<mutex> lock(queue_mutex_);
        while (!frame_queue_.empty()) {
            frame_queue_.pop();
        }
        frame_queue_.push(img.clone());
        queue_cv_.notify_one();
    }
    
    bool isRunning() const { return !stop_flag_ && connected_; }
    
    string getTargetIP() const { return target_ip_; }
    int getTargetPort() const { return target_port_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }

private:
    void workerLoop() {
        while (!stop_flag_) {
            cv::Mat img;
            {
                unique_lock<mutex> lock(queue_mutex_);
                queue_cv_.wait_for(lock, chrono::milliseconds(100), [this]() {
                    return !frame_queue_.empty() || stop_flag_;
                });
                
                if (stop_flag_) break;
                if (frame_queue_.empty()) continue;
                
                img = frame_queue_.front();
                frame_queue_.pop();
            }
            
            // Resize if needed
            if (img.cols != width_ * 2 || img.rows != height_) {
                cv::resize(img, img, cv::Size(width_ * 2, height_));
            }
            
            // Convert to BGR if needed
            cv::Mat img_bgr;
            if (img.channels() == 4) {
                cv::cvtColor(img, img_bgr, cv::COLOR_BGRA2BGR);
            } else {
                img_bgr = img;
            }
            
            // JPEG encode
            vector<uchar> jpeg_buf;
            vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality_};
            if (!cv::imencode(".jpg", img_bgr, jpeg_buf, params)) {
                continue;
            }
            
            sendPackets(jpeg_buf);
        }
    }
    
    void sendPackets(const vector<uchar>& data) {
        uint32_t fid = frame_id_++ & 0xFFFFFFFF;
        size_t total_len = data.size();
        uint16_t total_packets = (total_len + max_payload_ - 1) / max_payload_;
        
        vector<uchar> packet_buf(8 + max_payload_);
        
        for (uint16_t idx = 0; idx < total_packets; idx++) {
            size_t start = idx * max_payload_;
            size_t end = min(start + (size_t)max_payload_, total_len);
            size_t chunk_size = end - start;
            
            uint32_t fid_net = htonl(fid);
            uint16_t idx_net = htons(idx);
            uint16_t total_net = htons(total_packets);
            
            memcpy(packet_buf.data(), &fid_net, 4);
            memcpy(packet_buf.data() + 4, &idx_net, 2);
            memcpy(packet_buf.data() + 6, &total_net, 2);
            memcpy(packet_buf.data() + 8, data.data() + start, chunk_size);
            
            send(sock_fd_, packet_buf.data(), 8 + chunk_size, 0);
        }
    }
    
    string target_ip_;
    int target_port_;
    int width_;
    int height_;
    int quality_;
    int max_payload_;
    
    int sock_fd_;
    struct sockaddr_in target_addr_;
    atomic<uint32_t> frame_id_;
    atomic<bool> connected_;
    atomic<bool> stop_flag_;
    
    queue<cv::Mat> frame_queue_;
    mutex queue_mutex_;
    condition_variable queue_cv_;
    thread worker_thread_;
};

// ============================================
// Video Recorder with Metadata CSV
// ============================================
class VideoRecorder {
public:
    VideoRecorder() : is_recording_(false), frame_count_(0) {}
    
    ~VideoRecorder() { stop(); }
    
    static string getUniqueFilename(const string& path, const string& basename, const string& ext) {
        string full_path = path + "/" + basename + ext;
        
        if (!fs::exists(full_path)) {
            return full_path;
        }
        
        int counter = 1;
        while (true) {
            full_path = path + "/" + basename + "_" + to_string(counter) + ext;
            if (!fs::exists(full_path)) {
                return full_path;
            }
            counter++;
        }
    }
    
    bool start(const string& path, const string& filename, int width, int height, int fps,
               const string& camera_serial, const string& camera_name) {
        if (is_recording_) stop();
        
        fs::create_directories(path);
        
        video_filepath_ = getUniqueFilename(path, filename, ".mp4");
        csv_filepath_ = video_filepath_.substr(0, video_filepath_.length() - 4) + "_metadata.csv";
        
        camera_serial_ = camera_serial;
        camera_name_ = camera_name;
        
        int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        writer_.open(video_filepath_, fourcc, fps, cv::Size(width, height), true);
        
        if (!writer_.isOpened()) {
            cerr << "[Record Error] Failed to open video writer: " << video_filepath_ << endl;
            return false;
        }
        
        // Open CSV file
        csv_file_.open(csv_filepath_);
        if (!csv_file_.is_open()) {
            writer_.release();
            cerr << "[Record Error] Failed to open CSV: " << csv_filepath_ << endl;
            return false;
        }
        
        // Write CSV header
        csv_file_ << "frame_number,camera_timestamp_us,system_timestamp_us,global_time,"
                  << "qr_detected,qr_pc_time,qr_robot_time,"
                  << "last_qr_pc_time,last_qr_robot_time,"
                  << "last_qr_pc_timestamp_us,last_qr_robot_timestamp_us,last_qr_cam_timestamp_us,"
                  << "interpolated_robot_us,interpolated_robot_time,"
                  << "interpolated_robot_cam_us,interpolated_robot_cam_time,"
                  << "camera_serial,camera_name\n";
        
        frame_count_ = 0;
        is_recording_ = true;
        cout << "[Record] Started: " << video_filepath_ << endl;
        return true;
    }
    
    void writeFrame(const cv::Mat& frame, int64_t camera_timestamp_us, int64_t system_timestamp_us,
                    bool qr_detected, const string& qr_pc_time, const string& qr_robot_time,
                    int64_t last_qr_pc_us, int64_t last_qr_robot_us, int64_t last_qr_cam_us,
                    const string& last_qr_pc_str, const string& last_qr_robot_str) {
        if (!is_recording_ || !writer_.isOpened()) return;
        
        cv::Mat frame_bgr;
        if (frame.channels() == 4) {
            cv::cvtColor(frame, frame_bgr, cv::COLOR_BGRA2BGR);
        } else {
            frame_bgr = frame;
        }
        
        writer_.write(frame_bgr);
        
        // Calculate interpolated robot time
        int64_t interp_robot_us = 0;
        int64_t interp_robot_cam_us = 0;
        string interp_robot_str, interp_robot_cam_str;
        
        if (last_qr_robot_us > 0 && last_qr_pc_us > 0) {
            // System time based interpolation
            interp_robot_us = last_qr_robot_us + (system_timestamp_us - last_qr_pc_us);
            interp_robot_str = formatTimestamp(interp_robot_us);
            
            // Camera time based interpolation
            if (last_qr_cam_us > 0) {
                interp_robot_cam_us = last_qr_robot_us + (camera_timestamp_us - last_qr_cam_us);
                interp_robot_cam_str = formatTimestamp(interp_robot_cam_us);
            }
        }
        
        // Write CSV row
        csv_file_ << frame_count_ << ","
                  << camera_timestamp_us << ","
                  << system_timestamp_us << ","
                  << formatTimestamp(system_timestamp_us) << ","
                  << (qr_detected ? "true" : "false") << ","
                  << qr_pc_time << "," << qr_robot_time << ","
                  << last_qr_pc_str << "," << last_qr_robot_str << ","
                  << last_qr_pc_us << "," << last_qr_robot_us << "," << last_qr_cam_us << ","
                  << interp_robot_us << "," << interp_robot_str << ","
                  << interp_robot_cam_us << "," << interp_robot_cam_str << ","
                  << camera_serial_ << "," << camera_name_ << "\n";
        
        frame_count_++;
    }
    
    void stop() {
        if (is_recording_) {
            writer_.release();
            csv_file_.close();
            is_recording_ = false;
            cout << "[Record] Stopped: " << video_filepath_ << " (" << frame_count_ << " frames)" << endl;
        }
    }
    
    bool isRecording() const { return is_recording_; }
    string getFilepath() const { return video_filepath_; }
    uint64_t getFrameCount() const { return frame_count_; }

private:
    cv::VideoWriter writer_;
    ofstream csv_file_;
    string video_filepath_;
    string csv_filepath_;
    string camera_serial_;
    string camera_name_;
    atomic<bool> is_recording_;
    uint64_t frame_count_;
};

// ============================================
// TCP Control Server
// ============================================
class ControlServer {
public:
    ControlServer(int port = 0) : server_fd_(-1), port_(port), running_(false) {}
    
    ~ControlServer() { stop(); }
    
    bool start() {
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0) {
            cerr << "[Control Error] Failed to create socket" << endl;
            return false;
        }
        
        int opt = 1;
        setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port_);
        
        if (bind(server_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            cerr << "[Control Error] Failed to bind" << endl;
            ::close(server_fd_);
            return false;
        }
        
        socklen_t len = sizeof(addr);
        getsockname(server_fd_, (struct sockaddr*)&addr, &len);
        port_ = ntohs(addr.sin_port);
        
        if (listen(server_fd_, 5) < 0) {
            cerr << "[Control Error] Failed to listen" << endl;
            ::close(server_fd_);
            return false;
        }
        
        fcntl(server_fd_, F_SETFL, O_NONBLOCK);
        
        running_ = true;
        cout << "\n==========================================" << endl;
        cout << "[Control Server] Listening on port: " << port_ << endl;
        cout << "==========================================\n" << endl;
        
        return true;
    }
    
    void stop() {
        running_ = false;
        if (server_fd_ >= 0) {
            ::close(server_fd_);
            server_fd_ = -1;
        }
    }
    
    bool pollCommand(string& command) {
        if (!running_ || server_fd_ < 0) return false;
        
        struct pollfd pfd;
        pfd.fd = server_fd_;
        pfd.events = POLLIN;
        
        if (poll(&pfd, 1, 0) <= 0) return false;
        
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);
        
        if (client_fd < 0) return false;
        
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        
        char buffer[4096];
        int n = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
        if (n > 0) {
            buffer[n] = '\0';
            command = string(buffer);
            pending_client_fd_ = client_fd;
            return true;
        }
        
        ::close(client_fd);
        return false;
    }
    
    void sendResponse(const string& response) {
        if (pending_client_fd_ >= 0) {
            string resp = response + "\n";
            send(pending_client_fd_, resp.c_str(), resp.length(), 0);
            ::close(pending_client_fd_);
            pending_client_fd_ = -1;
        }
    }
    
    int getPort() const { return port_; }

private:
    int server_fd_;
    int port_;
    int pending_client_fd_ = -1;
    atomic<bool> running_;
};

// ============================================
// Shared Memory Manager
// ============================================
class SharedMemoryManager {
public:
    SharedMemoryManager() : status_shm_(nullptr), frame_shm_(nullptr), command_shm_(nullptr) {}
    
    ~SharedMemoryManager() { cleanup(); }
    
    bool init() {
        // Create status shared memory
        shm_unlink(SHM_NAME_STATUS);
        int fd = shm_open(SHM_NAME_STATUS, O_CREAT | O_RDWR, 0666);
        if (fd < 0) {
            cerr << "[SHM Error] Failed to create status shm" << endl;
            return false;
        }
        ftruncate(fd, sizeof(GlobalStatus));
        status_shm_ = static_cast<GlobalStatus*>(
            mmap(nullptr, sizeof(GlobalStatus), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
        close(fd);
        
        if (status_shm_ == MAP_FAILED) {
            cerr << "[SHM Error] Failed to map status shm" << endl;
            return false;
        }
        status_shm_->init();
        
        // Create frame shared memory
        shm_unlink(SHM_NAME_FRAME);
        fd = shm_open(SHM_NAME_FRAME, O_CREAT | O_RDWR, 0666);
        if (fd < 0) {
            cerr << "[SHM Error] Failed to create frame shm" << endl;
            return false;
        }
        ftruncate(fd, FRAME_SHM_SIZE);
        frame_shm_ = mmap(nullptr, FRAME_SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        close(fd);
        
        if (frame_shm_ == MAP_FAILED) {
            cerr << "[SHM Error] Failed to map frame shm" << endl;
            return false;
        }
        getFrameHeader()->init();
        
        // Create command shared memory
        shm_unlink(SHM_NAME_COMMAND);
        fd = shm_open(SHM_NAME_COMMAND, O_CREAT | O_RDWR, 0666);
        if (fd < 0) {
            cerr << "[SHM Error] Failed to create command shm" << endl;
            return false;
        }
        ftruncate(fd, sizeof(Command));
        command_shm_ = static_cast<Command*>(
            mmap(nullptr, sizeof(Command), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
        close(fd);
        
        if (command_shm_ == MAP_FAILED) {
            cerr << "[SHM Error] Failed to map command shm" << endl;
            return false;
        }
        command_shm_->init();
        
        cout << "[SHM] Shared memory initialized" << endl;
        return true;
    }
    
    void cleanup() {
        if (status_shm_) {
            munmap(status_shm_, sizeof(GlobalStatus));
            shm_unlink(SHM_NAME_STATUS);
            status_shm_ = nullptr;
        }
        if (frame_shm_) {
            munmap(frame_shm_, FRAME_SHM_SIZE);
            shm_unlink(SHM_NAME_FRAME);
            frame_shm_ = nullptr;
        }
        if (command_shm_) {
            munmap(command_shm_, sizeof(Command));
            shm_unlink(SHM_NAME_COMMAND);
            command_shm_ = nullptr;
        }
    }
    
    GlobalStatus* getStatus() { return status_shm_; }
    FrameHeader* getFrameHeader() { return static_cast<FrameHeader*>(frame_shm_); }
    uint8_t* getFrameData() { return static_cast<uint8_t*>(frame_shm_) + sizeof(FrameHeader); }
    Command* getCommand() { return command_shm_; }
    
    void updateFrame(const cv::Mat& frame, int64_t camera_ts, int64_t system_ts,
                     bool qr_detected, const string& qr_pc, const string& qr_robot,
                     int64_t last_qr_pc_us, int64_t last_qr_robot_us, int64_t last_qr_cam_us,
                     const string& last_qr_pc_str, const string& last_qr_robot_str) {
        if (!frame_shm_) return;
        
        FrameHeader* header = getFrameHeader();
        uint8_t* data = getFrameData();
        
        int width = frame.cols;
        int height = frame.rows;
        int channels = frame.channels();
        
        if (width > MAX_FRAME_WIDTH || height > MAX_FRAME_HEIGHT) {
            return;
        }
        
        // Convert to BGR if needed
        cv::Mat bgr_frame;
        if (channels == 4) {
            cv::cvtColor(frame, bgr_frame, cv::COLOR_BGRA2BGR);
            channels = 3;
        } else {
            bgr_frame = frame;
        }
        
        header->width = width;
        header->height = height;
        header->channels = channels;
        header->stride = width * channels;
        header->camera_timestamp_us = camera_ts;
        header->system_timestamp_us = system_ts;
        strncpy(header->global_time_str, formatTimestamp(system_ts).c_str(), TIMESTAMP_LEN - 1);
        
        header->qr_detected = qr_detected;
        strncpy(header->qr_pc_time, qr_pc.c_str(), TIMESTAMP_LEN - 1);
        strncpy(header->qr_robot_time, qr_robot.c_str(), TIMESTAMP_LEN - 1);
        
        header->last_qr_pc_timestamp_us = last_qr_pc_us;
        header->last_qr_robot_timestamp_us = last_qr_robot_us;
        header->last_qr_cam_timestamp_us = last_qr_cam_us;
        strncpy(header->last_qr_pc_time, last_qr_pc_str.c_str(), TIMESTAMP_LEN - 1);
        strncpy(header->last_qr_robot_time, last_qr_robot_str.c_str(), TIMESTAMP_LEN - 1);
        
        // Copy frame data
        memcpy(data, bgr_frame.data, bgr_frame.total() * bgr_frame.elemSize());
        
        header->frame_counter.fetch_add(1, std::memory_order_release);
    }

private:
    GlobalStatus* status_shm_;
    void* frame_shm_;
    Command* command_shm_;
};

// ============================================
// Main ZED Camera Manager
// ============================================
class ZedCameraManager {
public:
    ZedCameraManager() : running_(true), show_preview_(false) {}
    
    bool init(const ZedConfig& config) {
        config_ = config;
        show_preview_ = config.enable_preview;
        
        // Initialize shared memory
        if (!shm_manager_.init()) {
            cerr << "[Error] Failed to initialize shared memory" << endl;
            return false;
        }
        
        // Initialize ZED camera
        sl::InitParameters init_params;
        init_params.sdk_verbose = false;
        
        // Set resolution
        if (config.width >= 2208) {
            init_params.camera_resolution = sl::RESOLUTION::HD2K;
        } else if (config.width >= 1920) {
            init_params.camera_resolution = sl::RESOLUTION::HD1080;
        } else if (config.width >= 1280) {
            init_params.camera_resolution = sl::RESOLUTION::HD720;
        } else {
            init_params.camera_resolution = sl::RESOLUTION::VGA;
        }
        
        init_params.camera_fps = config.fps;
        init_params.depth_mode = sl::DEPTH_MODE::NONE;
        init_params.async_grab_camera_recovery = false;
        
        auto ret = zed_.open(init_params);
        if (ret != ERROR_CODE::SUCCESS) {
            cerr << "[Error] Camera Open: " << toString(ret) << endl;
            return false;
        }
        
        auto info = zed_.getCameraInformation();
        camera_serial_ = to_string(info.serial_number);
        camera_name_ = config.name;
        
        cout << "\n=== ZED Camera ===" << endl;
        cout << "Model: " << toString(info.camera_model) << endl;
        cout << "Serial: " << camera_serial_ << endl;
        cout << "Resolution: " << info.camera_configuration.resolution.width << "x"
             << info.camera_configuration.resolution.height << endl;
        cout << "FPS: " << info.camera_configuration.fps << endl;
        
        // Update shared memory status
        auto* status = shm_manager_.getStatus();
        status->num_cameras = 1;
        status->manager_running = true;
        strncpy(status->cameras[0].serial_number, camera_serial_.c_str(), SERIAL_NUMBER_LEN - 1);
        strncpy(status->cameras[0].camera_name, camera_name_.c_str(), CAMERA_NAME_LEN - 1);
        status->cameras[0].width = info.camera_configuration.resolution.width;
        status->cameras[0].height = info.camera_configuration.resolution.height;
        status->cameras[0].fps = info.camera_configuration.fps;
        status->cameras[0].is_connected = true;
        status->cameras[0].state = CameraState::STATE_CONNECTED;
        
        // Start control server
        control_server_ = make_unique<ControlServer>(config.control_port);
        if (!control_server_->start()) {
            zed_.close();
            return false;
        }
        status->control_port = control_server_->getPort();
        
        // Start streaming if auto_stream enabled and IP configured
        if (config.auto_stream && !config.stream_target_ip.empty()) {
            startStreaming(config.stream_target_ip, config.stream_port,
                          config.stream_width, config.stream_height, config.stream_quality);
            
            // Send stereo params to XR device
            map<string, string> stereo_params;
            stereo_params["focus"] = SimpleJson::formatFloat(config.stereo_focus);
            stereo_params["quad"] = SimpleJson::formatFloat(config.stereo_quad);
            stereo_params["zoom"] = SimpleJson::formatFloat(config.stereo_zoom);
            stereo_params["addFocus"] = config.stereo_add_focus ? "true" : "false";
            
            cout << "[Stereo] Sending params to " << config.stream_target_ip << ":" << config.stereo_params_port << endl;
            cout << "[Stereo] Payload: " << SimpleJson::stringify(stereo_params) << endl;
            string result = sendStereoParams(config.stream_target_ip, config.stereo_params_port, stereo_params);
            cout << "[Stereo] Response: " << result << endl;
        }
        
        if (show_preview_) {
            cv::namedWindow("ZED Manager Preview", cv::WINDOW_NORMAL);
            cv::resizeWindow("ZED Manager Preview", 1280, 480);
        }
        
        return true;
    }
    
    void run() {
        cout << "\n[Ready] Waiting for commands..." << endl;
        cout << "Commands: start_stream, stop_stream, start_record, stop_record, get_status, quit\n" << endl;
        
        Mat zed_left, zed_right;
        auto info = zed_.getCameraInformation();
        int cam_width = info.camera_configuration.resolution.width;
        int cam_height = info.camera_configuration.resolution.height;
        
        cv::Mat stereo_image(cam_height, cam_width * 2, CV_8UC3);
        
        int frame_count = 0;
        auto fps_time = chrono::steady_clock::now();

        // QR scan scheduling and state
        const int QR_SCAN_INTERVAL_MS = 700; // 0.7s default
        auto last_qr_scan_time = chrono::steady_clock::now();
        bool qr_ever_detected = false;
        int qr_detection_count = 0;
        bool was_recording = false;
        auto recording_start_time = chrono::steady_clock::now();
        
        // Error tracking
        int consecutive_errors = 0;
        auto last_error_time = chrono::steady_clock::now();
        int error_msg_count = 0;
        
        while (running_ && !g_shutdown_requested) {
            // Handle TCP commands
            string cmd;
            if (control_server_->pollCommand(cmd)) {
                handleCommand(cmd);
            }
            
            // Handle shared memory commands
            handleSharedMemoryCommand();
            
            // Grab frame
            auto ret = zed_.grab();
            if (ret != ERROR_CODE::SUCCESS) {
                consecutive_errors++;
                auto now = chrono::steady_clock::now();
                auto error_elapsed = chrono::duration_cast<chrono::seconds>(now - last_error_time).count();
                
                // Rate limit error messages (max 1 per second)
                if (ret != ERROR_CODE::CAMERA_REBOOTING && error_elapsed >= 1) {
                    cerr << "[Error] Grab: " << toString(ret);
                    if (consecutive_errors > 1) {
                        cerr << " (x" << consecutive_errors << ")";
                    }
                    cerr << endl;
                    last_error_time = now;
                    error_msg_count++;
                }
                
                // Warn about persistent errors
                if (consecutive_errors == 50) {
                    cerr << "[Warning] 50+ consecutive grab errors. Check USB connection." << endl;
                }
                
                this_thread::sleep_for(chrono::milliseconds(10));
                continue;
            }
            
            // Reset error counter on success
            consecutive_errors = 0;
            
            // Get timestamps
            int64_t camera_ts_us = zed_.getTimestamp(sl::TIME_REFERENCE::IMAGE).getMicroseconds();
            int64_t system_ts_us = getCurrentTimestampUs();
            
            // Retrieve images
            zed_.retrieveImage(zed_left, VIEW::LEFT);
            zed_.retrieveImage(zed_right, VIEW::RIGHT);
            
            cv::Mat cv_left((int)zed_left.getHeight(), (int)zed_left.getWidth(),
                           CV_8UC4, zed_left.getPtr<sl::uchar1>(sl::MEM::CPU));
            cv::Mat cv_right((int)zed_right.getHeight(), (int)zed_right.getWidth(),
                            CV_8UC4, zed_right.getPtr<sl::uchar1>(sl::MEM::CPU));
            
            cv::Mat left_bgr, right_bgr;
            cv::cvtColor(cv_left, left_bgr, cv::COLOR_BGRA2BGR);
            cv::cvtColor(cv_right, right_bgr, cv::COLOR_BGRA2BGR);
            
            // Create stereo image
            left_bgr.copyTo(stereo_image(cv::Rect(0, 0, cam_width, cam_height)));
            right_bgr.copyTo(stereo_image(cv::Rect(cam_width, 0, cam_width, cam_height)));
            
            // QR code detection (only on left image for efficiency)
            bool qr_detected = false;
            string qr_pc_time, qr_robot_time;

            // Timer-based QR scanning to avoid matching to a later frame
            auto now_time = chrono::steady_clock::now();
            auto ms_since_last_scan = chrono::duration_cast<chrono::milliseconds>(now_time - last_qr_scan_time).count();
            bool should_scan = (ms_since_last_scan >= QR_SCAN_INTERVAL_MS);

            // If recording just started, scan more frequently for the first 3s
            if (recorder_ && recorder_->isRecording() && !was_recording) {
                recording_start_time = chrono::steady_clock::now();
            }
            was_recording = recorder_ && recorder_->isRecording();
            if (was_recording) {
                auto ms_since_rec_start = chrono::duration_cast<chrono::milliseconds>(now_time - recording_start_time).count();
                if (ms_since_rec_start < 3000 && ms_since_last_scan >= 200) {
                    should_scan = true;
                }
            }

            if (should_scan) {
                string qr_data;
                // Optionally downsample to speed up scanning
                cv::Mat scan_img;
                cv::resize(left_bgr, scan_img, cv::Size(), 0.6, 0.6, cv::INTER_LINEAR);
                if (qr_scanner_.scan(scan_img, qr_data)) {
                    try {
                        SimpleJson qr_json = SimpleJson::parse(qr_data);
                        // Expecting fields: PC and Robot
                        if (qr_json.hasKey("PC") && qr_json.hasKey("Robot")) {
                            qr_detected = true;
                            qr_pc_time = qr_json.get("PC");
                            qr_robot_time = qr_json.get("Robot");

                            // Preserve previous values for drift comparison
                            int64_t prev_pc_us = last_qr_pc_us_;
                            int64_t prev_cam_us = last_qr_cam_us_;

                            // Parse PC time (auto-detect ms vs μs) using existing helper
                            int64_t qr_pc_us = 0;
                            try {
                                qr_pc_us = parseTimestampToUs(qr_pc_time);
                            } catch (...) {
                                qr_pc_us = system_ts_us; // fallback
                            }

                            int64_t qr_robot_us = 0;
                            try {
                                qr_robot_us = parseTimestampToUs(qr_robot_time);
                            } catch (...) {
                                qr_robot_us = 0;
                            }

                            // Update persistent QR times using the parsed PC timestamp
                            last_qr_pc_us_ = qr_pc_us;
                            last_qr_robot_us_ = qr_robot_us;
                            last_qr_cam_us_ = camera_ts_us; // HW timestamp of frame which contained QR
                            last_qr_pc_str_ = qr_pc_time;
                            last_qr_robot_str_ = qr_robot_time;

                            qr_detection_count++;
                            last_qr_scan_time = now_time;

                            if (!qr_ever_detected) {
                                qr_ever_detected = true;
                                cout << "[QR] ✅ FIRST QR detected! Sync initialized.\n";
                            } else {
                                cout << "[QR] 🔄 QR re-detected (count: " << qr_detection_count << ") - sync updated\n";
                            }

                            // Drift/offset diagnostics
                            // PC-CAM offset: difference between parsed PC time and camera HW time
                            int64_t pc_cam_offset_ms = (last_qr_pc_us_ - last_qr_cam_us_) / 1000;
                            int64_t sys_pc_offset_ms = (system_ts_us - last_qr_pc_us_) / 1000;
                            cout << "[QR] Sync Detail:\n";
                            cout << "      QR PC time:     " << qr_pc_time << " (" << last_qr_pc_us_ << " us)\n";
                            cout << "      Frame CAM time: " << last_qr_cam_us_ << " us\n";
                            cout << "      Frame SYS time: " << system_ts_us << " us\n";
                            cout << "      PC-CAM offset:  " << pc_cam_offset_ms << " ms\n";
                            cout << "      SYS-PC offset:  " << sys_pc_offset_ms << " ms\n";

                            // Show microsecond precision
                            int64_t us_precision = last_qr_pc_us_ % 1000;
                            if (us_precision == 0) {
                                cout << "      ⚠️ WARNING: QR has only ms precision (no sub-ms data)\n";
                            } else {
                                cout << "      ✅ QR has μs precision (sub-ms: " << us_precision << "μs)\n";
                            }
                            // Compare with previous offset if available
                            if (prev_pc_us > 0 && prev_cam_us > 0) {
                                int64_t old_offset = prev_pc_us - prev_cam_us;
                                int64_t new_offset = last_qr_pc_us_ - last_qr_cam_us_;
                                int64_t offset_drift = llabs(new_offset - old_offset);
                                if (offset_drift > 500000) { // 500ms
                                    cout << "[QR] ⚠️  WARNING: Large QR offset drift detected: " << (offset_drift/1000) << " ms\n";
                                }
                            }
                        }
                    } catch (...) {}
                }
            }
            
            // Stream if enabled
            if (sender_ && sender_->isRunning()) {
                sender_->sendImage(stereo_image);
            }
            
            // Record if enabled
            if (recorder_ && recorder_->isRecording()) {
                recorder_->writeFrame(stereo_image, camera_ts_us, system_ts_us,
                                     qr_detected, qr_pc_time, qr_robot_time,
                                     last_qr_pc_us_, last_qr_robot_us_, last_qr_cam_us_,
                                     last_qr_pc_str_, last_qr_robot_str_);
            }
            
            // Update shared memory
            shm_manager_.updateFrame(stereo_image, camera_ts_us, system_ts_us,
                                     qr_detected, qr_pc_time, qr_robot_time,
                                     last_qr_pc_us_, last_qr_robot_us_, last_qr_cam_us_,
                                     last_qr_pc_str_, last_qr_robot_str_);
            
            // Update status
            updateStatus();
            
            // Preview
            if (show_preview_) {
                cv::Mat preview;
                cv::resize(stereo_image, preview, cv::Size(1280, 480));
                
                // Draw info
                string info_str = "FPS: " + to_string(config_.fps);
                if (sender_ && sender_->isRunning()) {
                    info_str += " | Streaming: " + sender_->getTargetIP();
                }
                if (recorder_ && recorder_->isRecording()) {
                    cv::circle(preview, cv::Point(preview.cols - 30, 30), 15, cv::Scalar(0, 0, 255), -1);
                    info_str += " | REC";
                }
                cv::putText(preview, info_str, cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

                // QR status overlay
                string qr_status;
                cv::Scalar qr_color = cv::Scalar(0, 0, 255);
                if (last_qr_pc_us_ > 0) {
                    int sync_age_sec = static_cast<int>((system_ts_us - last_qr_pc_us_) / 1000000);
                    qr_status = "QR: " + to_string(sync_age_sec) + "s ago [" + to_string(qr_detection_count) + "x]";
                    if (sync_age_sec < 2) qr_color = cv::Scalar(0, 255, 0);
                    else if (sync_age_sec < 10) qr_color = cv::Scalar(0, 255, 255);
                    else qr_color = cv::Scalar(0, 0, 255);
                } else {
                    qr_status = string("QR: WAITING (scan every ") + to_string((float)QR_SCAN_INTERVAL_MS/1000.0f) + "s)";
                    qr_color = cv::Scalar(0, 0, 255);
                }
                cv::putText(preview, qr_status, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, qr_color, 2);

                // Next scan countdown
                int next_scan_ms = QR_SCAN_INTERVAL_MS - (int)ms_since_last_scan;
                if (next_scan_ms < 0) next_scan_ms = 0;
                cv::putText(preview, string("Next scan: ") + to_string(next_scan_ms) + "ms",
                           cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(180, 180, 180), 1);
                
                cv::imshow("ZED Manager Preview", preview);
                char key = cv::waitKey(1);
                if (key == 'q') {
                    running_ = false;
                }
            }

            // Periodic sync/divergence check while recording
            if (recorder_ && recorder_->isRecording() && frame_count % 300 == 0) {
                if (last_qr_pc_us_ > 0 && last_qr_robot_us_ > 0) {
                    int64_t interp_sys = last_qr_robot_us_ + (system_ts_us - last_qr_pc_us_);
                    int64_t interp_cam = last_qr_robot_us_ + (camera_ts_us - last_qr_cam_us_);
                    int64_t diff_us = llabs(interp_sys - interp_cam);
                    if (diff_us > 100000) { // >100ms
                        cout << "[Sync Warning] Interpolation divergence: " << (diff_us/1000) << " ms" << endl;
                    } else {
                        cout << "[Sync] Interpolation divergence OK: " << (diff_us/1000) << " ms" << endl;
                    }
                }
            }
            
            // FPS counter
            frame_count++;
            auto now = chrono::steady_clock::now();
            auto elapsed = chrono::duration_cast<chrono::seconds>(now - fps_time).count();
            if (elapsed >= 5) {
                float fps = (float)frame_count / elapsed;
                cout << "[Stats] FPS: " << fps;
                if (sender_ && sender_->isRunning()) cout << " | Streaming";
                if (recorder_ && recorder_->isRecording()) cout << " | Recording";
                cout << endl;
                frame_count = 0;
                fps_time = now;
            }
        }
        
        cleanup();
    }

private:
    void handleCommand(const string& cmd_str) {
        cout << "[Command] Received: " << cmd_str << endl;
        
        SimpleJson cmd = SimpleJson::parse(cmd_str);
        string action = cmd.get("action");
        map<string, string> response;
        
        if (action == "start_stream") {
            string ip = cmd.get("ip", config_.stream_target_ip);
            int port = cmd.getInt("port", config_.stream_port);
            int quality = cmd.getInt("quality", config_.stream_quality);
            int width = cmd.getInt("width", config_.stream_width);
            int height = cmd.getInt("height", config_.stream_height);
            
            if (ip.empty()) {
                response["status"] = "error";
                response["message"] = "IP address required";
            } else {
                if (startStreaming(ip, port, width, height, quality)) {
                    response["status"] = "ok";
                    response["message"] = "Streaming started";
                    
                    // Send stereo params to XR device
                    map<string, string> stereo_params;
                    stereo_params["focus"] = SimpleJson::formatFloat(config_.stereo_focus);
                    stereo_params["quad"] = SimpleJson::formatFloat(config_.stereo_quad);
                    stereo_params["zoom"] = SimpleJson::formatFloat(config_.stereo_zoom);
                    stereo_params["addFocus"] = config_.stereo_add_focus ? "true" : "false";
                    
                    cout << "[Stereo] Sending params to " << ip << ":" << config_.stereo_params_port << endl;
                    cout << "[Stereo] Payload: " << SimpleJson::stringify(stereo_params) << endl;
                    string result = sendStereoParams(ip, config_.stereo_params_port, stereo_params);
                    cout << "[Stereo] Response: " << result << endl;
                    response["stereo_params_sent"] = (result.find("error") == string::npos) ? "true" : "false";
                } else {
                    response["status"] = "error";
                    response["message"] = "Failed to start streaming";
                }
            }
        }
        else if (action == "stop_stream") {
            stopStreaming();
            response["status"] = "ok";
            response["message"] = "Streaming stopped";
        }
        else if (action == "start_record") {
            string path = cmd.get("path", config_.output_folder);
            string filename = cmd.get("filename", camera_name_);
            
            if (startRecording(path, filename)) {
                response["status"] = "ok";
                response["message"] = "Recording started";
                response["filepath"] = recorder_->getFilepath();
            } else {
                response["status"] = "error";
                response["message"] = "Failed to start recording";
            }
        }
        else if (action == "stop_record") {
            string filepath;
            if (recorder_) {
                filepath = recorder_->getFilepath();
            }
            stopRecording();
            response["status"] = "ok";
            response["message"] = "Recording stopped";
            response["filepath"] = filepath;
        }
        else if (action == "start_preview") {
            show_preview_ = true;
            cv::namedWindow("ZED Manager Preview", cv::WINDOW_NORMAL);
            cv::resizeWindow("ZED Manager Preview", 1280, 480);
            response["status"] = "ok";
            response["message"] = "Preview started";
        }
        else if (action == "stop_preview") {
            show_preview_ = false;
            cv::destroyWindow("ZED Manager Preview");
            response["status"] = "ok";
            response["message"] = "Preview stopped";
        }
        else if (action == "get_status") {
            response["status"] = "ok";
            response["streaming"] = (sender_ && sender_->isRunning()) ? "true" : "false";
            response["recording"] = (recorder_ && recorder_->isRecording()) ? "true" : "false";
            response["preview"] = show_preview_ ? "true" : "false";
            response["control_port"] = to_string(control_server_->getPort());
            response["camera_serial"] = camera_serial_;
            response["camera_name"] = camera_name_;
            
            if (sender_ && sender_->isRunning()) {
                response["stream_ip"] = sender_->getTargetIP();
                response["stream_port"] = to_string(sender_->getTargetPort());
            }
            if (recorder_ && recorder_->isRecording()) {
                response["recording_file"] = recorder_->getFilepath();
                response["frames_recorded"] = to_string(recorder_->getFrameCount());
            }
            if (last_qr_robot_us_ > 0) {
                response["last_qr_robot_time"] = last_qr_robot_str_;
            }
        }
        else if (action == "set_stereo_params") {
            string target_ip = cmd.get("target_ip");
            int target_port = cmd.getInt("target_port", 9004);
            
            if (target_ip.empty()) {
                response["status"] = "error";
                response["message"] = "target_ip required";
            } else {
                map<string, string> payload;
                if (cmd.hasKey("focus")) payload["focus"] = SimpleJson::formatFloat(cmd.getFloat("focus", 0.0f));
                if (cmd.hasKey("quad")) payload["quad"] = SimpleJson::formatFloat(cmd.getFloat("quad", 1.0f));
                if (cmd.hasKey("zoom")) payload["zoom"] = SimpleJson::formatFloat(cmd.getFloat("zoom", 1.0f));
                if (cmd.hasKey("add_focus")) payload["addFocus"] = cmd.getBool("add_focus") ? "true" : "false";
                
                cout << "[Stereo] Sending params: " << SimpleJson::stringify(payload) << endl;
                string result = sendStereoParams(target_ip, target_port, payload);
                if (result.find("error") == string::npos) {
                    response["status"] = "ok";
                    response["message"] = "Stereo params sent";
                    response["device_response"] = result;
                } else {
                    response["status"] = "error";
                    response["message"] = result;
                }
            }
        }
        else if (action == "quit" || action == "shutdown") {
            running_ = false;
            response["status"] = "ok";
            response["message"] = "Shutting down";
        }
        else {
            response["status"] = "error";
            response["message"] = "Unknown action: " + action;
        }
        
        string resp_str = SimpleJson::stringify(response);
        cout << "[Response] " << resp_str << endl;
        control_server_->sendResponse(resp_str);
    }
    
    void handleSharedMemoryCommand() {
        Command* cmd = shm_manager_.getCommand();
        if (!cmd) return;
        
        uint64_t counter = cmd->command_counter.load(std::memory_order_acquire);
        if (counter == cmd->last_processed) return;
        
        switch (cmd->type) {
            case CommandType::CMD_START_PREVIEW:
                show_preview_ = true;
                cv::namedWindow("ZED Manager Preview", cv::WINDOW_NORMAL);
                break;
            case CommandType::CMD_STOP_PREVIEW:
                show_preview_ = false;
                cv::destroyWindow("ZED Manager Preview");
                break;
            case CommandType::CMD_START_RECORDING:
                startRecording(cmd->params.recording.path, cmd->params.recording.filename);
                break;
            case CommandType::CMD_STOP_RECORDING:
                stopRecording();
                break;
            case CommandType::CMD_START_STREAMING:
                startStreaming(cmd->params.streaming.target_ip,
                              cmd->params.streaming.target_port,
                              cmd->params.streaming.width,
                              cmd->params.streaming.height,
                              cmd->params.streaming.jpeg_quality);
                break;
            case CommandType::CMD_STOP_STREAMING:
                stopStreaming();
                break;
            case CommandType::CMD_SHUTDOWN:
                running_ = false;
                break;
            default:
                break;
        }
        
        cmd->last_processed = counter;
    }
    
    bool startStreaming(const string& ip, int port, int width, int height, int quality) {
        if (sender_) sender_->close();
        sender_ = make_unique<UdpImageSender>();
        return sender_->open(ip, port, width, height, quality);
    }
    
    void stopStreaming() {
        if (sender_) {
            sender_->close();
            sender_.reset();
        }
    }
    
    bool startRecording(const string& path, const string& filename) {
        if (!recorder_) {
            recorder_ = make_unique<VideoRecorder>();
        }
        
        // Use defaults if empty
        string rec_path = path.empty() ? config_.output_folder : path;
        string rec_filename = filename.empty() ? camera_name_ : filename;
        
        auto info = zed_.getCameraInformation();
        int width = info.camera_configuration.resolution.width * 2;
        int height = info.camera_configuration.resolution.height;
        int fps = info.camera_configuration.fps;
        
        return recorder_->start(rec_path, rec_filename, width, height, fps, camera_serial_, camera_name_);
    }
    
    void stopRecording() {
        if (recorder_) {
            recorder_->stop();
        }
    }
    
    string sendStereoParams(const string& ip, int port, const map<string, string>& params) {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) return "error: socket creation failed";
        
        struct timeval tv;
        tv.tv_sec = 3;
        tv.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);
        
        if (::connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            ::close(sock);
            return "error: connection failed to " + ip + ":" + to_string(port);
        }
        
        string payload = SimpleJson::stringify(params) + "\n";
        send(sock, payload.c_str(), payload.length(), 0);
        shutdown(sock, SHUT_WR);
        
        char buffer[4096];
        string response;
        int n;
        while ((n = recv(sock, buffer, sizeof(buffer) - 1, 0)) > 0) {
            buffer[n] = '\0';
            response += buffer;
        }
        
        ::close(sock);
        return response.empty() ? "ok" : response;
    }
    
    void updateStatus() {
        auto* status = shm_manager_.getStatus();
        if (!status) return;
        
        status->cameras[0].is_streaming = (sender_ && sender_->isRunning());
        status->cameras[0].is_recording = (recorder_ && recorder_->isRecording());
        status->cameras[0].is_previewing = show_preview_;
        
        if (status->cameras[0].is_streaming && status->cameras[0].is_recording) {
            status->cameras[0].state = CameraState::STATE_STREAMING_RECORDING;
        } else if (status->cameras[0].is_streaming) {
            status->cameras[0].state = CameraState::STATE_STREAMING;
        } else if (status->cameras[0].is_recording) {
            status->cameras[0].state = CameraState::STATE_RECORDING;
        } else if (status->cameras[0].is_previewing) {
            status->cameras[0].state = CameraState::STATE_PREVIEW;
        } else {
            status->cameras[0].state = CameraState::STATE_CONNECTED;
        }
        
        if (recorder_ && recorder_->isRecording()) {
            status->cameras[0].frames_recorded = recorder_->getFrameCount();
            strncpy(status->cameras[0].recording_path, recorder_->getFilepath().c_str(), PATH_LEN - 1);
        }
        
        status->update_counter.fetch_add(1, std::memory_order_release);
    }
    
    void cleanup() {
        auto* status = shm_manager_.getStatus();
        if (status) {
            status->manager_running = false;
        }
        
        if (sender_) sender_->close();
        if (recorder_) recorder_->stop();
        control_server_->stop();
        zed_.close();
        
        if (show_preview_) {
            cv::destroyAllWindows();
        }
        
        cout << "\n[Cleanup] Done." << endl;
    }
    
    ZedConfig config_;
    Camera zed_;
    string camera_serial_;
    string camera_name_;
    
    unique_ptr<UdpImageSender> sender_;
    unique_ptr<VideoRecorder> recorder_;
    unique_ptr<ControlServer> control_server_;
    SharedMemoryManager shm_manager_;
    QRScanner qr_scanner_;
    
    // QR sync state
    int64_t last_qr_pc_us_ = 0;
    int64_t last_qr_robot_us_ = 0;
    int64_t last_qr_cam_us_ = 0;
    string last_qr_pc_str_;
    string last_qr_robot_str_;
    
    atomic<bool> running_;
    bool show_preview_;
};

// ============================================
// Main
// ============================================
void printHelp() {
    cout << "\n=== ZED Camera Manager ===\n";
    cout << "Usage: ./zed_camera_manager [options]\n\n";
    cout << "Options:\n";
    cout << "  --config <file>    Configuration file (default: zed_config.json)\n";
    cout << "  --port <port>      Control server port (0 for auto)\n";
    cout << "  --preview          Enable preview window\n";
    cout << "  --stream <ip>      Start streaming to IP\n";
    cout << "  --help             Show this help\n";
    cout << "\nTCP JSON Commands:\n";
    cout << "  {\"action\": \"start_stream\", \"ip\": \"192.168.0.140\", \"port\": 9003, \"quality\": 50}\n";
    cout << "  {\"action\": \"stop_stream\"}\n";
    cout << "  {\"action\": \"start_record\", \"path\": \"./recordings\", \"filename\": \"video\"}\n";
    cout << "  {\"action\": \"stop_record\"}\n";
    cout << "  {\"action\": \"get_status\"}\n";
    cout << "  {\"action\": \"quit\"}\n";
    cout << endl;
}

int main(int argc, char** argv) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    ZedConfig config;
    string config_file = "";
    
    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_file = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            config.control_port = stoi(argv[++i]);
        } else if (arg == "--preview") {
            config.enable_preview = true;
        } else if (arg == "--stream" && i + 1 < argc) {
            config.stream_target_ip = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            printHelp();
            return 0;
        }
    }
    
    // Load config file - try multiple locations
    vector<string> config_paths = {
        config_file,                    // User specified
        "zed_config.json",              // Current directory
        "../zed_config.json",           // Parent directory (if running from build/)
        "/app/zed_config.json",         // Docker absolute path
    };
    
    bool config_loaded = false;
    for (const auto& path : config_paths) {
        if (!path.empty() && fs::exists(path)) {
            cout << "[Config] Loading: " << path << endl;
            config.load(path);
            config_loaded = true;
            break;
        }
    }
    
    if (!config_loaded) {
        cout << "[Config] No config file found, using default settings" << endl;
    }
    
    printHelp();
    
    ZedCameraManager manager;
    if (!manager.init(config)) {
        return EXIT_FAILURE;
    }
    
    manager.run();
    
    return EXIT_SUCCESS;
}
