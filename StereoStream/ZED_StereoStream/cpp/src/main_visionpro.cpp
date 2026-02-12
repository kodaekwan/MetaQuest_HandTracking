///////////////////////////////////////////////////////////////////////////
//
// VisionPro Stereo Streaming - UDP JPEG Sender for ZED Camera
// With TCP Control Server for Python Control
// Based on StereoStreamer.py and example_visionpro.py
//
///////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string.h>
#include <string>
#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <sstream>
#include <filesystem>
#include <fstream>

// Network includes
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

// ZED include
#include <sl/Camera.hpp>

// OpenCV include
#include <opencv2/opencv.hpp>

// JSON parsing (simple implementation)
#include <map>
#include <vector>
#include <algorithm>

using namespace std;
using namespace sl;
namespace fs = std::filesystem;

// ============== Simple JSON Parser ==============
class SimpleJson {
public:
    map<string, string> data;
    
    static SimpleJson parse(const string& json_str) {
        SimpleJson result;
        string s = json_str;
        // Remove whitespace and braces
        s.erase(remove(s.begin(), s.end(), ' '), s.end());
        s.erase(remove(s.begin(), s.end(), '\n'), s.end());
        s.erase(remove(s.begin(), s.end(), '\r'), s.end());
        s.erase(remove(s.begin(), s.end(), '\t'), s.end());
        
        if (s.front() == '{') s = s.substr(1);
        if (s.back() == '}') s = s.substr(0, s.length() - 1);
        
        // Parse key-value pairs
        size_t pos = 0;
        while (pos < s.length()) {
            // Find key
            size_t key_start = s.find('"', pos);
            if (key_start == string::npos) break;
            size_t key_end = s.find('"', key_start + 1);
            if (key_end == string::npos) break;
            string key = s.substr(key_start + 1, key_end - key_start - 1);
            
            // Find colon
            size_t colon = s.find(':', key_end);
            if (colon == string::npos) break;
            
            // Find value
            size_t val_start = colon + 1;
            string value;
            
            if (s[val_start] == '"') {
                // String value
                size_t val_end = s.find('"', val_start + 1);
                value = s.substr(val_start + 1, val_end - val_start - 1);
                pos = val_end + 1;
            } else {
                // Number or boolean
                size_t val_end = s.find_first_of(",}", val_start);
                if (val_end == string::npos) val_end = s.length();
                value = s.substr(val_start, val_end - val_start);
                pos = val_end;
            }
            
            result.data[key] = value;
            
            // Skip comma
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
    
    static string stringify(const map<string, string>& obj) {
        stringstream ss;
        ss << "{";
        bool first = true;
        for (const auto& kv : obj) {
            if (!first) ss << ",";
            first = false;
            ss << "\"" << kv.first << "\":";
            // Check if value is a number or boolean
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

// ============== Configuration ==============
struct StreamConfig {
    string target_ip = "";
    int target_port = 9003;
    int width = 640;
    int height = 480;
    int max_payload = 1400;
    int jpeg_quality = 50;
    int fps = 30;
    bool streaming_enabled = false;
};

struct RecordConfig {
    string save_path = ".";
    string filename = "recording";
    bool recording_enabled = false;
    int fps = 30;
};

// ============== Application State ==============
enum class AppState {
    IDLE,           // 대기 상태
    STREAMING,      // 스트리밍 중
    RECORDING,      // 녹화 중
    STREAMING_RECORDING,  // 스트리밍 + 녹화
    STOPPED         // 종료 상태
};

string stateToString(AppState state) {
    switch (state) {
        case AppState::IDLE: return "idle";
        case AppState::STREAMING: return "streaming";
        case AppState::RECORDING: return "recording";
        case AppState::STREAMING_RECORDING: return "streaming_recording";
        case AppState::STOPPED: return "stopped";
        default: return "unknown";
    }
}

// ============== UDP Image Sender ==============
class UdpImageSender {
public:
    UdpImageSender() : sock_fd_(-1), frame_id_(0), connected_(false), stop_flag_(true) {}
    
    ~UdpImageSender() { close(); }

    bool open(const StreamConfig& config) {
        config_ = config;
        
        sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd_ < 0) {
            cerr << "[UDP Error] Failed to create socket" << endl;
            return false;
        }

        int buf_size = 4 * 1024 * 1024;
        setsockopt(sock_fd_, SOL_SOCKET, SO_SNDBUF, &buf_size, sizeof(buf_size));

        memset(&target_addr_, 0, sizeof(target_addr_));
        target_addr_.sin_family = AF_INET;
        target_addr_.sin_port = htons(config_.target_port);
        inet_pton(AF_INET, config_.target_ip.c_str(), &target_addr_.sin_addr);

        int ret = ::connect(sock_fd_, (struct sockaddr*)&target_addr_, sizeof(target_addr_));
        if (ret < 0) {
            cerr << "[UDP Error] Failed to connect" << endl;
            ::close(sock_fd_);
            sock_fd_ = -1;
            return false;
        }
        
        connected_ = true;
        stop_flag_ = false;
        worker_thread_ = thread(&UdpImageSender::workerLoop, this);
        
        cout << "[UDP] Streaming to " << config_.target_ip << ":" << config_.target_port << endl;
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

            if (img.cols != config_.width * 2 || img.rows != config_.height) {
                cv::resize(img, img, cv::Size(config_.width * 2, config_.height));
            }

            cv::Mat img_bgr;
            if (img.channels() == 4) {
                cv::cvtColor(img, img_bgr, cv::COLOR_BGRA2BGR);
            } else {
                img_bgr = img;
            }

            vector<uchar> jpeg_buf;
            vector<int> params = {cv::IMWRITE_JPEG_QUALITY, config_.jpeg_quality};
            if (!cv::imencode(".jpg", img_bgr, jpeg_buf, params)) {
                continue;
            }

            sendPackets(jpeg_buf);
        }
    }

    void sendPackets(const vector<uchar>& data) {
        uint32_t fid = frame_id_++ & 0xFFFFFFFF;
        size_t total_len = data.size();
        uint16_t total_packets = (total_len + config_.max_payload - 1) / config_.max_payload;

        vector<uchar> packet_buf(8 + config_.max_payload);

        for (uint16_t idx = 0; idx < total_packets; idx++) {
            size_t start = idx * config_.max_payload;
            size_t end = min(start + config_.max_payload, total_len);
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

    StreamConfig config_;
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

// ============== Video Recorder ==============
class VideoRecorder {
public:
    VideoRecorder() : is_recording_(false) {}
    
    ~VideoRecorder() { stop(); }
    
    // Generate unique filename (avoid overwriting)
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
    
    bool start(const string& path, const string& filename, int width, int height, int fps) {
        if (is_recording_) {
            stop();
        }
        
        // Ensure directory exists
        fs::create_directories(path);
        
        // Get unique filename
        filepath_ = getUniqueFilename(path, filename, ".mp4");
        
        // Use MJPG codec for high quality, or mp4v
        int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        
        writer_.open(filepath_, fourcc, fps, cv::Size(width, height), true);
        
        if (!writer_.isOpened()) {
            cerr << "[Record Error] Failed to open video writer: " << filepath_ << endl;
            return false;
        }
        
        is_recording_ = true;
        cout << "[Record] Started recording: " << filepath_ << endl;
        return true;
    }
    
    void writeFrame(const cv::Mat& frame) {
        if (!is_recording_ || !writer_.isOpened()) return;
        
        cv::Mat frame_bgr;
        if (frame.channels() == 4) {
            cv::cvtColor(frame, frame_bgr, cv::COLOR_BGRA2BGR);
        } else {
            frame_bgr = frame;
        }
        
        writer_.write(frame_bgr);
    }
    
    void stop() {
        if (is_recording_) {
            writer_.release();
            is_recording_ = false;
            cout << "[Record] Stopped recording: " << filepath_ << endl;
        }
    }
    
    bool isRecording() const { return is_recording_; }
    string getFilepath() const { return filepath_; }

private:
    cv::VideoWriter writer_;
    string filepath_;
    atomic<bool> is_recording_;
};

// ============== TCP Control Server ==============
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
        
        // Get assigned port if port was 0
        socklen_t len = sizeof(addr);
        getsockname(server_fd_, (struct sockaddr*)&addr, &len);
        port_ = ntohs(addr.sin_port);
        
        if (listen(server_fd_, 5) < 0) {
            cerr << "[Control Error] Failed to listen" << endl;
            ::close(server_fd_);
            return false;
        }
        
        // Set non-blocking
        fcntl(server_fd_, F_SETFL, O_NONBLOCK);
        
        running_ = true;
        cout << "\n========================================" << endl;
        cout << "[Control Server] Listening on port: " << port_ << endl;
        cout << "========================================\n" << endl;
        
        return true;
    }
    
    void stop() {
        running_ = false;
        if (server_fd_ >= 0) {
            ::close(server_fd_);
            server_fd_ = -1;
        }
    }
    
    // Non-blocking check for new command
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
        
        // Set timeout for recv
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        
        // Receive command
        char buffer[4096];
        int n = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
        if (n > 0) {
            buffer[n] = '\0';
            command = string(buffer);
            // Store client fd to send response
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

// ============== Main Application ==============
class VisionProApp {
public:
    VisionProApp() : state_(AppState::IDLE), running_(true), show_preview_(true) {}
    
    bool init(int control_port = 0, bool preview = true) {
        show_preview_ = preview;
        
        // Init ZED camera
        sl::InitParameters init_params;
        init_params.sdk_verbose = false;
        init_params.camera_resolution = sl::RESOLUTION::VGA;
        init_params.camera_fps = 30;
        init_params.depth_mode = sl::DEPTH_MODE::NONE;
        init_params.async_grab_camera_recovery = false;

        auto ret = zed_.open(init_params);
        if (ret != ERROR_CODE::SUCCESS) {
            cerr << "[Error] Camera Open: " << toString(ret) << endl;
            return false;
        }

        auto info = zed_.getCameraInformation();
        auto conf = info.camera_configuration;
        cout << "\n=== ZED Camera ===" << endl;
        cout << "Model: " << info.camera_model << endl;
        cout << "Serial: " << info.serial_number << endl;
        cout << "Resolution: " << conf.resolution.width << "x" << conf.resolution.height << endl;
        
        stream_config_.width = 640;
        stream_config_.height = 480;
        stream_config_.fps = 30;
        record_config_.fps = 30;
        
        // Start control server
        control_server_ = make_unique<ControlServer>(control_port);
        if (!control_server_->start()) {
            zed_.close();
            return false;
        }
        
        if (show_preview_) {
            cv::namedWindow("VisionPro Stream", cv::WINDOW_NORMAL);
            cv::resizeWindow("VisionPro Stream", 1280, 480);
        }
        
        return true;
    }
    
    void run() {
        cout << "\n[Ready] Waiting for commands..." << endl;
        cout << "Commands: start_stream, stop_stream, start_record, stop_record, get_status, quit\n" << endl;
        
        Mat zed_left, zed_right;
        cv::Mat stereo_image(stream_config_.height, stream_config_.width * 2, CV_8UC3);
        cv::Mat stereo_raw;  // For recording (original resolution, no compression)
        
        int frame_count = 0;
        auto fps_time = chrono::steady_clock::now();
        
        while (running_) {
            // Handle control commands
            string cmd;
            if (control_server_->pollCommand(cmd)) {
                handleCommand(cmd);
            }
            
            // Grab frame
            auto ret = zed_.grab();
            if (ret != ERROR_CODE::SUCCESS) {
                if (ret != ERROR_CODE::CAMERA_REBOOTING) {
                    cerr << "[Error] Grab: " << toString(ret) << endl;
                }
                this_thread::sleep_for(chrono::milliseconds(10));
                continue;
            }
            
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
            
            // Create stereo image for streaming (resized if needed)
            cv::Mat left_stream, right_stream;
            if (left_bgr.cols != stream_config_.width || left_bgr.rows != stream_config_.height) {
                cv::resize(left_bgr, left_stream, cv::Size(stream_config_.width, stream_config_.height));
                cv::resize(right_bgr, right_stream, cv::Size(stream_config_.width, stream_config_.height));
            } else {
                left_stream = left_bgr;
                right_stream = right_bgr;
            }
            
            left_stream.copyTo(stereo_image(cv::Rect(0, 0, stream_config_.width, stream_config_.height)));
            right_stream.copyTo(stereo_image(cv::Rect(stream_config_.width, 0, stream_config_.width, stream_config_.height)));
            
            // Stream if enabled
            if (sender_ && sender_->isRunning()) {
                sender_->sendImage(stereo_image);
            }
            
            // Record if enabled (use original quality, side-by-side)
            if (recorder_ && recorder_->isRecording()) {
                // Create full resolution side-by-side for recording
                int rec_w = left_bgr.cols * 2;
                int rec_h = left_bgr.rows;
                if (stereo_raw.empty() || stereo_raw.cols != rec_w || stereo_raw.rows != rec_h) {
                    stereo_raw = cv::Mat(rec_h, rec_w, CV_8UC3);
                }
                left_bgr.copyTo(stereo_raw(cv::Rect(0, 0, left_bgr.cols, left_bgr.rows)));
                right_bgr.copyTo(stereo_raw(cv::Rect(left_bgr.cols, 0, right_bgr.cols, right_bgr.rows)));
                recorder_->writeFrame(stereo_raw);
            }
            
            // Update state
            updateState();
            
            // Preview
            if (show_preview_) {
                cv::imshow("VisionPro Stream", stereo_image);
                char key = cv::waitKey(1);
                if (key == 'q') {
                    running_ = false;
                }
            }
            
            // FPS counter
            frame_count++;
            auto now = chrono::steady_clock::now();
            auto elapsed = chrono::duration_cast<chrono::seconds>(now - fps_time).count();
            if (elapsed >= 5) {
                float fps = (float)frame_count / elapsed;
                cout << "[Stats] FPS: " << fps << " | State: " << stateToString(state_) << endl;
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
            string ip = cmd.get("ip","192.168.0.140");
            int port = cmd.getInt("port", 9003);
            int quality = cmd.getInt("quality", 50);
            int width = cmd.getInt("width", 640);
            int height = cmd.getInt("height", 480);
            
            if (ip.empty()) {
                response["status"] = "error";
                response["message"] = "IP address required";
            } else {
                if (sender_) sender_->close();
                
                stream_config_.target_ip = ip;
                stream_config_.target_port = port;
                stream_config_.jpeg_quality = quality;
                stream_config_.width = width;
                stream_config_.height = height;
                
                sender_ = make_unique<UdpImageSender>();
                if (sender_->open(stream_config_)) {
                    response["status"] = "ok";
                    response["message"] = "Streaming started";
                } else {
                    response["status"] = "error";
                    response["message"] = "Failed to start streaming";
                }
            }
        }
        else if (action == "stop_stream") {
            if (sender_) {
                sender_->close();
                sender_.reset();
            }
            response["status"] = "ok";
            response["message"] = "Streaming stopped";
        }
        else if (action == "start_record") {
            string path = cmd.get("path", ".");
            string filename = cmd.get("filename", "recording");
            
            if (!recorder_) {
                recorder_ = make_unique<VideoRecorder>();
            }
            
            // Use camera's native resolution for recording
            auto info = zed_.getCameraInformation();
            int width = info.camera_configuration.resolution.width * 2;  // Side-by-side
            int height = info.camera_configuration.resolution.height;
            
            if (recorder_->start(path, filename, width, height, record_config_.fps)) {
                response["status"] = "ok";
                response["message"] = "Recording started";
                response["filepath"] = recorder_->getFilepath();
            } else {
                response["status"] = "error";
                response["message"] = "Failed to start recording";
            }
        }
        else if (action == "stop_record") {
            if (recorder_) {
                string filepath = recorder_->getFilepath();
                recorder_->stop();
                response["status"] = "ok";
                response["message"] = "Recording stopped";
                response["filepath"] = filepath;
            } else {
                response["status"] = "ok";
                response["message"] = "No active recording";
            }
        }
        else if (action == "get_status") {
            updateState();
            response["status"] = "ok";
            response["state"] = stateToString(state_);
            response["streaming"] = (sender_ && sender_->isRunning()) ? "true" : "false";
            response["recording"] = (recorder_ && recorder_->isRecording()) ? "true" : "false";
            if (recorder_ && recorder_->isRecording()) {
                response["recording_file"] = recorder_->getFilepath();
            }
            response["control_port"] = to_string(control_server_->getPort());
        }
        else if (action == "set_stereo_params") {
            // Send stereo parameters to external device (VisionPro/Quest)
            string target_ip = cmd.get("target_ip","192.1680.");
            int target_port = cmd.getInt("target_port", 9004);
            
            if (target_ip.empty()) {
                response["status"] = "error";
                response["message"] = "target_ip required";
            } else {
                // Build payload for external device
                map<string, string> payload;
                if (cmd.hasKey("focus")) payload["focus"] = to_string(cmd.getFloat("focus", 0.0f));
                if (cmd.hasKey("quad")) payload["quad"] = to_string(cmd.getFloat("quad", 1.0f));
                if (cmd.hasKey("zoom")) payload["zoom"] = to_string(cmd.getFloat("zoom", 1.0f));
                if (cmd.hasKey("add_focus")) payload["addFocus"] = cmd.getBool("add_focus") ? "true" : "false";
                
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
        else if (action == "quit") {
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
    
    // Send stereo parameters to external device via TCP
    string sendStereoParams(const string& ip, int port, const map<string, string>& params) {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) return "error: socket creation failed";
        
        // Set timeout
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
        
        // Send JSON payload
        string payload = SimpleJson::stringify(params) + "\n";
        send(sock, payload.c_str(), payload.length(), 0);
        shutdown(sock, SHUT_WR);
        
        // Receive response
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
    
    void updateState() {
        bool streaming = sender_ && sender_->isRunning();
        bool recording = recorder_ && recorder_->isRecording();
        
        if (streaming && recording) {
            state_ = AppState::STREAMING_RECORDING;
        } else if (streaming) {
            state_ = AppState::STREAMING;
        } else if (recording) {
            state_ = AppState::RECORDING;
        } else {
            state_ = AppState::IDLE;
        }
    }
    
    void cleanup() {
        state_ = AppState::STOPPED;
        
        if (sender_) sender_->close();
        if (recorder_) recorder_->stop();
        control_server_->stop();
        zed_.close();
        
        if (show_preview_) {
            cv::destroyAllWindows();
        }
        
        cout << "\n[Cleanup] Done." << endl;
    }

    Camera zed_;
    unique_ptr<UdpImageSender> sender_;
    unique_ptr<VideoRecorder> recorder_;
    unique_ptr<ControlServer> control_server_;
    
    StreamConfig stream_config_;
    RecordConfig record_config_;
    
    AppState state_;
    atomic<bool> running_;
    bool show_preview_;
};

void printHelp() {
    cout << "\n=== VisionPro Stereo Streaming Server ===\n";
    cout << "Usage: ./ZED_VisionPro_Stream [options]\n\n";
    cout << "Options:\n";
    cout << "  --port <port>      Control server port (0 for auto, default: 0)\n";
    cout << "  --preview          Enable preview window\n";
    cout << "  --help             Show this help\n";
    cout << "\nControl via TCP JSON commands:\n";
    cout << "  {\"action\": \"start_stream\", \"ip\": \"192.168.0.140\", \"port\": 9003, \"quality\": 50}\n";
    cout << "  {\"action\": \"stop_stream\"}\n";
    cout << "  {\"action\": \"start_record\", \"path\": \"./videos\", \"filename\": \"test\"}\n";
    cout << "  {\"action\": \"stop_record\"}\n";
    cout << "  {\"action\": \"get_status\"}\n";
    cout << "  {\"action\": \"set_stereo_params\", \"target_ip\": \"192.168.0.140\", \"focus\": 0.0, \"quad\": 1.8, \"zoom\": 1.0}\n";
    cout << "  {\"action\": \"quit\"}\n";
    cout << endl;
}

int main(int argc, char** argv) {
    int control_port = 0;
    bool show_preview = false;
    
    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            control_port = stoi(argv[++i]);
        } else if (arg == "--preview") {
            show_preview = true;
        } else if (arg == "--help" || arg == "-h") {
            printHelp();
            return 0;
        }
    }
    
    printHelp();
    
    VisionProApp app;
    if (!app.init(control_port, show_preview)) {
        return EXIT_FAILURE;
    }
    
    app.run();
    
    return EXIT_SUCCESS;
}
