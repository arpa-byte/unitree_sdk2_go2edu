#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <signal.h>
#include <string>
#include <vector>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>

// Unitree SDK Headers
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/robot/go2/sport/sport_client.hpp"
#include "unitree/idl/go2/Go2FrontVideoData_.hpp" 

// OpenCV Headers for image processing
#include <opencv2/opencv.hpp>

// ONNX Runtime Headers for AI model inference
#include <onnxruntime_cxx_api.h>

// --- Topic Definition ---
#define TOPIC_VIDEO "rt/video_front"

// --- Global State and Control Variables ---
std::atomic<bool> running(true);
unitree::robot::go2::SportClient* sport_client_ptr = nullptr;
struct termios old_tio, new_tio;

// --- Thread-Safe Shared Data ---
std::mutex frame_mutex;
cv::Mat shared_frame;

std::mutex gesture_mutex;
std::string detected_gesture = "No Gesture";

// --- Function Declarations ---
void sigint_handler(int sig);
void set_terminal_mode();
void restore_terminal_mode();
std::string classify_gesture_from_keypoints(const std::vector<cv::Point3f>& keypoints);
void run_inference_loop(const std::string& model_path);
void VideoFrameCallback(const void* message);

// --- Main Application ---
int main(int argc, char** argv) {
    unitree::robot::ChannelFactory::Instance()->Init(0);
    unitree::robot::go2::SportClient sport_client;
    sport_client.SetTimeout(10.0f);
    sport_client.Init();
    sport_client_ptr = &sport_client;

    signal(SIGINT, sigint_handler);
    set_terminal_mode();

    auto video_subscriber = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::Go2FrontVideoData_>>(TOPIC_VIDEO);
    video_subscriber->InitChannel(VideoFrameCallback, 1);
    
    std::cout << "Commanding robot to initial Sit pose..." << std::endl;
    sport_client.Sit();
    sleep(2);

    std::string model_path = "/home/arpan/unitree2/handmodel/best.onnx"; 
    std::thread inference_thread(run_inference_loop, model_path);

    std::cout << "\nGesture Control Initialized." << std::endl;
    std::cout << "----------------------------------" << std::endl;
    std::cout << "[Enter]: Execute command for the detected gesture." << std::endl;
    std::cout << "[Spacebar]: Stop all motion." << std::endl;
    std::cout << "[Ctrl+C]: Exit." << std::endl;
    std::cout << "----------------------------------" << std::endl;
    
    std::string last_displayed_gesture = "";

    while (running) {
        std::string current_gesture;
        {
            std::lock_guard<std::mutex> lock(gesture_mutex);
            current_gesture = detected_gesture;
        }

        if (current_gesture != last_displayed_gesture) {
            std::cout << "\rDetected: " << current_gesture << " (Press Enter to confirm)    " << std::flush;
            last_displayed_gesture = current_gesture;
        }

        char c = 0;
        if (read(STDIN_FILENO, &c, 1) > 0) {
            if (c == '\n') {
                std::cout << "\nExecuting command for gesture: " << current_gesture << std::endl;
                if (current_gesture == "Thumbs Up") {
                    sport_client.StandUp();
                } else if (current_gesture == "Thumbs Down") {
                    sport_client.Sit();
                } else {
                    std::cout << "No valid action mapped for this gesture." << std::endl;
                }
            } else if (c == ' ') {
                std::cout << "\nCommand: Stop Motion!" << std::endl;
                sport_client.StopMove();
            }
        }
        usleep(50000);
    }

    inference_thread.join();
    restore_terminal_mode();
    std::cout << "Exiting." << std::endl;
    return 0;
}

// --- Function Implementations ---

void VideoFrameCallback(const void* message) {
    auto video_msg = static_cast<const unitree_go::msg::dds_::Go2FrontVideoData_*>(message);
    
    // **THE DEFINITIVE FIX: Access the 720p video stream using the correct function.**
    const auto& frame_data = video_msg->video720p();

    if (!frame_data.empty()) {
        try {
            cv::Mat raw_data(1, frame_data.size(), CV_8UC1, (void*)frame_data.data());
            cv::Mat frame = cv::imdecode(raw_data, cv::IMREAD_COLOR);
            if (!frame.empty()) {
                std::lock_guard<std::mutex> lock(frame_mutex);
                shared_frame = frame.clone();
            }
        } catch (const cv::Exception& e) {
            std::cerr << "OpenCV exception in video callback: " << e.what() << std::endl;
        }
    }
}

void run_inference_loop(const std::string& model_path) {
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "gesture-inference");
    Ort::SessionOptions session_options;
    Ort::Session session(env, model_path.c_str(), session_options);

    const int64_t input_height = 640;
    const int64_t input_width = 640;
    
    while(running) {
        cv::Mat current_frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (shared_frame.empty()) {
                usleep(50000);
                continue;
            }
            current_frame = shared_frame.clone();
        }

        cv::Mat resized_frame;
        cv::resize(current_frame, resized_frame, cv::Size(input_width, input_height));
        cv::cvtColor(resized_frame, resized_frame, cv::COLOR_BGR2RGB);

        std::vector<float> input_tensor_values(3 * input_height * input_width);
        cv::Mat float_frame;
        resized_frame.convertTo(float_frame, CV_32F, 1.0 / 255.0);
        
        std::vector<cv::Mat> channels(3);
        cv::split(float_frame, channels);
        memcpy(input_tensor_values.data(), channels[0].data, channels[0].total() * channels[0].elemSize());
        memcpy(input_tensor_values.data() + channels[0].total(), channels[1].data, channels[1].total() * channels[1].elemSize());
        memcpy(input_tensor_values.data() + channels[0].total() * 2, channels[2].data, channels[2].total() * channels[2].elemSize());
       
        std::vector<int64_t> input_shape = {1, 3, input_height, input_width};
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_tensor_values.data(), input_tensor_values.size(), input_shape.data(), input_shape.size());

        const char* input_names[] = {"images"};
        const char* output_names[] = {"output0"};
        auto output_tensors = session.Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

        const float* raw_output = output_tensors[0].GetTensorData<float>();
        auto output_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
        const int num_detections = output_shape[2];
        const int num_keypoints = 21;

        float max_conf = 0.0f;
        int best_detection_idx = -1;

        for (int i = 0; i < num_detections; ++i) {
            float confidence = raw_output[4 * num_detections + i];
            if (confidence > max_conf) {
                max_conf = confidence;
                best_detection_idx = i;
            }
        }

        std::string gesture = "No Hand Detected";
        if (max_conf > 0.5f) {
            std::vector<cv::Point3f> keypoints;
            for (int j = 0; j < num_keypoints; ++j) {
                float x = raw_output[(5 + j * 3) * num_detections + best_detection_idx];
                float y = raw_output[(5 + j * 3 + 1) * num_detections + best_detection_idx];
                float vis = raw_output[(5 + j * 3 + 2) * num_detections + best_detection_idx];
                keypoints.push_back(cv::Point3f(x, y, vis));
            }
            gesture = classify_gesture_from_keypoints(keypoints);
        }
        
        {
            std::lock_guard<std::mutex> lock(gesture_mutex);
            detected_gesture = gesture;
        }
        
        usleep(50000);
    }
}

std::string classify_gesture_from_keypoints(const std::vector<cv::Point3f>& keypoints) {
    if (keypoints.size() != 21) return "Gesture Not Clear";
    float thumb_tip_y = keypoints[4].y; float thumb_pip_y = keypoints[3].y;
    float index_tip_y = keypoints[8].y; float index_mcp_y = keypoints[5].y;
    float middle_tip_y = keypoints[12].y; float middle_mcp_y = keypoints[9].y;
    float ring_tip_y = keypoints[16].y; float ring_mcp_y = keypoints[13].y;
    float pinky_tip_y = keypoints[20].y; float pinky_mcp_y = keypoints[17].y;
    bool fingers_curled = (index_tip_y > index_mcp_y && middle_tip_y > middle_mcp_y && ring_tip_y > ring_mcp_y && pinky_tip_y > pinky_mcp_y);
    bool thumbs_up = (thumb_tip_y < thumb_pip_y) && fingers_curled;
    bool thumbs_down = (thumb_tip_y > thumb_pip_y) && fingers_curled; 
    if (thumbs_up) return "Thumbs Up";
    if (thumbs_down) return "Thumbs Down";
    return "Gesture Not Clear";
}

void sigint_handler(int sig) { if(sport_client_ptr) sport_client_ptr->StopMove(); running = false; }
void set_terminal_mode() { tcgetattr(STDIN_FILENO, &old_tio); new_tio = old_tio; new_tio.c_lflag &= (~ICANON & ~ECHO); tcsetattr(STDIN_FILENO, TCSANOW, &new_tio); fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); }
void restore_terminal_mode() { tcsetattr(STDIN_FILENO, TCSANOW, &old_tio); }