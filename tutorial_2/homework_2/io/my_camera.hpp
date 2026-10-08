#pragma once
#include <string>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

namespace io {

class CameraBase {
public:
    virtual ~CameraBase() = default;
    virtual bool Open(const YAML::Node& cfg) = 0;
    virtual bool Read(cv::Mat& image, uint64_t& timestamp) = 0;
    virtual void Close() = 0;
};

class HikCamera : public CameraBase {
public:
    HikCamera();
    ~HikCamera() override;
    bool Open(const YAML::Node& cfg) override;
    bool Read(cv::Mat& image, uint64_t& timestamp) override;
    void Close() override;
private:
    void* handle_ = nullptr;
    bool is_grabbing_ = false;
};

class UsbCamera : public CameraBase {
public:
    UsbCamera() = default;
    bool Open(const YAML::Node& cfg) override;
    bool Read(cv::Mat& image, uint64_t& timestamp) override;
    void Close() override;
private:
    cv::VideoCapture cap_;
    int64_t frame_count_ = 0;
};

class Camera {
public:
    Camera();
    ~Camera();
    bool Open(const std::string& config_path);
    bool Read(cv::Mat& image, uint64_t& timestamp);
    void Close();
private:
    std::unique_ptr<CameraBase> impl_;
    int flip_code_ = -1;
};

} // namespace io
