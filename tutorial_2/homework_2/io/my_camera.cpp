#include "io/my_camera.hpp"
#include "hikrobot/include/MvCameraControl.h"
#include <spdlog/spdlog.h>
#include <unordered_map>

namespace io {

// ==================== 海康 Bayer -> BGR 转换（照搬 example.cpp） ====================
static cv::Mat transfer(MV_FRAME_OUT& raw) {
    MV_CC_PIXEL_CONVERT_PARAM cvt_param;
    cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight),
                CV_8U, raw.pBufAddr);
    cvt_param.nWidth = raw.stFrameInfo.nWidth;
    cvt_param.nHeight = raw.stFrameInfo.nHeight;
    cvt_param.pSrcData = raw.pBufAddr;
    cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
    cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;
    cvt_param.pDstBuffer = img.data;
    cvt_param.nDstBufferSize = img.total() * img.elemSize();
    cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

    auto pixel_type = raw.stFrameInfo.enPixelType;
    static const std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
    cv::cvtColor(img, img, type_map.at(pixel_type));
    return img;
}

// ==================== HikCamera 实现 ====================
HikCamera::HikCamera() = default;

HikCamera::~HikCamera() {
    Close();
}

bool HikCamera::Open(const YAML::Node& cfg) {
    MV_CC_DEVICE_INFO_LIST device_list;
    int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK || device_list.nDeviceNum == 0) {
        SPDLOG_ERROR("海康相机枚举失败或未找到设备, ret={:#x}", ret);
        return false;
    }

    // 优先按 vid_pid 匹配（yaml 里形如 "2bdf:0001"）
    int target_index = 0;
    if (cfg["vid_pid"]) {
        std::string vid_pid = cfg["vid_pid"].as<std::string>();
        // 拆出 "2bdf" 和 "0001"
        unsigned int vid = std::stoul(vid_pid.substr(0, 4), nullptr, 16);
        unsigned int pid = std::stoul(vid_pid.substr(5, 4), nullptr, 16);
        for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
            auto* info = device_list.pDeviceInfo[i];
            if (info->nTLayerType == MV_USB_DEVICE) {
                if (info->SpecialInfo.stUsb3VInfo.idVendor == vid &&
                    info->SpecialInfo.stUsb3VInfo.idProduct == pid) {
                    target_index = i;
                    break;
                }
            }
        }
    }

    MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[target_index]);
    if (handle_ == nullptr) {
        SPDLOG_ERROR("海康相机创建句柄失败");
        return false;
    }
    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) {
        SPDLOG_ERROR("海康相机打开失败, ret={:#x}", ret);
        return false;
    }

    // 设置相机参数（对应 camera.yaml）
    MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
    if (cfg["exposure_ms"]) {
        MV_CC_SetFloatValue(handle_, "ExposureTime", cfg["exposure_ms"].as<float>() * 1000.0f);
    }
    if (cfg["gain"]) {
        MV_CC_SetFloatValue(handle_, "Gain", cfg["gain"].as<float>());
    }
    MV_CC_SetFrameRate(handle_, 60);

    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) {
        SPDLOG_ERROR("海康相机开始取流失败, ret={:#x}", ret);
        return false;
    }
    is_grabbing_ = true;
    SPDLOG_INFO("海康相机打开成功");
    return true;
}

bool HikCamera::Read(cv::Mat& image, uint64_t& timestamp) {
    if (!is_grabbing_ || handle_ == nullptr) return false;

    MV_FRAME_OUT raw;
    int ret = MV_CC_GetImageBuffer(handle_, &raw, 100);
    if (ret != MV_OK) return false;

    image = transfer(raw);
    timestamp = raw.stFrameInfo.nDevTimeStampHigh;
    MV_CC_FreeImageBuffer(handle_, &raw);
    return true;
}

void HikCamera::Close() {
    if (handle_ == nullptr) return;
    if (is_grabbing_) {
        MV_CC_StopGrabbing(handle_);
        is_grabbing_ = false;
    }
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
}

// ==================== UsbCamera 实现（虚拟机可跑，方便无硬件自测） ====================
bool UsbCamera::Open(const YAML::Node& cfg) {
    int index = cfg["index"] ? cfg["index"].as<int>() : 0;
    int width = cfg["width"] ? cfg["width"].as<int>() : 640;
    int height = cfg["height"] ? cfg["height"].as<int>() : 480;

    cap_.open(index);
    if (!cap_.isOpened()) {
        SPDLOG_ERROR("USB 相机打开失败, index={}", index);
        return false;
    }
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    frame_count_ = 0;
    SPDLOG_INFO("USB 相机打开成功: {}x{}", width, height);
    return true;
}

bool UsbCamera::Read(cv::Mat& image, uint64_t& timestamp) {
    if (!cap_.isOpened()) return false;
    if (!cap_.read(image) || image.empty()) return false;
    timestamp = cv::getTickCount() / (cv::getTickFrequency() / 1e6);
    frame_count_++;
    return true;
}

void UsbCamera::Close() {
    if (cap_.isOpened()) cap_.release();
}

// ==================== Camera 外观类实现 ====================
Camera::Camera() = default;

Camera::~Camera() {
    Close();
}

bool Camera::Open(const std::string& config_path) {
    YAML::Node cfg = YAML::LoadFile(config_path);
    flip_code_ = cfg["flip_code"] ? cfg["flip_code"].as<int>() : -1;

    std::string type = cfg["camera_type"] ? cfg["camera_type"].as<std::string>() : "usb";
    if (type == "hik") {
        impl_ = std::make_unique<HikCamera>();
    } else {
        impl_ = std::make_unique<UsbCamera>();
    }
    return impl_->Open(cfg);
}

bool Camera::Read(cv::Mat& image, uint64_t& timestamp) {
    if (!impl_) return false;
    if (!impl_->Read(image, timestamp)) return false;
    if (flip_code_ != -1) {
        cv::flip(image, image, flip_code_);
    }
    return true;
}

void Camera::Close() {
    if (impl_) {
        impl_->Close();
        impl_.reset();
    }
}

} // namespace io
