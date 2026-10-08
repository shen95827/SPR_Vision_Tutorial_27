#include "io/my_camera.hpp"
#include "opencv2/opencv.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include <spdlog/spdlog.h>
#include <chrono>
#include <vector>
#include <list>

int main(int argc, char** argv) {
    // 解析命令行参数，支持显式指定两个yaml路径，默认读configs下的配置
    std::string camera_cfg_path = "configs/camera.yaml";
    std::string yolo_cfg_path = "configs/yolo.yaml";
    if (argc >= 3) {
        camera_cfg_path = argv[1];
        yolo_cfg_path = argv[2];
    }

    // ---------------- 第一处TODO：初始化相机和YOLO类 ----------------
    io::Camera camera;
    if (!camera.Open(camera_cfg_path)) {
        SPDLOG_ERROR("相机初始化失败，程序退出");
        return -1;
    }

    auto yolo = std::make_unique<auto_aim::YOLO>(yolo_cfg_path, true);
    

    SPDLOG_INFO("识别流水线初始化完成，开始运行");

    // FPS统计变量，用滑窗平均计算帧率
    int frame_cnt = 0;
    auto last_tick = std::chrono::steady_clock::now();

    while (true) {
        cv::Mat img;
        uint64_t timestamp;
        // 读取相机帧
        if (!camera.Read(img, timestamp)) {
            SPDLOG_WARN("读取图像失败，跳过当前帧");
            continue;
        }

        // ---------------- 第二处TODO：调用YOLO识别装甲板 ----------------
        std::list<auto_aim::Armor> armors = yolo->detect(img);

        // ---------------- 第三处TODO：画装甲板角点、显示图像 ----------------
        // 把所有识别到的装甲板四个角点画到图上
        for (const auto& armor : armors) {
            tools::draw_points(img, armor.points);
        }

        // 统计FPS，每30帧打印一次平均帧率
        frame_cnt++;
        auto now_tick = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now_tick - last_tick).count();
        if (duration > 1000) {
            float fps = frame_cnt * 1000.0f / duration;
            SPDLOG_INFO("当前FPS: {:.1f} 识别到装甲板数量: {}", fps, armors.size());
            frame_cnt = 0;
            last_tick = now_tick;
        }

        // 显示画面，按q键退出程序
        cv::imshow("Armor Detection", img);
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
            SPDLOG_INFO("用户按下q键，程序正常退出");
            break;
        }
    }

    // 正常退出，释放所有资源
    camera.Close();
    return 0;
}
