#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include "AdaptiveMesh.h"
#include "MVCCloner.h"

DEFINE_bool(interactive, false, "Interactive mode switch(true for GUI, false for CLI)");
DEFINE_string(src, "", "Source img file location");
DEFINE_string(dst, "", "Target img file location");
DEFINE_string(mask, "", "Mask file location");
DEFINE_string(out, "", "Output img file location");
DEFINE_int32(x, 0, "Position x to put source img");
DEFINE_int32(y, 0, "Position y to put source img");

namespace mask {
    constexpr char mask_window_name[] = "Mask Selection(Enter to confirm, c to clear)";
    constexpr char target_pos_sel_window[] = "Target position selection";

    cv::Point prev_point(-1, -1);
    cv::Point sel_point(-1, -1);
    cv::Mat mask;
    cv::Mat orig_img;
    cv::Mat orig_dst_img;
    cv::Mat dst_img;

    short mode = 0;

    void onMouse(int evt, int x, int y, int flags, void *param) {
        if (evt == CV_EVENT_LBUTTONUP || !(flags & CV_EVENT_FLAG_LBUTTON))
            prev_point = cv::Point(-1, -1);
        else if (evt == CV_EVENT_LBUTTONDOWN)
            prev_point = cv::Point(x, y);
        else if (evt == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON)) {
            cv::Point curr(x, y);
            if (prev_point.x > 0) {
                cv::line(mask, prev_point, curr, cv::Scalar::all(255));
                cv::line(orig_img, prev_point, curr, cv::Scalar::all(255));
            }
            prev_point = curr;
            cv::imshow(mask_window_name, orig_img);
        }
    }

    void onMouseSelDest(int evt, int x, int y, int flags, void *param) {
        if (evt == CV_EVENT_LBUTTONDOWN || evt == CV_EVENT_RBUTTONDOWN) {
            if(evt == CV_EVENT_RBUTTONDOWN)
            {
                ++mode;
                mode %= 4;
            }
            dst_img = orig_dst_img.clone();
            cv::drawMarker(dst_img, cv::Point(x, y), cv::Scalar(0, 20, 255));
            switch(mode)
            {
                case 0:
                    sel_point = cv::Point(x,y);
                    cv::rectangle(dst_img, sel_point, cv::Point(x + orig_img.cols, y + orig_img.rows), cv::Scalar(0, 100, 255));
                    break;
                case 1:
                    sel_point = cv::Point(x - orig_img.cols, y);
                    cv::rectangle(dst_img, sel_point, cv::Point(x , y + orig_img.rows), cv::Scalar(0, 100, 255));
                    break;
                case 2:
                    sel_point = cv::Point(x - orig_img.cols, y - orig_img.rows);
                    cv::rectangle(dst_img, sel_point, cv::Point(x, y), cv::Scalar(0, 100, 255));
                    break;
                case 3:
                    sel_point = cv::Point(x, y - orig_img.rows);
                    cv::rectangle(dst_img, sel_point, cv::Point(x + orig_img.cols, y), cv::Scalar(0, 100, 255));
                    break;
            }
            cv::imshow(target_pos_sel_window, dst_img);
        }
    }
}

int main(int argc, char **argv) {
    gflags::SetUsageMessage(
            "MVC Implementation by zx1239856. \nNote: Some CLI options not available when Interactive mode is on");
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    google::LogToStderr();

    std::vector<mesh::Point> boundary;
    if (FLAGS_interactive)
        CHECK(FLAGS_src != "" && FLAGS_dst != "") << "Source and target should be specified";
    else
        CHECK(FLAGS_src != "" && FLAGS_dst != "" && FLAGS_mask != "") << "Source, target and mask should be specified";
    cv::Mat src_img = cv::imread(FLAGS_src);
    cv::Mat dst_img = cv::imread(FLAGS_dst);
    CHECK(src_img.data != nullptr) << "Source img load failed";
    CHECK(dst_img.data != nullptr) << "Target img load failed";
    if (FLAGS_x > dst_img.cols) {
        LOG(WARNING) << "Destination x invalid, set to default";
        FLAGS_x = 0;
    }
    if (FLAGS_y > dst_img.rows) {
        LOG(WARNING) << "Destination y invalid, set to default";
        FLAGS_y = 0;
    }

    mask::orig_img = src_img.clone();

    if (!FLAGS_mask.empty()) {
        cv::Mat mask_img;
        mask_img = cv::imread(FLAGS_mask, 0);
        CHECK(mask_img.data != nullptr) << "Mask load failed";
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(mask_img, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        CHECK(contours.size() > 0) << "Error calculating contours";
        for (auto &it: contours[0]) {
            boundary.emplace_back(mesh::Point(it.x, it.y));
        }
    } else {
        cv::Mat empty_mask = cv::Mat(src_img.size(), CV_8U, cv::Scalar::all(0));
        mask::mask = empty_mask.clone();
        cv::namedWindow(mask::mask_window_name);
        cv::setMouseCallback(mask::mask_window_name, &mask::onMouse, 0);
        while (true) {
            cv::imshow(mask::mask_window_name, mask::orig_img);
            int code = cv::waitKey(0);
            if (code == 10 || code == 13) // `enter` key
            {
                LOG(INFO) << "Begin to process mask";
                std::vector<std::vector<cv::Point> > contours;
                cv::findContours(mask::mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
                mask::mask = empty_mask;
                cv::drawContours(mask::mask, contours, -1, cv::Scalar(255), CV_FILLED);
                cv::erode(mask::mask, mask::mask, cv::Mat()); // erode extra lines
                contours.clear();
                cv::findContours(mask::mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
                CHECK(contours.size() > 0) << "Error calculating contours";
                for (auto &it: contours[0]) {
                    boundary.emplace_back(mesh::Point(it.x, it.y));
                }
                break;
            } else if (code == 'c') // clear selection
            {
                mask::orig_img = src_img.clone();
                mask::mask = empty_mask.clone();
            }
        }
        cv::destroyWindow(mask::mask_window_name);
    }
    if (FLAGS_interactive) {
        mask::orig_dst_img = dst_img;
        mask::dst_img = dst_img.clone();
        cv::namedWindow(mask::target_pos_sel_window);
        cv::setMouseCallback(mask::target_pos_sel_window, &mask::onMouseSelDest, 0);
        while (true) {
            cv::imshow(mask::target_pos_sel_window, dst_img);
            int code = cv::waitKey(0);
            if (code == 10 || code == 13) // `enter` key
                break;
        }
        FLAGS_x = mask::sel_point.x;
        FLAGS_y = mask::sel_point.y;
        cv::destroyWindow(mask::target_pos_sel_window);

    }
    mesh::AdaptiveMesh adp_mesh(boundary);
    adp_mesh.calcPoints();
    cv::imshow("Mesh", adp_mesh.visualize(src_img.size()));
    cv::waitKey(10);
    MVCCloner cloner(adp_mesh, src_img, dst_img);
    cloner.startClone(FLAGS_x, FLAGS_y);
    auto result = cloner.getResult();
    if (!FLAGS_out.empty()) {
        cv::imwrite(FLAGS_out, result);
        LOG(INFO) << "Writing to file " << FLAGS_out;
    }
    cv::imshow("Final result", result);
    cv::waitKey();
    cv::destroyAllWindows();
    return 0;
}
