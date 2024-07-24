#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 1) return control_points[0];

    std::vector<cv::Point2f> new_points;
    for (int i = 0; i < control_points.size() - 1; i++) {
        auto p0 = control_points[i];
        auto p1 = control_points[i + 1];
        new_points.push_back(cv::Point2f((1 - t) * p0.x + t * p1.x, (1 - t) * p0.y + t * p1.y));
    }

    return recursive_bezier(new_points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.

    cv::Point2f lastPoint;

    std::vector<cv::Point2f> aa = {cv::Point2f(-1.f, -1.f), cv::Point2f(-1.f, 0.f), cv::Point2f(-1.f, 1.f), cv::Point2f(0.f, -1.f), cv::Point2f(0.f, 0.f), cv::Point2f(0.f, 1.f), cv::Point2f(1.f, -1.f), cv::Point2f(1.f, 0.f), cv::Point2f(1.f, 1.f)};

    for (float t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);

        float px = point.x;
        float py = point.y;
        float cx = floor(px);
        float cy = floor(py);
        // anti antialias 1
        // 9 pixels
        for (cv::Point2f &aa_point : aa) {
            float ax = aa_point.x + cx + 0.5f;
            float ay = aa_point.y + cy + 0.5f;

            float d = sqrt((px - ax) * (px - ax) + (py - ay) * (py - ay));
            float ratio = 1 - sqrt(2) * d / 3;

            window.at<cv::Vec3b>(ay, ax)[1] = std::max(static_cast<uchar>(ratio * 255), window.at<cv::Vec3b>(ay, ax)[1]);
        }

        // anti antialias
        // 4 pixels
//        if (t == 0.0) {
//            lastPoint = point;
//            window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
//        }
//        else {
//            float lsx = lastPoint.x;
//            float lsy = lastPoint.y;
//            float lscx = floor(lsx);
//            float lscy = floor(lsy);
//
//            if (lscx == cx && lscy == cy)
//
//
//            lastPoint = point;
//        }
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
