#include <opencv2/core/types.hpp>

class Cone
{
public:
    cv::Point position;
    cv::Rect rect;

    Cone()
    {
    }

    Cone(cv::Rect rect, int x, int y)
    {
        this->rect = rect;
        this->position = cv::Point(x, y);
    }
};
