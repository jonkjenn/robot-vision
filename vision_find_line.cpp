using namespace cv;
using namespace std;

Vision_find_line::update(Mat &image)
{
    Mat dst;
    Rect roi{image.cols - 11, 0,image.cols,image.rows};
    cvtColor(image(roi),dst,CV_BGR2GRAY);
}
