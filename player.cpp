#include <player.h>
#include <string>

using namespace cv;
using namespace std;

namespace player{

Size res = Size(1600,900);

int counter = 0;

void create_windows(int count, Size size)
{
    int columns = 1600/size.width;
    int row = 0;
    for(int i=0;i<count;i++)
    {
        if(i>0 && i%columns==0){row++;}
        namedWindow("nw" + to_string(i), WINDOW_AUTOSIZE);
        moveWindow("nw" + to_string(i), (i%columns)*size.width, row*size.height);
    }
}

void show_frame(const Mat &frame)
{
    imshow("nw" + to_string(counter), frame);
    counter++;
}

void loop()
{
    counter = 0;
}

}
