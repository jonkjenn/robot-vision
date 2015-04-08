#include "easylogging++.h"
#include "controller.hpp"
#define ELPP_CUSTOM_COUT std::cerr

using namespace std;

int main(int argc, char** argv)
{
    vector<string> args(argv, argv+argc);
    Controller controller{args};

    controller.driveForward(110,3000);

    string input;
    while(true)
    {
        cout << "test" << endl;

        cin >> input;

        cout << input;
    };

    return 0;
}

