#include "Demo.hpp"
#include<iostream>
using namespace std;
int main(int argc, char* argv[]) {
    demo Demo;

    // process command line options

    Demo.setup();

    if (Demo.isVideo()) {
        cout << "Processing video" << endl;

        // setup image source, window for drawing, serial port...
        Demo.setupVideo();

        // the actual processing loop where tags are detected and visualized
        Demo.loop();

    } else {
        cout << "Processing image" << endl;

        // process single image
        Demo.loadImages();

    }

    return 0;
}
