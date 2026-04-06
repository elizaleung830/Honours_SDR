#Honours_SDR
The Info directory stored all information from Ancrotek and Luswave, while src contains the source code.
## Prerequisites
To compile and run this project on Linux, you must have the libusb-1.0.0-dev and pkg-config library installed.
## Compilation
Follow command is used to build the demo scirpt, libusb-1.0 needed to be passed into the compiler so that it recongise the reference of the libary
`g++ main.cpp `pkg-config --libs --cflags libusb-1.0` -o main`
## Execution
Because this application interacts directly with USB hardware via libusb, it requires elevated permissions to run. Execute the compiled binary using sudo:
`sudo ./main`
(Note: If you wish to run the script without sudo, you will need to configure custom udev rules for your specific SDR device's VID/PID).
