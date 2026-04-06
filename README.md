## Honours_SDR

The **Info** directory stores all information from Ancrotek and Luswave, while **src** contains the source code.

---

## Prerequisites
To compile and run this project on Linux, you must have the following libraries installed:
* `libusb-1.0.0-dev`
* `pkg-config`

---

## Compilation
Use the following command to build the demo script. Note that `libusb-1.0` must be passed into the compiler so that it recognizes the library references:

```bash
g++ main.cpp `pkg-config --libs --cflags libusb-1.0` -o main
```

---

## Execution
Because this application interacts directly with USB hardware via **libusb**, it requires elevated permissions to run. Execute the compiled binary using `sudo`:

```bash
sudo ./main
```

> **Note:** If you wish to run the script without `sudo`, you will need to configure custom **udev rules** for your specific SDR device's VID/PID.
