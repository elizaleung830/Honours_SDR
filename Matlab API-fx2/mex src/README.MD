# Compile mex files from SDRadar API C++ source in Windows

- Install VS2015 if not so already. `mex -setup C++` command in Matlab should give something like:

    ```matlab
    MEX configured to use 'Microsoft Visual C++ 2015' for C++ language compilation.
    ```

- type command below in Matlab command window:
(dynamic linking to the 64-bit library)

    ``` matlab
    mex -I.\libusb-1.0.21\include\libusb-1.0 -L.\libusb-1.0.21\MS64\dll\ -llibusb-1.0 sdradar_init.cpp

    mex -I.\libusb-1.0.21\include\libusb-1.0 -L.\libusb-1.0.21\MS64\dll\ -llibusb-1.0 sdradar_get_data.cpp

    mex -I.\libusb-1.0.21\include\libusb-1.0 -L.\libusb-1.0.21\MS64\dll\ -llibusb-1.0 sdradar_send_data.cpp
    ```

- Keep the corresponding `libusb-1.0.dll` in the same directory as the mex functions.

# Compile mex files from SDRadar API C++ source in macOS

- Install Xcode and open it up once to initialize things if not so already. `mex -setup C++` command in Matlab should give something like:

    ```matlab
    MEX configured to use 'Xcode Clang++' for C++ language compilation.
    ```

- type command below in Matlab command window:
(dynamic linking)

    ```matlab
    mex -L/usr/local/lib -lusb-1.0 sdradar_init.cpp
    mex -L/usr/local/lib -lusb-1.0 sdradar_send_data.cpp
    mex -L/usr/local/lib -lusb-1.0 sdradar_get_data.cpp
    ```

More on Matlab C/C++ API can be found in:
<https://www.mathworks.com/help/matlab/call-mex-files-1.html>
