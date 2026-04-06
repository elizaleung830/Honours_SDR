// bytes_sent = sdr_send_data(outdata);

// Standard Includes
#include <iostream>
#include <stdexcept>

// Include Matlab
#include "mex.h"
#include "matrix.h"

// Include LibUSB
#include "libusb.h"

// Setup Cypress VID/PID
#define SDR_USB_VID 0x04b4
#define SDR_USB_PID 0x8613

// Setup LibUSB Endpoints
#define SDR_ENDPOINT_RX 0x86                    // Endpoint - Device to PC
#define SDR_ENDPOINT_TX 0x02                    // Endpoint - PC to Device

// Define Timeouts
#define SDR_TIMEOUT_FIRMWARE_WRITE_MS 1000      // Firmware write timeout
#define SDR_TIMEOUT_DEVICE_RESET_MS 1000        // Device reset timeout
#define SDR_TIMEOUT_READ_DATA_MS 1000           // Read data from device timeout
#define SDR_TIMEOUT_READ_INFINITE 0             // Read data infinetely


int sdradar_send_data(unsigned char* instruction_buffer, size_t buffer_size){

	// LibUSB Context
	libusb_context *sdr_context;

	// LibUSB Device Handle
	libusb_device_handle *sdr_handle;

	// Initialize LibUSB pointers to NULL
	sdr_context = NULL;
	sdr_handle = NULL;

	// Initialize LibUSB
    if (libusb_init (&sdr_context) != LIBUSB_SUCCESS)
		mexErrMsgTxt("Error: Could not initialize LibUSB.");

    // Open the device matching our SDR's VID and PID
    sdr_handle = libusb_open_device_with_vid_pid (sdr_context, SDR_USB_VID, SDR_USB_PID);
    if (!sdr_handle)
        mexErrMsgTxt("Error: LibUSB could not find a matching VID/PID or an error was encountered.");

	// Claim the USB Interface
    if (libusb_claim_interface (sdr_handle, 0) != LIBUSB_SUCCESS)
        mexErrMsgTxt("Error: Could not claim interface 0.");

	// Set alternate interface
	if (libusb_set_interface_alt_setting (sdr_handle, 0, 1) != LIBUSB_SUCCESS)
		mexErrMsgTxt("Error: Could not set interface 0 to alternate setting (1).");

    // Keep track of the number of bytes transfered to the device
    int bytes_sent = 0;

    // Attempt a bulk data transfer from the device to the PC
    int libusb_status = libusb_bulk_transfer (sdr_handle, SDR_ENDPOINT_TX, instruction_buffer, buffer_size, &bytes_sent, SDR_TIMEOUT_READ_DATA_MS);

    // Check for any errors from LibUSB
    if (libusb_status != LIBUSB_SUCCESS)
        std::cerr << "Error: libusb_bulk_transfer out returned code " << libusb_status << ": " << libusb_error_name (libusb_status) << std::endl;


    // Release the LibUSB interface claimed for the handle
    if (sdr_handle) libusb_release_interface (sdr_handle, 0);

    // Close the SDR LibUSB device handle
    if (sdr_handle) libusb_close (sdr_handle);

    // Destroy the LibUSB context
    if (sdr_context) libusb_exit (sdr_context);

    return bytes_sent;

}


void mexFunction(
         int          nlhs,
         mxArray      *plhs[],
         int          nrhs,
         const mxArray *prhs[]
         )
{
    // instruction input
    size_t rows = mxGetM(prhs[0]);
    unsigned char* instruction_buffer;
    instruction_buffer = (unsigned char*) mxGetData(prhs[0]);

    // bytes send output
    plhs[0] = mxCreateNumericMatrix(1,1,mxUINT16_CLASS,mxREAL);
    int *bytes_sent = (int *) mxGetData(plhs[0]);
    *bytes_sent = sdradar_send_data( instruction_buffer, rows);

}