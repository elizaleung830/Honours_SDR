// data = sdr_get_data(data_length);

// Standard Includes
#include <iostream>
#include <stdexcept>


// Include Matlab
#include "mex.h"
#include "matrix.h"

// Include LibUSB
#include "/usr/local/include/libusb-1.0/libusb.h"

// Setup Cypress VID/PID
#define SDR_USB_VID 0x04b4
#define SDR_USB_PID 0x8613

// Setup LibUSB Endpoints
#define SDR_ENDPOINT_RX 0x86                    // Endpoint - Device to PC
#define SDR_ENDPOINT_TX 0x02                    // Endpoint - PC to Device

// Define Timeouts
#define SDR_TIMEOUT_READ 1000 


// Get data from the device
void sdradar_get_data(size_t buffer_size, unsigned char *buffer_char)
{

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


    // Keep track of the number of bytes attained from the device
    int bytes_attained = -1;

    // Attempt a bulk data transfer from the device to the PC
    int libusb_status = libusb_bulk_transfer (sdr_handle, SDR_ENDPOINT_RX, buffer_char, buffer_size, &bytes_attained, SDR_TIMEOUT_READ);

    // Check for any errors from LibUSB
    if (libusb_status != LIBUSB_SUCCESS)
        std::cerr << "Error: libusb_bulk_transfer in returned code " << libusb_status << ": " << libusb_error_name (libusb_status) << std::endl;

    // Release the LibUSB interface claimed for the handle
    if (sdr_handle) libusb_release_interface (sdr_handle, 0);

    // Close the SDR LibUSB device handle
    if (sdr_handle) libusb_close (sdr_handle);

    // Destroy the LibUSB context
    if (sdr_context) libusb_exit (sdr_context);
}



void mexFunction(
		 int          nlhs,
		 mxArray      *plhs[],
		 int          nrhs,
		 const mxArray *prhs[]
		 )
{

   // Create a Matlab input entry point
    long data_length;
    double *p_input_1;
    p_input_1 = mxGetPr(prhs[0]);
    data_length = (long) *p_input_1;
    size_t buffer_size = data_length * 2; // 2 bytes per sample

    // Create an output buffer
    unsigned char *p_output_1;

    // Create an output pointer to the first variable that outputs to Matlab workspace
    plhs[0] = mxCreateNumericMatrix(data_length,1,mxUINT16_CLASS,mxREAL);
    
    // Get the pointer that points to the first output variable and assign it to the output buffer
    p_output_1 = (unsigned char *) mxGetData(plhs[0]);

    // Request data from the device
    sdradar_get_data(buffer_size, p_output_1);

	return;	
}