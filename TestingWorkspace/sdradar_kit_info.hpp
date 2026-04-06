#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

// Updated Structure to hold the FA05 protocol data
struct SDRKitInfo {
    std::string model;
    unsigned int FrequencyBand = 0;
    unsigned int Num_Tx = 0;
    unsigned int Num_Rx = 0;
    unsigned int AntennaType = 0;
    unsigned int Version = 0;
    long long modelcode = 0; // Using long long to prevent integer overflow
};

// The new parsing function
bool parse_sdr_info(const std::vector<uint16_t>& kit_info, SDRKitInfo& obj) {
    
    // 1. Search for the FA05 header
    // The MATLAB code looks specifically at index 1024, but std::find is safer 
    // as it will find the header even if the buffer shifts slightly.
    auto it = std::find(kit_info.begin(), kit_info.end(), 0xFA05);
    
    if (it == kit_info.end()) {
        std::cerr << "Warning: FA05 header not found." << std::endl;
        obj.model = "";
        return false;
    }

    // Get the index of the header
    size_t header_ind = std::distance(kit_info.begin(), it);

    // Capture the next 3 rows of data (matching MATLAB rows 2, 3, and 4)
    uint16_t val1 = kit_info[header_ind + 1]; // PUPradarBoardInfo(2,:)
    uint16_t val2 = kit_info[header_ind + 2]; // PUPradarBoardInfo(3,:)
    uint16_t val3 = kit_info[header_ind + 3]; // PUPradarBoardInfo(4,:)

    // --- Bitwise Extraction ---
    
    // 1. Frequency Band
    // This is the Lower Byte of val1
    obj.FrequencyBand = val1 & 0xFF;

    // 2. Num_Tx
    obj.Num_Tx = (val2 >> 12) & 0xF;

    // 3. Num_Rx
    obj.Num_Rx = (val2 >> 8) & 0xF;

    // 4. Antenna Type 
    obj.AntennaType = val2 & 0xFF;

    // 5. Version 
    obj.Version = (val3 >> 8) & 0xFF;

    // --- Model Code Calculation ---
    // Multiply by 1000000LL (Long Long) to prevent 32-bit math overflows
    obj.modelcode = (obj.FrequencyBand * 1000000LL) + 
                    (obj.Num_Tx * 100000LL) + 
                    (obj.Num_Rx * 10000LL) + 
                    (obj.AntennaType * 100LL) + 
                    obj.Version;

    // --- Model String Mapping ---
    if (obj.modelcode == 24240100) {
        obj.model = "Model  PUP_DU24P_T2R4";
    } 
    else if (obj.modelcode == 240240100) {
        obj.model = "Model  PUP_EN24P_T2R4";
    } 
    else if (obj.modelcode == 240240200) {
        obj.model = "Model  PUP_EN24C_T2R4 V1";
    } 
    else if (obj.modelcode == 240240202) {
        obj.model = "Model  PUP_EN24C_T2R4 V2";
    } 
    else if (obj.modelcode == 240140201) {
        obj.model = "Model  PUP_EN24C_T1R4";
    } 
    else {
        obj.model = "Needs Refresh";
    }

    return true; // Success
}

