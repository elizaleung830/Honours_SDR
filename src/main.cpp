#include "sdr_init.hpp"
#include "sdradar_send_data.hpp"
#include "sdradar_get_data.hpp"
#include "sdradar_kit_info.hpp"
#include "vector"
#include <cstring>
#include "cstdlib"

int main(){
    initialize();
    std::cout << "[main]: Initialised device!" << std::endl;

    // Create cmd for sending data
    size_t cmd_length = 1024; // length of command vec
    size_t cmd_rep = 1024; // how many slot of the command vec needed to be overwrite
    uint16_t cmd_getInfo = 0xFA00; // command header

    // create the buffer where it is filled with the command
    std::vector<uint16_t> instruction_vector(cmd_length, cmd_getInfo);
    unsigned char* instruction_buffer = reinterpret_cast<unsigned char*>(instruction_vector.data());
    int bytes_send = sdradar_send_data(instruction_buffer, cmd_length);
    std::cout << "[main]: sent " << bytes_send << " bytes!" << std::endl;;
    
    // Create vector to hold to output data
    int max_data_rate = 2048000; // amount of uint16 per second

    size_t output_size = 2048+512; // amount of u16int output, from PUPradarGUI
    std::vector<unsigned char> byte_buffer(output_size * sizeof(uint16_t), 0);
    sdradar_get_data(byte_buffer.size() ,byte_buffer.data());

    std::vector<uint16_t> output_vector(output_size);
    std::memcpy(output_vector.data(), byte_buffer.data(), byte_buffer.size());
    std::cout << "[main]: receive " << byte_buffer.size() << " bytes!" << std::endl;;
    
    // std::cout << "[main]: receive bytes in u16int: ";
    // for (auto val : output_vector){
    //     std::cout << val;
    // }
    // std::cout << std::endl;

    SDRKitInfo my_radar;
    // Pass the buffer to your parser
    if (parse_sdr_info(output_vector, my_radar)) {
        std::cout << "[main] Successfully Parsed FA05 Protocol Data\n";
        std::cout << "--------------------------------------\n";
        std::cout << "Model String:  " << my_radar.model << "\n";
        std::cout << "Model Code:    " << my_radar.modelcode << "\n";
        std::cout << "Freq Band:     " << my_radar.FrequencyBand << "\n";
        std::cout << "Num Tx:        " << my_radar.Num_Tx << "\n";
        std::cout << "Num Rx:        " << my_radar.Num_Rx << "\n";
        std::cout << "Antenna Type:  " << my_radar.AntennaType << "\n";
        std::cout << "Version:       " << my_radar.Version << "\n";
    }

    return 0;
}