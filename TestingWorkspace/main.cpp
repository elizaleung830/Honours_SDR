#include "sdr_init.hpp"
#include "sdradar_send_data.hpp"
#include "vector"
#include "cstdlib"

int main(){
    initialize();
    std::cout << "[main]: Initialised device!" << std::endl;

    size_t cmd_length = 1024; // length of command vec
    size_t cmd_rep = 1024; // how many slot of the command vec needed to be overwrite
    uint16_t cmd_getInfo = 0x0000; // command header

    // create the buffer where it is filled with the command
    std::vector<uint16_t> instruction_vector(cmd_length, cmd_getInfo);
    unsigned char* instruction_buffer = reinterpret_cast<unsigned char*>(instruction_vector.data());
    int bytes_send = sdradar_send_data(instruction_buffer, cmd_length);
    std::cout << "[main]: sent " << bytes_send << " bytes!" << std::endl;;
    // get a response

    return 0;
}