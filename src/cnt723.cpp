#include <cnt723/cnt723.h>

Cnt723::Cnt723(const std::string& port,
               const unsigned baudrate)
{
    serial.setPort(port);
    serial.setBaudrate(baudrate);
    serial.setTimeout(serial::Timeout::max(),
                      READ_WRITE_TIMEOUT_MS, 0,
                      READ_WRITE_TIMEOUT_MS, 0);
}

Cnt723::~Cnt723()
{
    serial.close();
}

void
Cnt723::connect() {
    if (serial.isOpen()) {
        return;
    }

    serial.open();
}

void
Cnt723::reset_count() {
    serial.write(RESET_COUNT_COMMAND);

    std::string response;
    while (response != RESET_COUNT_RESPONSE) {
        response = serial.readline();
    }
}

unsigned long long int
Cnt723::get_count() {
    serial.write(GET_COUNT_COMMAND);
    std::string response;
    size_t read_bytes = 0;
    while (read_bytes != GET_COUNT_RESPONSE_BYTES) {
        read_bytes = serial.readline(response);
    }

    return std::strtoull(response.c_str(), nullptr, 16);
}

void
Cnt723::enter_program_mode() {
    serial.write(ENTER_PROGRAM_MODE_COMMAND);
}

void
Cnt723::exit_program_mode() {
    serial.write(EXIT_PROGRAM_MODE_COMMAND);
}
