#ifndef CNT723_H_
#define CNT723_H_

#include <serial/serial.h>

#include <cstdlib>

class Cnt723 {
public:
    /**
     * Timeout for reading and writing to serial
     */
    const unsigned READ_WRITE_TIMEOUT_MS = 250;

    const std::string GET_COUNT_COMMAND = "O\r";
    const std::string RESET_COUNT_COMMAND = "R\r";
    const std::string ENTER_PROGRAM_MODE_COMMAND = "P\r";
    const std::string EXIT_PROGRAM_MODE_COMMAND = "E\r";

    /**
     * Number of bytes in the response when getting the count
     *
     * According to the user manual, the response is a 12 bytes long hex
     * string plus CRLF, which makes it 14 bytes long.
     */
    const unsigned GET_COUNT_RESPONSE_BYTES = 14;

    /**
     * Response when the count is reset successfully
     */
    const std::string RESET_COUNT_RESPONSE = "O\r\n";

    /* Constructors, Destructor, and Assignment operators {{{ */
    // Default constructor
    Cnt723(const std::string& port, const unsigned baudrate);

    // Copy constructor (deleted)
    Cnt723(const Cnt723& other) = delete;

    // Move constructor (deleted)
    Cnt723(Cnt723&& other) = delete;

    // Destructor
    ~Cnt723();

    // Assignment operator (deleted)
    Cnt723&
    operator=(const Cnt723& other) = delete;

    // Move assignment operator (deleted)
    Cnt723&
    operator=(Cnt723&& other) = delete;
    /* }}} */

    /**
     * Connects to the CNT-723
     *
     * \throw serial::PortNotOpenedException
     */
    void
    connect();

    /**
     * Resets the count in the encoder
     */
    void
    reset_count();

    /**
     * Gets the current count in the encoder
     */
    unsigned long long int
    get_count();

    /**
     * Enters program mode, where one can check the configurations
     */
    void
    enter_program_mode();

    /**
     * Exits program mode
     */
    void
    exit_program_mode();

private:
    serial::Serial serial;
};

#endif /* end of include guard */
