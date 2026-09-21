#include "csp_command.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

static bool parse_command(int argc, char *argv[], CspCommand &command) {
    std::string text;
    if (argc == 2) {
        text = argv[1];
        const auto first = text.find_first_not_of(" \t\r\n");
        const auto last = text.find_last_not_of(" \t\r\n");
        if (first == std::string::npos || text[first] != '[' || text[last] != ']')
            return false;
        text = text.substr(first + 1, last - first - 1);
        // Exactly six comma-separated entries; empty entries are rejected below.
        std::istringstream array(text);
        for (int i = 0; i < CSP_MOTOR_COUNT; ++i) {
            std::string entry;
            if (!std::getline(array, entry, ','))
                return false;
            std::istringstream value_stream(entry);
            long long value;
            if (!(value_stream >> value) || value < INT32_MIN || value > INT32_MAX)
                return false;
            value_stream >> std::ws;
            if (!value_stream.eof())
                return false;
            command.positions[i] = static_cast<int32_t>(value);
            if (i < CSP_MOTOR_COUNT - 1 && array.eof())
                return false;
        }
        return array.eof(); // Reject extra entries and a trailing comma.
    }
    if (argc != CSP_MOTOR_COUNT + 1)
        return false;
    for (int i = 0; i < CSP_MOTOR_COUNT; ++i) {
        std::istringstream entry(argv[i + 1]);
        long long value;
        if (!(entry >> value) || value < INT32_MIN || value > INT32_MAX)
            return false;
        entry >> std::ws;
        if (!entry.eof())
            return false;
        command.positions[i] = static_cast<int32_t>(value);
    }
    return true;
}

int main(int argc, char *argv[]) {
    CspCommand command{};
    if (!parse_command(argc, argv, command)) {
        std::cerr << "Usage: " << argv[0] << " p1 p2 p3 p4 p5 p6\n"
                  << "   or: " << argv[0] << " \"[p1, p2, p3, p4, p5, p6]\"\n"
                  << "Supply exactly six signed 32-bit absolute encoder counts (slaves 1-6).\n";
        return 1;
    }

    const int sock = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (sock < 0) {
        perror("socket");
        return 1;
    }
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, CSP_SOCKET_PATH, sizeof(addr.sun_path) - 1);

    ssize_t sent;
    do {
        sent = sendto(sock, &command, sizeof(command), MSG_DONTWAIT,
                      reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    } while (sent < 0 && errno == EINTR);
    const int send_error = errno;
    close(sock);
    if (sent != static_cast<ssize_t>(sizeof(command))) {
        if (sent < 0 && (send_error == EAGAIN || send_error == EWOULDBLOCK))
            std::cerr << "Command queue full; command was NOT sent. Retry later.\n";
        else if (sent < 0)
            std::cerr << "Cannot send to " << CSP_SOCKET_PATH << ": "
                      << std::strerror(send_error) << '\n';
        else
            std::cerr << "Incomplete command send.\n";
        return 1;
    }
    std::cout << "Queued positions (slaves 1-6): [";
    for (int i = 0; i < CSP_MOTOR_COUNT; ++i)
        std::cout << (i ? ", " : "") << command.positions[i];
    std::cout << "]\n";
    return 0;
}
