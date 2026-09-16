#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <cerrno>

static constexpr const char* SOCKET_PATH =
    "../tmp/csp.sock";

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr
            << "Usage: "
            << argv[0]
            << " <target_position>\n";

        return 1;
    }

    char* end = nullptr;
    errno = 0;
    const long long value = std::strtoll(argv[1], &end, 10);
    if (errno == ERANGE || end == argv[1] || *end != '\0' ||
        value < INT32_MIN || value > INT32_MAX) {
        std::cerr << "Target must be an integer in the signed 32-bit range.\n";
        return 1;
    }
    const int32_t target = static_cast<int32_t>(value);

    int sock =
        socket(AF_UNIX, SOCK_DGRAM, 0);

    if (sock < 0)
    {
        perror("socket");
        return 1;
    }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;

    strncpy(
        addr.sun_path,
        SOCKET_PATH,
        sizeof(addr.sun_path) - 1
    );

    ssize_t sent =
        sendto(
            sock,
            &target,
            sizeof(target),
            0,
            reinterpret_cast<sockaddr*>(&addr),
            sizeof(addr)
        );

    if (sent != sizeof(target))
    {
        perror("sendto");
        close(sock);
        return 1;
    }

    std::cout
        << "Sent target position: "
        << target
        << '\n';

    close(sock);

    return 0;
}