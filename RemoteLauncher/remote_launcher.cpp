// remote_launcher.cpp
//
// Tiny cross-platform (Windows/Linux) TCP server that receives one line of JSON
// from the Unity app (RemoteExecutorClient) and launches a target executable,
// forwarding the JSON to the executable's stdin.
//
// Protocol (must match RemoteExecutorClient.cs):
//   1. Client connects.
//   2. Client sends one UTF-8 JSON line terminated by '\n':
//        {"command":"run_task","poses":[{"frame_id":"ur_base_link","sec":...,
//         "nanosec":...,"position":{"x":..,"y":..,"z":..},
//         "orientation":{"x":..,"y":..,"z":..,"w":..}}, ...]}
//   3. Server runs the target executable, writes the JSON to its stdin,
//      waits for it to exit, then replies with one line:
//        {"ok":true,"exit_code":0}   or   {"ok":false,"error":"..."}
//   4. Server closes the connection and waits for the next client.
//
// Usage:
//   remote_launcher <port> <path-to-target-executable>
//   e.g.  remote_launcher 6001 ./example_task
//         remote_launcher.exe 6001 example_task.exe
//
// Build:
//   Linux:   g++ -std=c++17 -O2 -o remote_launcher remote_launcher.cpp
//   Windows: cl /std:c++17 /O2 remote_launcher.cpp ws2_32.lib
//   (or use the provided CMakeLists.txt)

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "ws2_32.lib")
  typedef SOCKET socket_t;
  #define CLOSESOCK closesocket
  #define POPEN _popen
  #define PCLOSE _pclose
#else
  #include <arpa/inet.h>
  #include <netinet/in.h>
  #include <sys/socket.h>
  #include <sys/wait.h>
  #include <unistd.h>
  typedef int socket_t;
  #define INVALID_SOCKET (-1)
  #define CLOSESOCK close
  #define POPEN popen
  #define PCLOSE pclose
#endif

// Reads bytes from the socket until '\n' (or the peer closes the connection).
static bool readLine(socket_t sock, std::string& out)
{
    out.clear();
    char c;
    while (true)
    {
        int n = recv(sock, &c, 1, 0);
        if (n <= 0)
            return !out.empty();
        if (c == '\n')
            return true;
        if (c != '\r')
            out += c;
    }
}

static void sendLine(socket_t sock, const std::string& line)
{
    std::string data = line + "\n";
    send(sock, data.c_str(), (int)data.size(), 0);
}

// Launches the target executable and pipes the JSON line to its stdin.
// Returns the child's exit code, or -1 if it could not be started.
static int runTarget(const std::string& exePath, const std::string& jsonLine)
{
    // "w" = we get a pipe to the child's stdin.
    FILE* child = POPEN(exePath.c_str(), "w");
    if (!child)
        return -1;

    std::fputs(jsonLine.c_str(), child);
    std::fputc('\n', child);
    std::fflush(child);

    int status = PCLOSE(child);
#ifndef _WIN32
    if (WIFEXITED(status))
        status = WEXITSTATUS(status);
#endif
    return status;
}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <port> <path-to-target-executable>\n";
        return 1;
    }
    const int port = std::atoi(argv[1]);
    const std::string targetExe = argv[2];

#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        std::cerr << "WSAStartup failed\n";
        return 1;
    }
#endif

    socket_t server = socket(AF_INET, SOCK_STREAM, 0);
    if (server == INVALID_SOCKET)
    {
        std::cerr << "socket() failed\n";
        return 1;
    }

    int opt = 1;
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons((unsigned short)port);

    if (bind(server, (sockaddr*)&addr, sizeof(addr)) != 0)
    {
        std::cerr << "bind() failed on port " << port << "\n";
        return 1;
    }
    if (listen(server, 4) != 0)
    {
        std::cerr << "listen() failed\n";
        return 1;
    }

    std::cout << "remote_launcher listening on port " << port
              << ", target executable: " << targetExe << std::endl;

    while (true)
    {
        sockaddr_in clientAddr{};
        socklen_t clientLen = sizeof(clientAddr);
        socket_t client = accept(server, (sockaddr*)&clientAddr, &clientLen);
        if (client == INVALID_SOCKET)
            continue;

        char ip[INET_ADDRSTRLEN] = {0};
        inet_ntop(AF_INET, &clientAddr.sin_addr, ip, sizeof(ip));
        std::cout << "Connection from " << ip << std::endl;

        std::string json;
        if (!readLine(client, json) || json.empty())
        {
            sendLine(client, "{\"ok\":false,\"error\":\"empty request\"}");
            CLOSESOCK(client);
            continue;
        }

        std::cout << "Received: " << json << std::endl;
        std::cout << "Launching: " << targetExe << std::endl;

        int exitCode = runTarget(targetExe, json);
        if (exitCode < 0)
        {
            sendLine(client, "{\"ok\":false,\"error\":\"failed to start executable\"}");
        }
        else
        {
            sendLine(client, "{\"ok\":" + std::string(exitCode == 0 ? "true" : "false")
                           + ",\"exit_code\":" + std::to_string(exitCode) + "}");
        }
        std::cout << "Done, exit code " << exitCode << std::endl;

        CLOSESOCK(client);
    }

    // Unreachable, but for completeness:
    CLOSESOCK(server);
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
