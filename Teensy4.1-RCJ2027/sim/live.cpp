// Tiny web server for the live mode: the browser asks, world.cpp answers.
// Only on this computer (127.0.0.1). Kept apart from Arduino.h (its HIGH/INPUT macros clash with Windows).
#include <cstdio>
#include <cstdlib>
#include <string>

#ifdef _WIN32
#include <winsock2.h>
typedef int socklen_t;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
typedef int SOCKET;
#define INVALID_SOCKET (-1)
#define closesocket close
#endif

// handle(path, type): returns the reply body and sets the content type
typedef std::string (*LiveHandler)(const std::string& path, std::string& type);

int live_serve(int port, LiveHandler handle) {
#ifdef _WIN32
  WSADATA wsa;
  WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
  SOCKET srv = socket(AF_INET, SOCK_STREAM, 0);
  int yes = 1;
  setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, (const char*)&yes, sizeof(yes));
  sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (bind(srv, (sockaddr*)&addr, sizeof(addr)) != 0 || listen(srv, 8) != 0) {
    printf("Port %d is busy. Is another live window still running?\n", port);
    return 1;
  }
  printf("Live mode: open http://localhost:%d   (Ctrl+C to stop)\n", port);
  fflush(stdout);
#ifdef _WIN32
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "start \"\" http://localhost:%d", port);
  system(cmd);
#endif

  for (;;) {
    SOCKET c = accept(srv, nullptr, nullptr);
    if (c == INVALID_SOCKET) continue;
    std::string req;
    char buf[2048];
    while (req.find("\r\n\r\n") == std::string::npos) {   // read the request head
      int n = recv(c, buf, sizeof(buf), 0);
      if (n <= 0) break;
      req.append(buf, n);
    }
    // "GET /step?ms=20 HTTP/1.1" -> "/step?ms=20"
    size_t a = req.find(' '), b = req.find(' ', a + 1);
    std::string path = (a != std::string::npos && b != std::string::npos) ? req.substr(a + 1, b - a - 1) : "/";
    std::string type = "application/json";
    std::string body = handle(path, type);
    std::string head = "HTTP/1.1 200 OK\r\nContent-Type: " + type +
                       "\r\nCache-Control: no-store\r\nConnection: close\r\nContent-Length: " +
                       std::to_string(body.size()) + "\r\n\r\n";
    send(c, head.data(), (int)head.size(), 0);
    send(c, body.data(), (int)body.size(), 0);
    closesocket(c);
  }
}
