#pragma once
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <array>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>

namespace uav_path_planner {
// A MAVLink 1 GCS HEARTBEAT is the only outgoing message. No flight commands.
class GcsHeartbeat {
public:
  GcsHeartbeat() {
    socket_=::socket(AF_INET,SOCK_DGRAM,0);
    if (socket_<0) throw std::runtime_error("Cannot create GCS heartbeat UDP socket");
    sockaddr_in local{}; local.sin_family=AF_INET; local.sin_port=htons(14550); local.sin_addr.s_addr=INADDR_ANY;
    if (::bind(socket_,reinterpret_cast<sockaddr*>(&local),sizeof(local))<0) {
      ::close(socket_); socket_=-1; throw std::runtime_error("UDP 14550 unavailable; use external GCS heartbeat");
    }
    ::fcntl(socket_,F_SETFL,O_NONBLOCK);
  }
  ~GcsHeartbeat() { if (socket_>=0) ::close(socket_); }
  GcsHeartbeat(const GcsHeartbeat&)=delete;
  GcsHeartbeat& operator=(const GcsHeartbeat&)=delete;
  void tick(double now) {
    for (int i=0;i<1000;++i) {
      std::array<uint8_t,65536> data{}; sockaddr_in peer{}; socklen_t size=sizeof(peer);
      if (::recvfrom(socket_,data.data(),data.size(),0,reinterpret_cast<sockaddr*>(&peer),&size)<0) break;
      peers_[{peer.sin_addr.s_addr,peer.sin_port}]=peer;
    }
    if (now-last_send_<1) return;
    last_send_=now;
    const auto packet=heartbeat(sequence_++);
    for (const auto &[key,peer]:peers_) {
      (void)key; ::sendto(socket_,packet.data(),packet.size(),0,reinterpret_cast<const sockaddr*>(&peer),sizeof(peer));
    }
  }
  static std::array<uint8_t,17> heartbeat(uint8_t sequence) {
    std::array<uint8_t,17> p{0xfe,9,sequence,255,190,0,0,0,0,0,6,8,0,4,3,0,0};
    uint16_t crc=0xffff;
    auto accumulate=[&](uint8_t byte) {
      uint8_t tmp=byte^(crc&0xff); tmp^=tmp<<4;
      crc=(crc>>8)^(uint16_t(tmp)<<8)^(uint16_t(tmp)<<3)^(uint16_t(tmp)>>4);
    };
    for (size_t i=1;i<15;++i) accumulate(p[i]);
    accumulate(50); p[15]=crc&0xff; p[16]=crc>>8; return p;
  }
private:
  int socket_=-1;
  uint8_t sequence_=0;
  double last_send_=0;
  std::map<std::pair<uint32_t,uint16_t>,sockaddr_in> peers_;
};
}  // namespace uav_path_planner
