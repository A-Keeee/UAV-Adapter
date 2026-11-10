#pragma once
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <array>
#include <cerrno>
#include <stdexcept>
#include <string>

extern char **environ;
namespace uav_path_planner {
// Execute gz directly, without shell parsing or allocations in a forked child.
inline void gazeboService(const std::string &name,const std::string &type,const std::string &request) {
  int output[2];
  if (::pipe(output)<0) throw std::runtime_error("Cannot create Gazebo service pipe");
  const std::string topic="/world/walls_demo/"+name;
  std::array<const char*,13> args{"gz","service","-s",topic.c_str(),"--reqtype",type.c_str(),
      "--reptype","gz.msgs.Boolean","--timeout","4000","--req",request.c_str(),nullptr};
  posix_spawn_file_actions_t actions;
  ::posix_spawn_file_actions_init(&actions);
  ::posix_spawn_file_actions_addclose(&actions,output[0]);
  ::posix_spawn_file_actions_adddup2(&actions,output[1],STDOUT_FILENO);
  ::posix_spawn_file_actions_adddup2(&actions,output[1],STDERR_FILENO);
  ::posix_spawn_file_actions_addclose(&actions,output[1]);
  pid_t pid=0;
  const int error=::posix_spawnp(&pid,"gz",&actions,nullptr,const_cast<char* const*>(args.data()),environ);
  ::posix_spawn_file_actions_destroy(&actions);
  ::close(output[1]);
  if (error) { ::close(output[0]); throw std::runtime_error("Cannot execute Gazebo service"); }
  std::string response; std::array<char,4096> buffer{};
  for (;;) {
    const ssize_t size=::read(output[0],buffer.data(),buffer.size());
    if (size>0) response.append(buffer.data(),static_cast<size_t>(size));
    else if (size<0 && errno==EINTR) continue;
    else break;
  }
  ::close(output[0]); int status=0;
  while (::waitpid(pid,&status,0)<0) if (errno!=EINTR) throw std::runtime_error("Cannot wait for Gazebo service");
  if (!WIFEXITED(status) || WEXITSTATUS(status)!=0 || response.find("data: true")==std::string::npos)
    throw std::runtime_error("Gazebo "+name+" failed: "+response);
}
}  // namespace uav_path_planner
