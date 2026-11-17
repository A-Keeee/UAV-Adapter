#pragma once
// Only middleware compatibility lives here. No ROS 1 libraries, master, message
// transport, geometry, map storage or optimisation are used by this adapter.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <chrono>
#include <thread>
#include <functional>

namespace ego_ros2 {
inline rclcpp::Node::SharedPtr node;
struct Duration {
  double value;
  explicit Duration(double seconds=0):value(seconds){}
  double toSec()const{return value;}
  Duration operator-(Duration b)const{return Duration(value-b.value);}
  Duration operator+(Duration b)const{return Duration(value+b.value);}
  void sleep()const{std::this_thread::sleep_for(std::chrono::duration<double>(value));}
};
class Time {
public:
  Time(double seconds=0):value_(seconds){}
  Time(const builtin_interfaces::msg::Time &m):value_(m.sec+m.nanosec*1e-9){}
  static Time now(){return Time(node->now().seconds());}
  double toSec()const{return value_;}
  Time &fromSec(double seconds){value_=seconds;return *this;}
  operator builtin_interfaces::msg::Time()const{return rclcpp::Time(int64_t(value_*1e9),RCL_ROS_TIME);}
  Duration operator-(Time b)const{return Duration(value_-b.value_);}
  Time operator+(Duration d)const{return Time(value_+d.value);}
  Time operator-(Duration d)const{return Time(value_-d.value);}
  bool operator<(Time b)const{return value_<b.value_;}
  bool operator>(Time b)const{return value_>b.value_;}
  bool operator<=(Time b)const{return value_<=b.value_;}
  bool operator==(Time b)const{return value_==b.value_;}
private:double value_;
};
struct TimerEvent {};
class Timer {
public:
  rclcpp::TimerBase::SharedPtr handle;
  void stop(){if(handle)handle->cancel();}
  void start(){if(handle)handle->reset();}
};
struct TransportHints {TransportHints &tcpNoDelay(){return *this;}};
class Publisher {
public:
  rclcpp::PublisherBase::SharedPtr handle;
  template<class M>void publish(const M &m)const {
    auto p=std::dynamic_pointer_cast<rclcpp::Publisher<M>>(handle);
    if(!p)throw std::runtime_error("EGO publisher message type mismatch");p->publish(m);
  }
  size_t getNumSubscribers()const{return handle?handle->get_subscription_count():0;}
};
using Subscriber=rclcpp::SubscriptionBase::SharedPtr;
class NodeHandle {
public:
  explicit NodeHandle(const std::string & = ""){}
  template<class T>void param(const std::string &name,T &value,const T &fallback) {
    if(!node->has_parameter(name))node->declare_parameter<T>(name,fallback);
    node->get_parameter(name,value);
  }
  template<class M>Publisher advertise(const std::string &topic,int depth) {
    Publisher p;p.handle=node->create_publisher<M>(topic,depth);return p;
  }
  template<class M,class C>Subscriber subscribe(const std::string &topic,int depth,void(C::*cb)(const std::shared_ptr<const M>&),C *self,TransportHints={}) {
    return node->create_subscription<M>(topic,rclcpp::SensorDataQoS().keep_last(depth),[self,cb](std::shared_ptr<const M> m){(self->*cb)(m);});
  }
  template<class M,class C>Subscriber subscribe(const std::string &topic,int depth,void(C::*cb)(std::shared_ptr<const M>),C *self,TransportHints={}) {
    return node->create_subscription<M>(topic,rclcpp::SensorDataQoS().keep_last(depth),[self,cb](std::shared_ptr<const M> m){(self->*cb)(m);});
  }
  template<class M>Subscriber subscribe(const std::string &topic,int depth,void(*cb)(const std::shared_ptr<const M>&)) {
    return node->create_subscription<M>(topic,rclcpp::SensorDataQoS().keep_last(depth),cb);
  }
  template<class M,class C>Subscriber subscribe(const std::string &topic,int depth,void(C::*cb)(const std::shared_ptr<M>&),C *self,TransportHints={}) {
    return node->create_subscription<M>(topic,rclcpp::SensorDataQoS().keep_last(depth),[self,cb](std::shared_ptr<M> m){(self->*cb)(m);});
  }
  template<class M,class C>Subscriber subscribe(const std::string &topic,int depth,void(C::*cb)(const M&),C *self,TransportHints={}) {
    return node->create_subscription<M>(topic,rclcpp::SensorDataQoS().keep_last(depth),[self,cb](std::shared_ptr<const M> m){(self->*cb)(*m);});
  }
  template<class C>Timer createTimer(Duration duration,void(C::*cb)(const TimerEvent &),C *self) {
    Timer t;t.handle=rclcpp::create_timer(node,node->get_clock(),std::chrono::duration<double>(duration.value),[self,cb](){(self->*cb)(TimerEvent{});});return t;
  }
};
inline void init(int argc,char **argv,const std::string &name){rclcpp::init(argc,argv);node=std::make_shared<rclcpp::Node>(name);}
inline bool ok(){return rclcpp::ok();}
inline void spin(){rclcpp::spin(node);}
inline void spinOnce(){rclcpp::spin_some(node);}
inline void shutdown(){rclcpp::shutdown();}
}
#define ROS_INFO(...) RCLCPP_INFO(ego_ros2::node->get_logger(),__VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(ego_ros2::node->get_logger(),__VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(ego_ros2::node->get_logger(),__VA_ARGS__)
#define ROS_ERROR_ONCE(...) RCLCPP_ERROR_ONCE(ego_ros2::node->get_logger(),__VA_ARGS__)
#define ROS_WARN_COND(condition,...) do {if(condition) ROS_WARN(__VA_ARGS__);} while(false)
#define ROS_ERROR_COND(condition,...) do {if(condition) ROS_ERROR(__VA_ARGS__);} while(false)
