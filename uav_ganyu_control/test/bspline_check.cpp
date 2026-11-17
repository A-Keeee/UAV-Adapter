#include "uav_ganyu_control/bspline.hpp"
#include <iostream>

using namespace uav_ganyu_control;
int main() {
  const Vec3 target(2,-3,1);
  std::vector<Vec3> points(4, Vec3::Zero());
  points.insert(points.end(), 4, target);
  std::vector<double> knots(8, 0); knots.insert(knots.end(), 8, 2);
  BSplineReference spline;
  spline.set(7, knots, points);
  for (double s : {0.0, 0.03, 0.2, 0.49, 0.8, 0.99}) {
    const auto r = spline.sample(2*s);
    const double p = 35*std::pow(s,4)-84*std::pow(s,5)+70*std::pow(s,6)-20*std::pow(s,7);
    const double v = (140*std::pow(s,3)-420*std::pow(s,4)+420*std::pow(s,5)-140*std::pow(s,6))/2;
    const double a = (420*s*s-1680*std::pow(s,3)+2100*std::pow(s,4)-840*std::pow(s,5))/4;
    const double j = (840*s-5040*s*s+8400*std::pow(s,3)-4200*std::pow(s,4))/8;
    if ((r.p-target*p).norm() > 1e-9 || (r.v-target*v).norm() > 1e-9 ||
        (r.a-target*a).norm() > 1e-9 || (r.j-target*j).norm() > 1e-9) return 1;
  }
  auto end = spline.sample(3);
  if ((end.p-target).norm() > 1e-12 || end.v.norm() || end.a.norm() || end.j.norm()) return 2;
  for (const auto &bad : {std::vector<double>{}, std::vector<double>(16, 0)}) {
    try { spline.set(7, bad, points); return 3; } catch (const std::runtime_error &) {}
  }
  for(const Vec3 &v:{Vec3::Zero().eval(),Vec3(.3,.2,-.1),Vec3(.7,0,0)}) {
    Reference moving;moving.p=Vec3(1,2,3);moving.v=v;moving.a=Vec3(.1,0,0);moving.j=Vec3(.05,0,0);
    auto brake=brakingReference(moving,10);
    auto first=brake.first.sample(10),last=brake.first.sample(10+brake.second);
    if((first.p-moving.p).norm()>1e-8 || (first.v-moving.v).norm()>1e-8 ||
       (first.a-moving.a).norm()>1e-8 || (first.j-moving.j).norm()>1e-8 || last.v.norm()>1e-8 || last.a.norm()>1e-8 || last.j.norm()>1e-8)return 4;
    for(double t=0;t<=brake.second;t+=.01){auto r=brake.first.sample(10+t);if(r.v.norm()>1.00001 || r.a.norm()>.70001 || r.j.norm()>1.50001){std::cerr<<"Brake exceeded V/A/J limits: "<<r.v.norm()<<' '<<r.a.norm()<<' '<<r.j.norm()<<'\n';return 5;}}
  }
  for (int axis=0;axis<3;++axis) {
    Reference invalid;
    if(axis==0)invalid.v=Vec3(1.01,0,0);
    if(axis==1)invalid.a=Vec3(.71,0,0);
    if(axis==2)invalid.j=Vec3(1.51,0,0);
    bool rejected=false;
    try{(void)brakingReference(invalid,10);}catch(const std::runtime_error &){rejected=true;}
    if(!rejected){std::cerr<<"Brake accepted a reference beyond the 1/.7/1.5 limits, axis "<<axis<<'\n';return 6;}
  }
  std::cout << "B-spline analytic p/v/a/jerk, endpoint hold and malformed input checks passed\n";
  return 0;
}
