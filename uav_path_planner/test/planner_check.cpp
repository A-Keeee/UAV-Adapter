#include "uav_path_planner/core.hpp"
#include "uav_path_planner/lidar.hpp"
#include "uav_path_planner/unknown_obstacles.hpp"
#include <gtest/gtest.h>
#include <cstring>
#include <fstream>
#include <filesystem>

using namespace uav_path_planner;
namespace {
World walls() { return World::fromSdf(TEST_WORLD,{-15,-16,.8},{25,18,4}); }
World open() { return World({}, {-4,-4,.8},{4,4,3.2},.35); }
void near(const Vec3 &a,const Vec3 &b,double epsilon=1e-8) { EXPECT_LT((a-b).norm(),epsilon); }
}
TEST(StaticPlanner, AllDemoLegsAreCertifiedAndFeasible) {
  const Points points{{0,0,2.5},{0,8,2.5},{8,8,2.5},{8,0,2.5}};
  const auto world=walls();
  for (size_t i=1;i<points.size();++i) {
    const auto result=plan(world,points[i-1],points[i]); const auto &s=result.spline;
    EXPECT_TRUE(s.certified(world)); near(s.sample(0),points[i-1]); near(s.sample(s.duration()),points[i]);
    for (int order=1;order<=3;++order) { near(s.sample(0,order),Vec3::Zero()); near(s.sample(s.duration(),order),Vec3::Zero()); }
    const auto bounds=s.bounds(); EXPECT_LE(bounds[0],1); EXPECT_LE(bounds[1],.7); EXPECT_LE(bounds[2],1.5);
    for (size_t j=1;j<result.raw.size();++j) EXPECT_TRUE(world.segmentFree(result.raw[j-1],result.raw[j]));
    for (int j=0;j<400;++j) EXPECT_TRUE(world.free(s.sample(s.duration()*j/399)));
  }
}
TEST(StaticPlanner, DerivativesMatchFiniteDifferences) {
  const auto s=plan(walls(),{0,0,2.5},{0,8,2.5}).spline;
  for (int i=1;i<30;++i) for (int order=1;order<=3;++order) {
    const double t=s.duration()*i/30,h=1e-4;
    near((s.sample(t+h,order-1)-s.sample(t-h,order-1))/(2*h),s.sample(t,order),1e-6);
  }
}
TEST(StaticPlanner, InvalidWaypointsAndLimitsAreRejected) {
  const auto w=walls();
  for (const auto &p:Points{{5,0,2.5},{50,0,2.5},{NAN,0,2.5}}) EXPECT_THROW(plan(w,{0,0,2.5},p),std::runtime_error);
  EXPECT_THROW(plan(w,{0,0,2.5},{0,0,2.5}),std::runtime_error);
  EXPECT_THROW(plan(w,{0,0,2.5},{0,8,2.5},{1,0,1}),std::runtime_error);
}
TEST(StaticPlanner, UnreachableGoalIsRejected) {
  World w({{"barrier",{-.5,-4,0},{.5,4,5}}},{-3,-2,1},{3,2,3},.3,.5);
  EXPECT_THROW(w.astar({-2,0,2},{2,0,2}),std::runtime_error);
}
TEST(StaticPlanner, CollisionBetweenSparseSamplesIsRejected) {
  World w({{"thin",{-.01,-1,0},{.01,1,3}}},{-3,-3,.5},{3,3,4},.1);
  Points controls(4,{-2,0,2}); controls.insert(controls.end(),4,{2,0,2});
  std::vector<double> knots(8,0); knots.insert(knots.end(),8,1);
  Spline s(controls,knots); EXPECT_TRUE(w.free(s.sample(0))); EXPECT_TRUE(w.free(s.sample(1))); EXPECT_FALSE(s.certified(w));
}
TEST(StaticPlanner, RotatedSdfIsRejected) {
  const auto file=std::filesystem::temp_directory_path()/"gazebo_planner_rotated_test.sdf";
  { std::ofstream out(file); out<<"<sdf><world><model name='wall'><static>true</static><pose>0 0 1 0 0 .5</pose>"
    "<link><collision><geometry><box><size>1 2 3</size></box></geometry></collision></link></model></world></sdf>"; }
  EXPECT_THROW(World::fromSdf(file,{-3,-3,.5},{3,3,4}),std::runtime_error); std::filesystem::remove(file);
}
TEST(StaticPlanner, SlabIntersectionIncludesContactAndParallelLines) {
  World w({{"box",{0,0,1},{1,1,2}}},{-4,-4,.5},{4,4,4},.1);
  EXPECT_FALSE(w.segmentFree({-2,-.1,1.5},{2,-.1,1.5}));
  EXPECT_TRUE(w.segmentFree({-2,-.1001,1.5},{2,-.1001,1.5}));
  EXPECT_TRUE(w.segmentFree({-2,0,3},{2,0,3}));
}
TEST(StaticPlanner, LookaheadExcludesAlreadyPassedObstacles) {
  Points controls(4,{-2,0,2}); controls.insert(controls.end(),4,{2,0,2});
  std::vector<double> knots(8,0); knots.insert(knots.end(),8,10); Spline s(controls,knots);
  World w({{"behind",{-2.1,-.1,1.9},{-1.9,.1,2.1}}},{-4,-4,.8},{4,4,4},.1);
  EXPECT_FALSE(s.certified(w)); EXPECT_TRUE(s.certified(w,5,8)); EXPECT_FALSE(s.certified(w,0,.1));
}
TEST(Lidar, OrganizedPaddedReorderedAndBigEndianClouds) {
  for (bool big:{false,true}) {
    sensor_msgs::msg::PointCloud2 msg; msg.width=2; msg.height=2; msg.point_step=20; msg.row_step=48;
    msg.is_bigendian=big; msg.data.resize(96);
    for (const auto &pair:std::vector<std::pair<std::string,uint32_t>>{{"z",0},{"x",8},{"y",12}}) {
      sensor_msgs::msg::PointField f; f.name=pair.first; f.offset=pair.second; f.datatype=7; f.count=1; msg.fields.push_back(f);
    }
    const std::array<uint32_t,3> offsets{8,12,0};
    for (int i=0;i<4;++i) for (int axis=0;axis<3;++axis) {
      const float value=i*3+axis+1; std::array<uint8_t,4> bytes{}; std::memcpy(bytes.data(),&value,4);
      if (big) std::reverse(bytes.begin(),bytes.end());
      std::copy(bytes.begin(),bytes.end(),msg.data.begin()+(i/2)*48+(i%2)*20+offsets[axis]);
    }
    const auto p=cloudXYZ(msg); ASSERT_EQ(p.size(),4u);
    for (int i=0;i<4;++i) near(p[i],Vec3(i*3+1,i*3+2,i*3+3));
    msg.data.pop_back(); EXPECT_THROW(cloudXYZ(msg),std::runtime_error);
  }
}
TEST(Lidar, BadFieldsAndOversizedFramesAreRejected) {
  sensor_msgs::msg::PointCloud2 msg; msg.width=1; msg.height=1; msg.point_step=12; msg.row_step=12; msg.data.resize(12);
  EXPECT_THROW(cloudXYZ(msg),std::runtime_error);
  msg.width=200001; msg.row_step=msg.width*12; msg.data.resize(msg.row_step);
  EXPECT_THROW(cloudXYZ(msg),std::runtime_error);
}
TEST(Lidar, OccupancyExpiresAndStaticGeometryIsPreserved) {
  World w({{"wall",{4,-3,0},{5,3,4}}},{-5,-5,.8},{6,5,4},.3);
  LocalObstacles local(.2,.6,.1); local.update({{10,20,2},{10.01,20.01,2.01}},1);
  auto observed=local.world(w,1.3,{10,20,0}); EXPECT_EQ(observed.boxes.size(),2u); EXPECT_FALSE(observed.free({0,0,2}));
  EXPECT_EQ(local.world(w,1.7,{10,20,0}).boxes.size(),1u); EXPECT_EQ(w.boxes.size(),1u);
  EXPECT_THROW(local.update({{0,0,2}},1),std::runtime_error);
}
TEST(Lidar, CapacityOverflowRetainsPreviousValidObservation) {
  LocalObstacles local(.3,.6,.15,1); local.update({{0,0,2}},1);
  EXPECT_THROW(local.update({{2,0,2}},1.1),std::runtime_error); EXPECT_EQ(local.lastStamp(),1); EXPECT_EQ(local.size(),1u);
}
TEST(Lidar, KnownWallsAreNotInflatedTwice) {
  World w({{"wall",{1,-2,0},{2,2,4}}},{-5,-5,.8},{5,5,4});
  EXPECT_FALSE(unmodeled({1,0,2},w)); EXPECT_FALSE(unmodeled({.95,0,2},w)); EXPECT_TRUE(unmodeled({.8,0,2},w));
}
TEST(Lidar, ANewObstacleInvalidatesTheOriginalAndAstarFindsADetour) {
  const auto w=open(); const Vec3 start(-2,0,2),goal(2,0,2); const auto original=plan(w,start,goal);
  LocalObstacles local; local.update({{0,0,2}},1); auto observed=local.world(w,1.1,Vec3::Zero());
  EXPECT_FALSE(original.spline.certified(observed)); EXPECT_FALSE(observed.segmentFree(start,goal));
  const auto detour=plan(observed,start,goal); EXPECT_TRUE(detour.spline.certified(observed));
  EXPECT_GT(detour.spline_length,original.spline_length+.1);
  EXPECT_TRUE(original.spline.certified(local.world(w,1.7,Vec3::Zero())));
}
TEST(Lidar, MovingObstacleLeavesNoPermanentTrail) {
  LocalObstacles local; const auto w=open(); local.update({{0,0,2}},1); local.update({{2,2,2}},1.5);
  EXPECT_FALSE(local.world(w,1.5,Vec3::Zero()).free({0,0,2}));
  const auto moved=local.world(w,1.7,Vec3::Zero()); EXPECT_TRUE(moved.free({0,0,2})); EXPECT_FALSE(moved.free({2,2,2}));
}
TEST(Scenario, EachHiddenObstacleBlocksItsOriginalGloballyPlannedLeg) {
  const auto world=walls(); const auto obstacles=loadUnknownObstacles(TEST_SCENARIO);
  ASSERT_EQ(world.boxes.size(),4u); ASSERT_EQ(obstacles.size(),3u);
  const Points waypoints{{0,0,2.5},{0,8,2.5},{8,8,2.5},{8,0,2.5}};
  for (const auto &o:obstacles) {
    ASSERT_GE(o.leg,1u); ASSERT_LT(o.leg,waypoints.size());
    const auto original=plan(world,waypoints[o.leg-1],waypoints[o.leg]);
    EXPECT_TRUE(original.spline.certified(world));
    World actual=world; actual.boxes.push_back({o.name,o.center-o.size/2,o.center+o.size/2});
    EXPECT_FALSE(original.spline.certified(actual))<<"Hidden obstacle misses leg "<<o.leg;
    EXPECT_EQ(world.boxes.size(),4u);  // Scenario geometry never enters the global World.
    EXPECT_TRUE(world.free(o.center));
  }
}
TEST(StaticPlanner, NarrowCorridorIsNotClosedByExtraVoxelInflation) {
  World w({{"lower",{-3,-2,0},{3,-.8,4}},{"upper",{-3,.8,0},{3,2,4}}},
          {-3,-2,1},{3,2,3},.65,.4);
  const auto route=plan(w,{-2,0,2},{2,0,2});
  EXPECT_TRUE(route.spline.certified(w));
  for (size_t i=1;i<route.raw.size();++i) EXPECT_TRUE(w.segmentFree(route.raw[i-1],route.raw[i]));
}
TEST(StaticPlanner, ExactEdgesCannotCutThroughInflatedDiagonalCorners) {
  World w({{"a",{0,-1,0},{1,0,4}},{"b",{-1,0,0},{0,1,4}}},
          {-2,-2,1},{2,2,3},.15,.4);
  const Vec3 start(-.8,-.8,2),goal(.8,.8,2);
  EXPECT_FALSE(w.segmentFree(start,goal));
  const auto route=w.astar(start,goal);
  for (size_t i=1;i<route.size();++i) EXPECT_TRUE(w.segmentFree(route[i-1],route[i]));
}
TEST(Lidar, FineGridRetryKeepsTheSafetyRadiusInAnOffsetNarrowPassage) {
  World w({{"lower",{-3,-2,0},{3,-.2,4}},{"upper",{-3,.6,0},{3,2,4}}},
          {-3,-2,1},{3,2,3},.3,.4);
  const Vec3 start(-2,.2,2),goal(2,.2,2);
  EXPECT_THROW(plan(w,start,goal),std::runtime_error);
  const auto route=planLocal(w,start,goal,{1,.7,1.5},0);
  EXPECT_DOUBLE_EQ(route.planning_resolution,.2); EXPECT_DOUBLE_EQ(w.radius,.3);
  EXPECT_TRUE(route.spline.certified(w));
}
TEST(Lidar, OptionalPlanningBufferDoesNotTrapAnOtherwiseFreeStart) {
  LocalObstacles local; local.update({{0,0,2}},1);
  const auto w=local.world(open(),1.1,Vec3::Zero());
  const auto route=planLocal(w,{-.9,0,2},{2,0,2},{1,.7,1.5},.5);
  EXPECT_DOUBLE_EQ(route.local_margin,0); EXPECT_TRUE(route.spline.certified(w));
}
namespace {
Points knownWallReturns() {
  Points points;
  for (int i=0;i<60;++i) for (double z:{.5,1.5,2.5,3.5}) {
    points.emplace_back(4.5,-4+i*.13,z);
    points.emplace_back(-5+i*.11,4.5,z);
  }
  return points;
}
}
TEST(Lidar, WallRegistrationCorrectsHeadingAndKeepsUnknownReturns) {
  const auto w=walls(); const Vec3 pivot(0,0,2.5); auto truth=knownWallReturns();
  const size_t known=truth.size();
  for (int i=0;i<100;++i) truth.emplace_back(1.33+.001*i,1.77,2.5);
  const Eigen::Matrix3d drift=Eigen::AngleAxisd(.08,Vec3::UnitZ()).toRotationMatrix();
  Points observed; for (const auto &p:truth) observed.push_back(pivot+drift*(p-pivot)+Vec3(.04,-.03,0));
  const auto alignment=alignKnownWalls(observed,w,pivot); ASSERT_TRUE(alignment.valid);
  EXPECT_NEAR(alignment.yaw,-.08,.005); EXPECT_LT(alignment.rms,.015);
  const auto corrected=applyAlignment(observed,pivot,alignment);
  for (size_t i=known;i<corrected.size();++i) {
    EXPECT_TRUE(unmodeled(corrected[i],w)); near(corrected[i],truth[i],.03);
  }
  EXPECT_EQ(w.boxes.size(),4u);
}
TEST(Lidar, WallRegistrationFollowsHeadingDriftRatherThanFreezingAtTakeoff) {
  const auto w=walls(); const Vec3 pivot(0,0,2.5); const auto truth=knownWallReturns();
  double correction=0;
  for (int frame=0;frame<25;++frame) {
    const double heading_error=.02*frame;
    const Eigen::Matrix3d drift=Eigen::AngleAxisd(heading_error+correction,Vec3::UnitZ()).toRotationMatrix();
    Points observed; for (const auto &p:truth) observed.push_back(pivot+drift*(p-pivot));
    const auto alignment=alignKnownWalls(observed,w,pivot); ASSERT_TRUE(alignment.valid);
    correction+=alignment.yaw; EXPECT_NEAR(correction,-heading_error,.005);
  }
}
TEST(Lidar, UnrelatedPointsCannotBeRegisteredIntoKnownWalls) {
  const auto w=walls(); Points points;
  for (int i=0;i<200;++i) points.emplace_back(1+i*.001,2,2.5);
  const auto alignment=alignKnownWalls(points,w,{0,0,2.5}); EXPECT_FALSE(alignment.valid);
  const auto unchanged=applyAlignment(points,{0,0,2.5},alignment);
  near(unchanged.front(),points.front());
}
TEST(Lidar, ReportedHoverCanReachWaypointAroundTheFirstTestPillar) {
  const auto w=walls(); LocalObstacles local;
  Points points;
  for (double x:{1.03,1.33,1.63}) for (double y:{1.47,1.77,2.07})
    for (int z=1;z<46;++z) points.emplace_back(x,y,z*.1);
  local.update(points,1);
  const auto observed=local.world(w,1.1,Vec3::Zero());
  const auto route=planLocal(observed,{2.949945,1.836575,2.115487},{0,8,2.5});
  EXPECT_TRUE(route.spline.certified(observed));
}
