// Virtual-CAN regression probe. The interface is deliberately fixed to vcan0.
#include "pi_arm_hardware/backend.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include <string>
#include <sstream>
#include <atomic>
using namespace std::chrono_literals;
int main(int argc, char **argv) {
  if(argc!=4) return 2;
  pi_arm_hardware::BackendConfig c;
  c.can_interface="vcan0";
  c.cable_kinematics_file=std::string(argv[1])+"/src/pi_arm_description/config/cable_kinematics.yaml";
  c.calibration_file=argv[2]; c.commissioning=std::string(argv[3])=="commissioning";
  c.allow_real_control=!c.commissioning;
  std::vector<std::string> names{"joint_01","joint_02","joint_04","joint_05","joint_06","joint_09"};
  std::vector<int> directions{1,1,-1,-1,-1,1};
  for(int i=0;i<6;i++) c.actuators.push_back({names[i],static_cast<uint8_t>(i+1),i<3?36.0:6.0,directions[i]});
  c.joint_upper_limits_rad={3.141592654,1.757546556759,1.494001839708,.523598776,1.047197551,3.141592654};
  for(double v:c.joint_upper_limits_rad)c.joint_lower_limits_rad.push_back(-v);
  c.joint_velocity_limits_rad_s.assign(6,.1745329);
  c.joint_feedback_tolerances_rad = {0, 3*3.141592653589793/180,
    3*3.141592653589793/180, 0, 0, 0};
  pi_arm_hardware::RealBackend b(c); b.configure(); b.activate();
  std::this_thread::sleep_for(350ms);
  std::cout << "READY" << std::endl;
  uint64_t lease = 0;
  std::string op;
  while(std::getline(std::cin,op)) {
    if(op=="quit")break;
    pi_arm_hardware::HardwareSample s;
    std::this_thread::sleep_for(100ms);
    const auto begin = std::chrono::steady_clock::now();
    b.read(s);
    const auto read_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - begin).count();
    std::string detail;
    std::istringstream input(op);
    std::string command; input >> command;
    bool ok = true;
    if(command=="motor" || command=="joint") {
      int axis; double delta, speed; input >> axis >> delta >> speed;
      auto r=b.start_axis_test(command,axis,delta,speed);ok=r.success;detail=r.message;
    }
    if(op=="noheartbeat") {b.update_axis_test(s);}
    if(op=="update") {b.heartbeat_axis_test(b.axis_test_id());b.update_axis_test(s);}
    if(op=="write") {
      std::vector<double> q(6,0),v(6,0);b.write(q,v);
      auto r=b.set_motion_armed(true,++lease);q[0]=.01;v[0]=.01;
      ok=r.success;detail=std::string("armed=")+(r.success?"true":"false")+" write="+(b.write(q,v)?"true":"false");
    }
    if(command=="recover" || command=="write_speed") {
      double cap; input >> cap;
      auto q=s.positions_rad;std::vector<double> v(6,0);b.write(q,v);
      auto r=b.set_motion_armed(true,++lease,cap*3.141592653589793/180);
      if(command=="recover")q[1]-=.1*3.141592653589793/180;
      else {q[0]+=.01;v[0]=.1;}
      ok=r.success && b.write(q,v);detail=r.message;
    }
    if(op=="idle") {b.write(s.positions_rad, std::vector<double>(6,0));}
    if(op=="hold" || op=="hold_wrong") {
      auto r=b.set_motion_armed(false, op=="hold" ? lease : lease+100, 0.0, true);ok=r.success;detail=r.message;
    }
    if(op=="stop_hold") {
      auto r=b.set_motion_armed(false,0U,0.0,true);ok=r.success;detail=r.message;
    }
    if(op=="release_old") {
      auto r=b.set_motion_armed(false,lease);ok=r.success;detail=r.message;
    }
    if(op=="axis_hold") {
      auto r=b.stop_axis_test("operator stop",true);ok=r.success;detail=r.message;
    }
    if(op=="hold_race") {
      std::atomic<bool> run{true};
      std::thread reader([&]() {
        while(run.load()) {pi_arm_hardware::HardwareSample feedback;b.read(feedback);std::this_thread::sleep_for(1ms);}
      });
      std::thread writer([&]() {
        auto q=s.positions_rad;std::vector<double> v(6,0);q[0]+=.01;
        while(run.load()) {b.write(q,v);std::this_thread::sleep_for(1ms);}
      });
      auto r=b.set_motion_armed(false,0U,0.0,true);ok=r.success;detail=r.message;
      std::this_thread::sleep_for(100ms);run=false;reader.join();writer.join();
    }
    if(op=="teach_on" || op=="teach_off") {auto r=b.set_teaching(op=="teach_on");ok=r.success;detail=r.message;}
    if(op=="enable") {auto r=b.manage(pi_arm_can::ManagementOperation::MOTOR_RUN,0);ok=r.success;detail=r.message;}
    if(op=="disarm") {auto r=b.set_motion_armed(false,0); b.write(std::vector<double>(6,0),std::vector<double>(6,0));ok=r.success;detail=r.message;}
    if(command=="write_target") {
      int axis; double degrees; input >> axis >> degrees;
      std::vector<double> q(6,0),v(6,0);
      q.at(axis-1)=degrees*3.141592653589793/180;
      ok=b.write(q,v);
    }
    if(op=="arm") {auto r=b.set_motion_armed(true,++lease);ok=r.success;detail=r.message;}
    if(op=="wrong_lease") {auto r=b.set_motion_armed(false,lease+100);ok=r.success;detail=r.message;}
    if(op=="race") {
      std::atomic<bool> run{true};
      std::thread writer([&]() {
        std::vector<double> q(6,0),v(6,0);v[0]=.01;
        while(run.load()) {q[0]=.02; b.write(q,v); std::this_thread::sleep_for(1ms);}
      });
      std::this_thread::sleep_for(30ms);
      auto r=b.set_motion_armed(false,0);ok=r.success;detail=r.message;
      std::this_thread::sleep_for(40ms);run=false;writer.join();
    }
    if(op=="stop") {auto r=b.stop_axis_test("operator stop");ok=r.success;detail=r.message;}
    std::cout << "op=" << command << " ok=" << ok << " read_us=" << read_us << " q2_deg=" << s.positions_rad[1]*180/3.141592653589793 << " q5_deg=" << s.positions_rad[3]*180/3.141592653589793
      << " q3_deg=" << s.positions_rad[2]*180/3.141592653589793
      << " teaching=" << s.teaching_active << " kin=" << s.kinematics_valid << " control=" << s.control_permitted
      << " joint_permit=" << s.joint_test_permitted << " active=" << b.axis_test_active()
      << " stop_pending=" << s.motion_stop_pending << " stop_failed=" << s.motion_stop_failed
      << " error=" << s.kinematics_error_code << " detail=" << detail + " | " + b.axis_test_status() << std::endl;
  }
  b.cleanup();
}
