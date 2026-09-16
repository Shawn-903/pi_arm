#include "pi_arm_hardware/backend.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include <string>
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
  pi_arm_hardware::RealBackend b(c); b.configure(); b.activate();
  std::this_thread::sleep_for(350ms);
  std::cout << "READY" << std::endl;
  std::string op;
  while(std::getline(std::cin,op)) {
    if(op=="quit")break;
    pi_arm_hardware::HardwareSample s;
    std::this_thread::sleep_for(100ms); b.read(s);
    std::string detail;
    if(op=="motor") {auto r=b.start_axis_test("motor",4,.1,.05);detail=r.message;}
    if(op=="joint") {auto r=b.start_axis_test("joint",2,.1,.05);detail=r.message;}
    if(op=="update") {b.heartbeat_axis_test(b.axis_test_id());b.update_axis_test(s);}
    if(op=="write") {
      std::vector<double> q(6,0),v(6,0);b.write(q,v);
      auto r=b.set_motion_armed(true,1);q[0]=.01;v[0]=.01;
      detail=std::string("armed=")+(r.success?"true":"false")+" write="+(b.write(q,v)?"true":"false");
    }
    if(op=="disarm") {auto r=b.set_motion_armed(false,0); b.write(std::vector<double>(6,0),std::vector<double>(6,0));detail=r.message;}
    if(op=="stop")detail=b.stop_axis_test("audit stop").message;
    std::cout << "op=" << op << " q5_deg=" << s.positions_rad[3]*180/3.141592653589793
      << " kin=" << s.kinematics_valid << " control=" << s.control_permitted
      << " joint_permit=" << s.joint_test_permitted << " active=" << b.axis_test_active()
      << " error=" << s.kinematics_error_code << " detail=" << detail << std::endl;
  }
  b.cleanup();
}
