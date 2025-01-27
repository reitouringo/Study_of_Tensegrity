#include <Dynamixel2Arduino.h>

//------------------------------------------------------------------//
const uint8_t DXL_ID_1 = 16;    // Lf11
const uint8_t DXL_ID_2 = 17;    // Lf12
const uint8_t DXL_ID_3 = 19;    // Lf13
const uint8_t DXL_ID_4 = 21;    // Lf14

const float DXL_PROTOCOL_VERSION = 2.0;

// if you change the values, you hardly see angular velocity changes.
int vel_1 = 50;
int vel_2 = 50;
int vel_3 = 50;
int vel_4 = 50;

// if you change the values, you can adjust angular velocity [deg/s].
float deg_per_sec = 36.0*9.0;

// degree division size[deg] (Recommended: this value >= 4[deg])
float deg_div_size = 36.0;

// extend actuator cable length [mm]
float Lf11 = 0.0; // ID 16
float Lf12 = 0.0; // ID 17
float Lf13 = 0.0; // ID 19
float Lf14 = 0.0; // ID 21

// previous Lf pattern
int before_state = 0;
//------------------------------------------------------------------//

Dynamixel2Arduino dxl(Serial3, 84);
using namespace ControlTableItem;

// left motor(ex. ID16,17) -> side=1, right motor(ex. ID19,21) -> side=-1
float calculate_goal_degree(float Lf, int side) {
  float goal_degree = Lf * 360/(24*PI);  // pulley diameter : 24 mm
  if(side == -1){
    goal_degree *= (-1.0);
  }
  return goal_degree;
}

// this function reads key-input.
int keyOperate(int now_state){
  if(Serial.available() > 0)
  {
    char key = Serial.read();
    // initial position
    if(key == '0')
    {
      return 0;
    }
    // optional cable length
    else if(key == '1')
    {
      return 1;
    }
    // stop program
    else if(key == 'q')
    {
      return 9;
    }
  }
  return now_state;
}


void setup() {
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL);


  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_ID_1);
  dxl.ping(DXL_ID_2);
  dxl.ping(DXL_ID_3);
  dxl.ping(DXL_ID_4);

  dxl.torqueOff(DXL_ID_1);
  dxl.setOperatingMode(DXL_ID_1, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_ID_1);

  dxl.torqueOff(DXL_ID_2);
  dxl.setOperatingMode(DXL_ID_2, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_ID_2);

  dxl.torqueOff(DXL_ID_3);
  dxl.setOperatingMode(DXL_ID_3, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_ID_3);

  dxl.torqueOff(DXL_ID_4);
  dxl.setOperatingMode(DXL_ID_4, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_ID_4);

  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_1, vel_1);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_2, vel_2);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_3, vel_3);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_4, vel_4);
}

void loop() {
  int state = keyOperate(before_state);
  
  switch(state)
  {
    case 0:
      Lf11 = 0.0; // ID 16
      Lf12 = 0.0; // ID 17
      Lf13 = 0.0; // ID 19
      Lf14 = 0.0; // ID 21
      break;
    
    case 1:
      Lf11 = -45.0; // ID 16
      Lf12 = 15.0; // ID 17 (20.0)
      Lf13 = 50.0; // ID 19 (55.0)
      Lf14 = -15.0; // ID 21
      break;

    case 9:
      dxl.setGoalPosition(DXL_ID_1, 4.5, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_2, 4.5, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_3, 4.5, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_4, 4.5, UNIT_DEGREE);
      delay(2000);
      state = 0;
      before_state = 0;
      while(true);
      break;

    default:
      break;
  }

  Serial.print("input_state : ");
  Serial.println(state);
  Serial.print("before_state: ");
  Serial.println(before_state);

  if(state < before_state)
  {
    Lf11 *= (-1.0);
    Lf12 *= (-1.0);
    Lf13 *= (-1.0);
    Lf14 *= (-1.0);
  }

  float goal_degree_1 = calculate_goal_degree(Lf11,1);
  float goal_degree_2 = calculate_goal_degree(Lf12,-1);
  float goal_degree_3 = calculate_goal_degree(Lf13,1);
  float goal_degree_4 = calculate_goal_degree(Lf14,-1);

  dxl.setGoalPosition(DXL_ID_1, goal_degree_1, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_2, goal_degree_2, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_3, goal_degree_3, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_4, goal_degree_4, UNIT_DEGREE);

  Serial.print("Present_Position1(degree) : ");
  Serial.println(dxl.getPresentPosition(DXL_ID_1, UNIT_DEGREE));
  Serial.print("Present_Position2(degree) : ");
  Serial.println(dxl.getPresentPosition(DXL_ID_2, UNIT_DEGREE));
  Serial.print("Present_Position3(degree) : ");
  Serial.println(dxl.getPresentPosition(DXL_ID_3, UNIT_DEGREE));
  Serial.print("Present_Position4(degree) : ");
  Serial.println(dxl.getPresentPosition(DXL_ID_4, UNIT_DEGREE));

  delay(1000);

  before_state = state;
}
