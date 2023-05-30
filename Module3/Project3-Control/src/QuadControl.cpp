#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // PARAMETERS (defined in 'QuadControlParams.txt' file)
  //   L: arm length
  //   kappa: drag/thrust ratio
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Moment arm 'l' for a squared quad with length L from centre of quad to motor
  float l = L / sqrtf(2.f);

  // Individual motor-level contributions of each force
  float A = collThrustCmd / 4.f;
  float B = momentCmd.x / (2.f * l * 2.f) ;
  float C = momentCmd.y / (2.f * l * 2.f) ;
  float D = momentCmd.z / (kappa * 4.f);   // kappa is torque (Nm) produced by motor per N of thrust produced

  // Motor forces
  float F1 = A + B + C - D; // front left
  float F2 = A - B + C + D; // front right
  float F3 = A + B - C + D; // rear left
  float F4 = A - B - C - D; // rear right
    
  // Constrained Motor forces
  cmd.desiredThrustsN[0] = CONSTRAIN(F1, minMotorThrust, maxMotorThrust); // front left
  cmd.desiredThrustsN[1] = CONSTRAIN(F2, minMotorThrust, maxMotorThrust); // front right
  cmd.desiredThrustsN[2] = CONSTRAIN(F3, minMotorThrust, maxMotorThrust); // rear left
  cmd.desiredThrustsN[3] = CONSTRAIN(F4, minMotorThrust, maxMotorThrust); // rear right

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // PARAMENTERS (defined in 'QuadControlParams.txt' file)
  //   Ixx, Iyy, Izz: moments of inertia of the vehicle
  //   kpPQR: body rate gain factor (V3F class)
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes
  // REMARKS:
  //  + V3F class used to define 3D-float vectors

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  /// Diagonal of the matrix of Inertia as a V3F vector
  V3F MoI =  V3F(Ixx, Iyy, Izz);

  // Error calculation
  V3F pqr_error = (pqrCmd - pqr);

  // Moment vector command calculation
  momentCmd = MoI * kpPQR * pqr_error;
    
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  // lateral acceleration, the current attitude of the quad, and desired
  // collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // PARAMETERS (defined in 'QuadControlParams.txt' file)
  //   kpBank: roll-pitch gain factor
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)
  // REMARKS:
  //  - Rotation matrix R is included in the attitude variable

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  // Acceleration evaluation
  float c =  - collThrustCmd / mass;

  // Calculation of Rotation Mattrix elements bx and by
  float b_x = CONSTRAIN((accelCmd.x / c), -maxTiltAngle, maxTiltAngle);
  float b_y = CONSTRAIN((accelCmd.y / c), -maxTiltAngle, maxTiltAngle);

  // Calculation of bx_dot and by_dot (P term)
  float b_x_dot = kpBank * (b_x - R(0,2));
  float b_y_dot = kpBank * (b_y - R(1,2));
    
  // Evaluation of p and q commands
  pqrCmd.x = (R(1,0) * b_x_dot - R(0,0) * b_y_dot) / R(2,2);
  pqrCmd.y = (R(1,1) * b_x_dot - R(0,1) * b_y_dot) / R(2,2);
    
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // PARAMETERS (defined in 'QuadControlParams.txt' file)
  //   kpPosZ: proportional z location gain factor
  //   kpVelZ: derivative z location gain factor
  //   KiPosZ: integral z location gain factor
  //   maxAscentRate and maxDescentRate: maximum vertical speeds; they're both >=0
  // OUTPUT:
  //   return a collective thrust command in [N]
  // REMARKS:
  //  - Rotation matrix R is included in the attitude variable
  //  - For an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  // z error calculation
  float posZ_err = posZCmd - posZ;
    
  // Calculation of intetrated altitud error --> for I-term
  integratedAltitudeError += posZ_err * dt;

  // Calculation of velocity command and constrain in its limits --> P-term
  velZCmd += kpPosZ * posZ_err;
  velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);
    
  // Calculation of velocity_error --> for D-term
  float vel_z_err = velZCmd - velZ;
    
  // Calculation of vertical acceleration command --> PID controller with FF
  accelZCmd +=  KiPosZ * integratedAltitudeError + kpVelZ * vel_z_err;

  // Thrust command evaluation
  thrust = mass * (CONST_GRAVITY - accelZCmd) / R(2,2);

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // PARAMETERS (defined in 'QuadControlParams.txt' file)
  //   kpPosXY: proportional xy-location gain factor
  //   kpVelXY: derivative xy-location gain factor
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // REMARKS:
  //  - Maximum horizontal velocity and acceleration are limited to maxSpeedXY and maxAccelXY.

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  // Position error
  V3F pos_error = posCmd - pos;

  // Velocity command calculation (P-term) and constrin in its limits
  velCmd += kpPosXY * pos_error;
  velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
  velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);

  // Velocity error
  V3F vel_error = velCmd - vel;

  // Acceleration command calculation (D-term) and constrain in its limits
  accelCmd += kpVelXY * vel_error;
  accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
  accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // PARAMETERS (defined in 'QuadControlParams.txt' file)
  //   kpYaw: yaw control gain factor kpYaw
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // REMARKS:
  //  + fmodf(foo,b) is used to unwrap a radian angle measure float foo to range [0,b].

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  // Yaw error calculation and transformation to radians
  float yaw_error = yawCmd - yaw;
  yaw_error = fmodf(yaw_error, 2.0 * F_PI);

  // yaw rate command calculation
  yawRateCmd = kpYaw * yaw_error;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
