/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include<frc/smartdashboard/SmartDashboard.h>
#include <frc/TimedRobot.h>
#include "rev/CANSparkMax.h" 


class Robot : public frc::TimedRobot {
  /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  rev::CANSparkMax::MotorType::kBrushless
   *  rev::CANSparkMax::MotorType::kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1, 2, 3 and 4. Change
   * these parameters to match your setup
   */ 
  static const int leftLeadDeviceID = 1, leftFollowDeviceID = 2, rightLeadDeviceID = 3, rightFollowDeviceID = 4;
  rev::CANSparkMax* m_leftLeadMotor = new rev::CANSparkMax(leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_rightLeadMotor = new rev::CANSparkMax(rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_leftFollowMotor = new rev::CANSparkMax(leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_rightFollowMotor = new rev::CANSparkMax(rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);

  /**
   * A CANAnalog object is constructed using the GetAnalog() method on an 
   * existing CANSparkMax object. 
   */
  rev::CANAnalog m_analogSensor_left_motor = m_leftLeadMotor->GetAnalog();
  rev::CANAnalog m_analogSensor_right_motor = m_rightLeadMotor->GetAnalog();

  /**
   * In RobotInit() below, we will configure m_leftFollowMotor and m_rightFollowMotor to follow 
   * m_leftLeadMotor and m_rightLeadMotor, respectively. 
   * 
   * Because of this, we only need to pass the lead motors to m_robotDrive. Whatever commands are 
   * sent to them will automatically be copied by the follower motors
   */
  
    void MoveForward(double speed) {
      // set left motor forward (positive voltage)
      m_leftLeadMotor->Set(speed);
      // set right motor forward (positive voltage)
      m_rightLeadMotor->Set(speed);
    }

    void MoveBackward(double speed) {
      // set left motor backward (negative voltage)
      m_leftLeadMotor->Set(-speed);
      // set right motor backward (negative voltage)
      m_rightLeadMotor->Set(-speed)
    }

    void TurnLeft(double speed) {
      // setting left motor back (negative voltage)
      m_leftLeadMotor->Set(-speed);
      // setting right motor forward (positive voltage)
      m_rightLeadMotor->Set(speed);
    }

    void TurnRight(double speed) {
      // setting left motor forward (positive voltage)
      m_leftLeadMotor->Set(speed);
      // setting right motor back (negative voltage)
      m_rightLeadMotor->Set(-speed);
    }

    void StopMotor(double speed) {
      // setting both motors to 0 when called
      m_leftLeadMotor->Set(speed);
      m_leftLeadMotor->Set(speed);
    }

 public:
  void RobotInit() {
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_leftLeadMotor->RestoreFactoryDefaults();
    m_rightLeadMotor->RestoreFactoryDefaults();
    m_leftFollowMotor->RestoreFactoryDefaults();
    m_rightFollowMotor->RestoreFactoryDefaults();
    
    /**
     * In CAN mode, one SPARK MAX can be configured to follow another. This is done by calling
     * the Follow() method on the SPARK MAX you want to configure as a follower, and by passing
     * as a parameter the SPARK MAX you want to configure as a leader.
     * 
     * This is shown in the example below, where one motor on each side of our drive train is
     * configured to follow a lead motor.
     */
    m_leftFollowMotor->Follow(*m_leftLeadMotor);
    m_rightFollowMotor->Follow(*m_rightLeadMotor);
  }

  void TeleopPeriodic() {
    //setting the speed to 1 so that robot moves forward
    MoveForward(1);
    // read the voltage on the lead motors to check if motors are moving forward 
    if((m_analogSensor_left_motor.GetVoltage() > 0) && (m_analogSensor_right_motor.GetVoltage() > 0)) {
      frc::SmartDashboard::PutString("motor Sensor Positive voltage", "Motor moving forward");
    } else { // error message
      frc::SmartDashboard::PutString("ERROR:Motor Sensor voltage", "Not positive");
    }

    //setting the speed to -1 so that robot moves backward
    MoveBackward(-1);
    // read the voltage on the lead motors to check if motors are moving backward
    if((m_analogSensor_left_motor.GetVoltage() < 0) && (m_analogSensor_right_motor.GetVoltage() < 0)) {
      frc::SmartDashboard::PutString("Motor Sensor Negative voltage", "Motor moving backward");
    } else { // error message
      frc::SmartDashboard::PutString("ERROR:Motor Sensor voltage", "Not negative");
    }

    // moving left
    TurnLeft(1);
    //MoveForward(1);
    // read the voltage on the lead motors to check if robot is moving left
    if((m_analogSensor_left_motor.GetVoltage() < 0) && (m_analogSensor_right_motor.GetVoltage() > 0)) {
      frc::SmartDashboard::PutString("Left - Negative Voltage; Right - Positive Voltage", "Motor moving left");
    } else { // error message
      frc::SmartDashboard::PutString("ERROR:Motor Sensor voltage", "Motor not moving left");
    }

    // turning right
    TurnRight(1);
    //MoveForward(1);
    // read the voltage on the lead motors to check if robot is moving right
    if((m_analogSensor_left_motor.GetVoltage() > 0) && (m_analogSensor_right_motor.GetVoltage() <>> 0)) {
      frc::SmartDashboard::PutString("Left - Positive Voltage; Right - Negative Voltage", "Motor moving right");
    } else { // error message
      frc::SmartDashboard::PutString("ERROR:Motor Sensor voltage", "Motor not moving right");
    }
    
    // setting speed to 0 to stop the motor
    StopMotor(0);
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif