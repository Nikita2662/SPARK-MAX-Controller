/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "rev/CANSparkMax.h"

/**
 * Sample program displaying position and velocity on the SmartDashboard
 * 
 * Position is displayed in revolutions and velocity is displayed in RPM
 */
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
   * In order to read encoder values encoder objects are created using the 
   * GetEncoder() method from an existing CANSparkMax object
   */

  // CHECK FORMAT OF THESE FOLLOWING LINES: (also should these be pointers?)
  // no encoder object created for the follow motors because they will reflect the encoders on the lead motors
  rev::CANEncoder m_encoder_left_motor = m_leftLeadMotor->GetEncoder(); 
  rev::CANEncoder m_encoder_right_motor = m_rightLeadMotor->GetEncoder(); 

  // moves robot in one of four directions by
  // setting speed of left and right motors
  void Move(double speedLeft, double speedRight) {
      m_leftLeadMotor->Set(speedLeft);
      m_rightLeadMotor->Set(speedRight);
    };

  /**
   * In RobotInit() below, we will configure m_leftFollowMotor and m_rightFollowMotor to follow 
   * m_leftLeadMotor and m_rightLeadMotor, respectively. 
   * 
   * Because of this, we only need to pass the lead motors to m_robotDrive. Whatever commands are 
   * sent to them will automatically be copied by the follower motors
   */

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
  };


  // IS THIS NECESSARY?
  public:
   Robot() { }

  void TeleopPeriodic() override {
    /**
     * Encoder position is read from a CANEncoder object by calling the
     * GetPosition() method.
     * 
     * GetPosition() returns the position of the encoder in units of revolutions
     */

    // IS THIS NECESSARY?
    frc::SmartDashboard::PutNumber("Left Encoder Position", m_encoder_left_motor.GetPosition());
    frc::SmartDashboard::PutNumber("Right Encoder Position", m_encoder_right_motor.GetPosition());

    /**
     * Encoder velocity is read from a CANEncoder object by calling the
     * GetVelocity() method.
     * 
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */

    // IS THIS NECESSARY?
    frc::SmartDashboard::PutNumber("Left Encoder Velocity", m_encoder_left_motor.GetVelocity());
    frc::SmartDashboard::PutNumber("Right Encoder Velocity", m_encoder_right_motor.GetVelocity());

    /**
     * moves the robot in one of 4 directions
     * also checks if the range for the speed of the motors is between -0.5 and 0.5 (half speed)
     * if the robot is moving left, checks that the speed of the left motor < speed of right motor
     * if the robot is moving right, checks that the speed of the right motor < speed of left motor
     * Note: with a dynamic input, 3 ways to move left (and similarly for right):
          * left motor: negative speed, right motor: positive speed
          * left motor: relatively slower, right motor: relatively faster
          * left motor: speed = 0; right motor: positive speed
     */
    Move (0.5, 0.5); // forward
    Move (-0.5, -0.5); // backward
    Move (0, 0.5); // left
    Move (0.5, 0); // right

    /**
     * uses the encoders and GetVelocity()
     * to check in what direction the robot moves 
     */

    // if speed of left motor < speed of right, robot moving left
    if (m_encoder_left_motor.GetVelocity() < m_encoder_right_motor.GetVelocity()) {
      // STORE IN FILE
    }

    // if speed of left motor > speed of right, robot moving right
    else if (m_encoder_left_motor.GetVelocity() > m_encoder_right_motor.GetVelocity()) {
      // STORE IN FILE
    };

    // moving forward
    // moving backward
  };
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif