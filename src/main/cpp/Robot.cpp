/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "rev/CANSparkMax.h"
using namespace std;

/**
 * Sample program displaying position and velocity on the SmartDashboard
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

  /** Moves robot in one of four directions by
   * setting speed of left and right motors
   * after ensuring that both speeds are in range.
   * Uses the encoders and GetVelocity() to check
   * in what direction the robot moves.
  */
  void Move(double speedLeft, double speedRight, char direction) {
      if (speedLeft <= 1 && speedLeft >= -1 && speedRight <= 1 && speedRight >= -1) { // speeds in range
        // INPUT TO THE MOTORS: 
        m_leftLeadMotor->Set(speedLeft);
        m_rightLeadMotor->Set(speedRight);

        // OUTPUT FROM THE ENCODERS:
        if (m_encoder_left_motor.GetVelocity() < m_encoder_right_motor.GetVelocity()) { // left
          // STORE IN FILE
          // OUTPUT "L" AS DIRECTION TO FILE
        } else if (m_encoder_left_motor.GetVelocity() > m_encoder_right_motor.GetVelocity()) { // right
          // STORE IN FILE
          // OUTPUT "R" AS DIRECTION TO FILE
        } else { // motor velocities are equal
          if (m_encoder_left_motor.GetVelocity() < 0) { // backward

          } else if (m_encoder_left_motor.GetVelocity() > 0) { //forward

          } else {
            // ROBOT DID NOT MOVE AT ALL
          };
        };

        // moving forward
        // moving backward
      } else { // throw error
        frc::SmartDashboard::PutString("Left motor speed out of range or", "Right motor speed out of range");
      };
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

      /**
        * moves the robot in one of 4 directions at half speed
        * if the robot is moving left, checks that the speed of the left motor < speed of right motor
        * if the robot is moving right, checks that the speed of the right motor < speed of left motor
        * Note: with a dynamic input, 3 ways to move left (and similarly for right):
              * left motor: negative speed, right motor: positive speed
              * left motor: relatively slower, right motor: relatively faster
              * left motor: speed = 0; right motor: positive speed
        */
      Move (0.5, 0.5, 'F'); // forward
      Move (-0.5, -0.5, 'B'); // backward
      Move (-0.5, 0.5, 'L'); // left
      Move (0.5, -0.5, 'R'); // right
  };

  public:
   Robot() { }

  void TeleopPeriodic() override { };
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif