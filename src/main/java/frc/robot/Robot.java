/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_driveStick;
  private Joystick m_armController;
  private static final int leftDriveID = 1; 
  private static final int rightDriveID = 2;
  private static final int leftArmID = 3; 
  private static final int rightArmID = 4;
  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;
  private CANSparkMax m_leftArm;
  private CANSparkMax m_rightArm;

  @Override
  public void robotInit() {
  /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1 and 2. Change
   * these parameters to match your setup
   */
    m_leftMotor = new CANSparkMax(leftDriveID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(rightDriveID, MotorType.kBrushless);
    m_leftArm = new CANSparkMax(leftArmID, MotorType.kBrushless);
    m_rightArm = new CANSparkMax(rightArmID, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();
    m_leftArm.restoreFactoryDefaults();
    m_rightArm.restoreFactoryDefaults();

    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);

    m_driveStick = new Joystick(0);
    m_armController = new Joystick(1);
    m_rightMotor.setInverted(true);
    m_leftArm.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    double speedController = m_driveStick.getRawAxis(3);
    double speed1 = speedController - 1;
    double speed = speed1 / 2;

    m_myRobot.arcadeDrive(m_driveStick.getY() * speed, -m_driveStick.getX() * speed);

    double leftArmSpeed = m_armController.getRawAxis(1);
    double rightArmSpeed = m_armController.getRawAxis(5);
    
    if(Math.abs(leftArmSpeed) > 0.15) { //left arm deadband
      m_leftArm.set(leftArmSpeed);
    } else {
      m_leftArm.set(0); 
    }

    if(Math.abs(rightArmSpeed) > 0.15) { //right arm deadband
      m_rightArm.set(rightArmSpeed);
    } else {
      m_rightArm.set(0);
    }

  }

  @Override
  public void autonomousPeriodic() {

  }
}
