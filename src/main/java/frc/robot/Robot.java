package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
  //variables galore
  private DifferentialDrive m_myRobot;
  private Joystick driveStick;
  private Joystick armGamepad;
  private static final int leftDriveID = 1; 
  private static final int rightDriveID = 2;
  private static final int leftArmID = 3; 
  private static final int rightArmID = 4;
  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;
  private CANSparkMax m_leftArm;
  private CANSparkMax m_rightArm;
  private PWMSparkMax m_intakeArm;
  private PWMSparkMax m_intake;

  double leftArmOut = 0;
  double rightArmOut = 0;

  double autoStart = 0;

  @Override
  public void robotInit() {

    m_leftMotor = new CANSparkMax(leftDriveID, MotorType.kBrushless); //woo canbus stuff
    m_rightMotor = new CANSparkMax(rightDriveID, MotorType.kBrushless);
    m_leftArm = new CANSparkMax(leftArmID, MotorType.kBrushless);
    m_rightArm = new CANSparkMax(rightArmID, MotorType.kBrushless);
    m_intakeArm = new PWMSparkMax(7); //pwm cause mentor asked for it
    m_intake = new PWMSparkMax(8); //pwm cause mentor asked for it

    m_rightArm.setOpenLoopRampRate(0.1);
    m_leftArm.setOpenLoopRampRate(0.1);
    m_rightArm.setIdleMode(IdleMode.kBrake);
    m_leftArm.setIdleMode(IdleMode.kBrake);

    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);

    driveStick = new Joystick(0); //big joystick thing
    armGamepad = new Joystick(1); //small standard console controller
    m_rightMotor.setInverted(true); //backwards, to add more confusion
    m_leftArm.setInverted(true); // backwards, to add more confusion

    CameraServer.startAutomaticCapture(); //woo I have eyes
  }

  @Override
  public void teleopPeriodic() {
    double speedController = driveStick.getRawAxis(3); //little dial thing on the front of the controller
    double speed1 = speedController - 1; //maths
    double speed = speed1 / 2; //more maths

    m_myRobot.arcadeDrive(driveStick.getY() * speed, -driveStick.getX() * speed); //vroom vroom

    double leftArmSpeed = armGamepad.getRawAxis(1); //y axis for left stick
    double rightArmSpeed = armGamepad.getRawAxis(5); //y axis for right stick

    boolean armRB = armGamepad.getRawButton(6); //button 6 for right bumper
    boolean armLB = armGamepad.getRawButton(5); //button 5 for left bumper

    if(armRB == true) {
      leftArmOut = -1;
      rightArmOut = 1;
    } else if(armLB == true) {
      leftArmOut = 1;
      rightArmOut = -1;
    } else if(armRB == false && armLB == false) {
      if(Math.abs(leftArmSpeed) > 0.15) { //left arm deadband
        leftArmOut = leftArmSpeed;
      } else {
        leftArmOut = 0;//who is insane enough to put a comment without a space separating it from the code?
      }
  
      if(Math.abs(rightArmSpeed) > 0.15) { //right arm deadband
        rightArmOut = rightArmSpeed;
      } else {
        rightArmOut = 0;
      }
    }

    m_leftArm.set(leftArmOut);
    m_rightArm.set(rightArmOut);

    boolean armUp = armGamepad.getRawButton(4); //y button
    boolean armDown = armGamepad.getRawButton(1); //a button
    boolean intake = armGamepad.getRawButton(3); //x button

    if(armUp) { //intake arm control
      m_intakeArm.set(0.5); //half speed cause I don't know how fast it will be yet
    } else if(armDown) {
      m_intakeArm.set(-0.5);
    } else {
      m_intakeArm.set(0);
    }

    if(intake) { //intake control
      m_intake.set(0.5); //also half speed, subject to change, same reason as above
    } else {
      m_intake.set(0);
    }

  }

  @Override
  public void autonomousInit() {
    autoStart = Timer.getFPGATimestamp();
  }


  @Override
  public void autonomousPeriodic() {
    //get a time for auton start to do events based on time later
    double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart; //get time since start of auto
    if(autoTimeElapsed < 3) { //move for 3 seconds
      m_leftMotor.set(0.5);
      m_rightMotor.set(0.5);
    } else {
      m_leftMotor.set(0);
      m_rightMotor.set(0);
    }
  }
}
