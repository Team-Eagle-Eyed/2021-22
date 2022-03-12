package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  //variables galore
  private DifferentialDrive m_myRobot;
  
  private Joystick driveStick;
  private XboxController armGamepad;

  private static final int leftDriveID = 1; 
  private static final int rightDriveID = 2;
  private static final int leftArmID = 3; 
  private static final int rightArmID = 4;

  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;
  private CANSparkMax m_leftArm;
  private CANSparkMax m_rightArm;

  private PWMSparkMax m_intake;
  private VictorSP m_intakeArm;

  double leftArmOut = 0;
  double rightArmOut = 0;

  Timer teleopTimer;
  Timer autoTimer;
  boolean runAuto = true;
  Double gamepadDeadband = .05; // I like to declare inline, especially for simple things
  int armFeedForward = 0;

  @Override
  public void robotInit() {
    m_leftMotor = new CANSparkMax(leftDriveID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(rightDriveID, MotorType.kBrushless);
    
    m_leftArm = new CANSparkMax(leftArmID, MotorType.kBrushless);
    m_rightArm = new CANSparkMax(rightArmID, MotorType.kBrushless);
    m_intakeArm = new VictorSP(8); //pwm cause mentor asked for it
    m_intake = new PWMSparkMax(9); //pwm cause mentor asked for it

    m_rightArm.setOpenLoopRampRate(0.1); //ramp rate so the motor cant stall when jumping to full power
    m_leftArm.setOpenLoopRampRate(0.1);
    m_rightMotor.setOpenLoopRampRate(0.1);
    m_leftMotor.setOpenLoopRampRate(0.1);
    m_rightArm.setIdleMode(IdleMode.kBrake); //brake so the arms don't coast past the limit switches
    m_leftArm.setIdleMode(IdleMode.kBrake);

    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_myRobot.setDeadband(0.05);

    driveStick = new Joystick(0); //big joystick thing
    armGamepad = new XboxController(1); //small standard console controller
    
    m_leftArm.setInverted(true); // backwards, to add more confusion
    m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(true);

    CameraServer.startAutomaticCapture(); //woo I have eyes

    SmartDashboard.putBoolean("driveEnabled", true);
    SmartDashboard.putBoolean("armsEnabled", true);
    SmartDashboard.putBoolean("intakeEnabled", true);
    SmartDashboard.putNumber("m_leftArm", 0);
    SmartDashboard.putNumber("m_rightArm", 0);
    SmartDashboard.putNumber("m_leftMotor", 0);
    SmartDashboard.putNumber("m_rightMotor", 0);
    SmartDashboard.putNumber("m_intakeArm", 0);
    SmartDashboard.putNumber("m_intake", 0);
    SmartDashboard.putNumber("speedController", 0);
    SmartDashboard.putNumber("armSpeedControl", 0.5);
  }

  @Override
  public void disabledPeriodic() {
    //nothing yet
  }

  @Override
  public void teleopInit() {
    teleopTimer = new Timer();
    teleopTimer.start();
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("m_leftArm", m_leftArm.get());
    SmartDashboard.putNumber("m_rightArm", m_rightArm.get());
    SmartDashboard.putNumber("m_leftMotor", m_leftMotor.get());
    SmartDashboard.putNumber("m_rightMotor", m_rightMotor.get());
    SmartDashboard.putNumber("m_intakeArm", m_intakeArm.get());
    SmartDashboard.putNumber("m_intake", m_intake.get());

    double speedController = driveStick.getRawAxis(3); //little "dial" thing on the front of the controller
    double speed1 = speedController - 1; //maths
    double speed = speed1 / 2; //more maths

    if(SmartDashboard.getBoolean("driveEnabled", true) == true) {
      m_myRobot.arcadeDrive(driveStick.getY() * speed, -driveStick.getZ() * speed, true); // Use twist for turning.  Square inputs for better control at low speed.
    } else {
      m_myRobot.arcadeDrive(0, 0);
    }

    double leftArmSpeed = MathUtil.applyDeadband(armGamepad.getLeftY(), gamepadDeadband);
    double rightArmSpeed = MathUtil.applyDeadband(armGamepad.getRightY(), gamepadDeadband);

    boolean armRB = armGamepad.getRightBumper(); //button 6 for right bumper
    boolean armLB = armGamepad.getLeftBumper(); //button 5 for left bumper

    if(armRB == true) { //retracts right arm, extends left. Note left arm is inverted.
      leftArmOut = -1;
      rightArmOut = 1;
    } else if(armLB == true) { //retracts left arm, extends right. Note left arm is inverted.
      leftArmOut = 1;
      rightArmOut = -1;
    } else { //if neither bumber is pressed, joysticks take over, we know they aren't pressed because above
        leftArmOut = Math.copySign(Math.abs(leftArmSpeed * (1-armFeedForward)) + armFeedForward, leftArmSpeed); // controller output is % pressed after deadband
        rightArmOut = Math.copySign(Math.abs(rightArmSpeed * (1-armFeedForward)) + armFeedForward, rightArmSpeed); // We multiply by the remaining range and then add the feedforward
        // the copySign stuff is just to maintain the -/+ when adding a scalar
    }

    if(SmartDashboard.getBoolean("armsEnabled", true) == true) {
      m_leftArm.set(leftArmOut);
      m_rightArm.set(rightArmOut);
    } else {
      m_leftArm.set(0);
      m_rightArm.set(0);
    }

    boolean o_armUp = armGamepad.getYButton();
    boolean o_armDown = armGamepad.getAButton();
    boolean o_intakeIn = armGamepad.getXButton();
    boolean o_intakeOut = armGamepad.getBButton();
    boolean d_armUp = (driveStick.getPOV() <= 45 || driveStick.getPOV() >= 315) && driveStick.getPOV() != -1;
    boolean d_armDown = driveStick.getPOV() >= 135 && driveStick.getPOV() <=225;
    boolean d_intakeIn = driveStick.getRawButton(2);
    boolean d_intakeOut = driveStick.getRawButton(1);
    double intakePowerOut = 0;
    double intakeArmPowerOut = 0;

    double armSpeed = SmartDashboard.getNumber("armSpeedControl", 0.5);

    if(d_armUp || d_armDown) {
      if(d_armUp) {
        intakeArmPowerOut = armSpeed;
      } else if(d_armDown) {
        intakeArmPowerOut =-armSpeed;
      } else {
        intakeArmPowerOut = -0.05;
//        m_intakeArm.setVoltage(-10);
      }
    } else {
      if(o_armUp) {
        intakeArmPowerOut = armSpeed;
      } else if(o_armDown) {
        intakeArmPowerOut = -armSpeed;
      } else {
        intakeArmPowerOut = -0.05;
//        m_intakeArm.setVoltage(-10);
      }
    }

    if(SmartDashboard.getBoolean("intakeEnabled", true) == true) { //if intake enabled, set intake arm speed
      m_intakeArm.set(intakeArmPowerOut);
    } else {
      m_intakeArm.set(0);
    }


    if(d_intakeIn || d_intakeOut) {
      if(d_intakeIn) {
        intakePowerOut = 1;
      } else if(d_intakeOut) {
        intakePowerOut = -1;
      } else {
        intakePowerOut = 0;
      }
    } else {
      if(o_intakeIn) {
        intakePowerOut = 1;
      } else if(o_intakeOut) {
        intakePowerOut = -1;
      } else {
        intakePowerOut = 0;
      }
    }

    if(SmartDashboard.getBoolean("intakeEnabled", true) == true) { //if intake enabled, set intake speed
      m_intake.set(intakePowerOut);
    } else {
      m_intake.set(0);
    }
  }

  @Override
  public void autonomousInit() {
    autoTimer = new Timer();
    autoTimer.start();
  }


  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("m_leftArm", m_leftArm.get());
    SmartDashboard.putNumber("m_rightArm", m_rightArm.get());
    SmartDashboard.putNumber("m_leftMotor", m_leftMotor.get());
    SmartDashboard.putNumber("m_rightMotor", m_rightMotor.get());
    SmartDashboard.putNumber("m_intakeArm", m_intakeArm.get());
    SmartDashboard.putNumber("m_intake", m_intake.get());
    
    //get a time for autonomous start to do events based on time later
    double autoTimeElapsed = autoTimer.get(); //get time since start of auto
    if(autoTimeElapsed < 1) { //move for 3 seconds
      m_intake.set(-1);
    } else if(autoTimeElapsed < 4.5) {
      m_intake.set(0);
      m_myRobot.arcadeDrive(-0.5, 0);
      m_intakeArm.set(-0.4);
    } else {
      m_myRobot.arcadeDrive(0, 0);
      m_intakeArm.set(0);
    }
  }
}
