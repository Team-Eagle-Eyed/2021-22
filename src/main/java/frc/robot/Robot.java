package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
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
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  //variables galore

  private PowerDistribution mainPDB;

  private DifferentialDrive m_myRobot;
  private DifferentialDrive m_myRobot2;
  
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
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_rightMotor2;

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
    mainPDB = new PowerDistribution(0, ModuleType.kCTRE);

    m_leftMotor = new CANSparkMax(leftDriveID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(rightDriveID, MotorType.kBrushless);
    m_leftMotor2 = new CANSparkMax(6, MotorType.kBrushless);
    m_rightMotor2 = new CANSparkMax(5, MotorType.kBrushless);
    
    m_leftArm = new CANSparkMax(leftArmID, MotorType.kBrushless);
    m_rightArm = new CANSparkMax(rightArmID, MotorType.kBrushless);
    m_intakeArm = new VictorSP(8);
    m_intake = new PWMSparkMax(9);

    m_rightArm.setOpenLoopRampRate(0.1); //ramp rate so the motor cant stall when jumping to full power
    m_leftArm.setOpenLoopRampRate(0.1);

    m_rightArm.setIdleMode(IdleMode.kBrake); //brake so the arms don't coast past the limit switches
    m_leftArm.setIdleMode(IdleMode.kBrake);

    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_myRobot2 = new DifferentialDrive(m_leftMotor2, m_rightMotor2);
    
    m_myRobot.setDeadband(0.05);
    m_myRobot2.setDeadband(0.05);

    driveStick = new Joystick(0); //big joystick thing
    armGamepad = new XboxController(1); //small standard console controller
    
    m_leftArm.setInverted(true); // backwards, to add more confusion
    m_leftMotor.setInverted(false);
    m_leftMotor2.setInverted(false);
    m_rightMotor.setInverted(true);
    m_rightMotor2.setInverted(true);

    CameraServer.startAutomaticCapture(); //woo I have eyes

    SmartDashboard.putNumber("intakeWinchCurrent", 0);

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

    SmartDashboard.putNumber("speedCap", 1);
    SmartDashboard.putNumber("intake", 0.9);
    SmartDashboard.putNumber("autoDelay", 0);
  }

  @Override
  public void disabledPeriodic() {
    //nothing yet
  }

  @Override
  public void teleopInit() {
    teleopTimer = new Timer();
    teleopTimer.start();
    m_rightMotor.setOpenLoopRampRate(0.12);
    m_leftMotor.setOpenLoopRampRate(0.12);
    m_rightMotor2.setOpenLoopRampRate(0.12);
    m_leftMotor2.setOpenLoopRampRate(0.12);
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("intakeWinchCurrent", mainPDB.getCurrent(3));

    SmartDashboard.putNumber("m_leftArm", m_leftArm.get());
    SmartDashboard.putNumber("m_rightArm", m_rightArm.get());
    SmartDashboard.putNumber("m_leftMotor", m_leftMotor.get());
    SmartDashboard.putNumber("m_rightMotor", m_rightMotor.get());
    SmartDashboard.putNumber("m_intakeArm", m_intakeArm.get());
    SmartDashboard.putNumber("m_intake", m_intake.get());

    double speedController = driveStick.getRawAxis(3); //little "dial" thing on the front of the controller
    double speed1 = speedController - 1; //maths
    double speed = speed1 / 2; //more maths
    SmartDashboard.putNumber("speedController", speed * -1);
    double speedCap = SmartDashboard.getNumber("speedCap", 0.8);

    if(SmartDashboard.getBoolean("driveEnabled", true) == true) {
      m_myRobot.arcadeDrive(driveStick.getY() * speed * speedCap, -driveStick.getZ() * speed * speedCap * 0.75, true); // Use twist for turning.  Square inputs for better control at low speed.
      m_myRobot2.arcadeDrive(driveStick.getY() * speed * speedCap, -driveStick.getZ() * speed * speedCap * 0.75, true);
    } else {
      m_myRobot.arcadeDrive(0, 0);
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

    double armSpeed = SmartDashboard.getNumber("intakeArmSpeedControl", 0.5);

/*     double teleopElapsed = teleopTimer.get();
    double timeLimitExpires;
    double intakeArmSpeedMult;
    if(mainPDB.getCurrent(3) >= 12) { //if amperage is above a threshhold
      timeLimitExpires = teleopElapsed + 1;
    } else {
      timeLimitExpires = teleopElapsed;
    }

    if(timeLimitExpires < teleopElapsed) {
      intakeArmSpeedMult = 0.3;
    } else {
      intakeArmSpeedMult = 1;
    } */

    if(o_armUp || o_armDown) {
      if(o_armUp) {
        intakeArmPowerOut = armSpeed/*  * intakeArmSpeedMult */;
      } else if(o_armDown) {
        intakeArmPowerOut =-armSpeed/*  * intakeArmSpeedMult */;
      } else {
        intakeArmPowerOut = -0.05;
//        m_intakeArm.setVoltage(-10);
      }
    } else {
      if(d_armUp && driveStick.getRawButton(7) == true) {
        intakeArmPowerOut = armSpeed;
      } else if(d_armDown && driveStick.getRawButton(7) == true) {
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


    if(o_intakeIn || o_intakeOut) {
      if(o_intakeIn) {
        intakePowerOut = 1;
      } else if(o_intakeOut) {
        intakePowerOut = -1;
      } else {
        intakePowerOut = 0;
      }
    } else {
      if(d_intakeIn && driveStick.getRawButton(7) == true) {
        intakePowerOut = 1;
      } else if(d_intakeOut && driveStick.getRawButton(7) == true) {
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
    m_rightMotor.setOpenLoopRampRate(0.2);
    m_leftMotor.setOpenLoopRampRate(0.2);
    m_rightMotor2.setOpenLoopRampRate(0.2);
    m_leftMotor2.setOpenLoopRampRate(0.2);
  }


  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("m_leftArm", m_leftArm.get());
    SmartDashboard.putNumber("m_rightArm", m_rightArm.get());
    SmartDashboard.putNumber("m_leftMotor", m_leftMotor.get());
    SmartDashboard.putNumber("m_rightMotor", m_rightMotor.get());
    SmartDashboard.putNumber("m_intakeArm", m_intakeArm.get());
    SmartDashboard.putNumber("m_intake", m_intake.get());
    double getAutoDelay = SmartDashboard.getNumber("autoDelay", 0);
    
    //get a time for autonomous start to do events based on time later
    double autoTimeElapsed = autoTimer.get(); //get time since start of auto
    if(autoTimeElapsed < getAutoDelay) {
      m_myRobot.feed();
      m_myRobot2.feed();
    } else if(autoTimeElapsed < 1 + getAutoDelay) { //move for 3 seconds
      m_intake.set(-1);
    } else if(autoTimeElapsed < 3 + getAutoDelay) {
      m_intake.set(0);
      m_myRobot.arcadeDrive(-0.7, 0);
      m_myRobot2.arcadeDrive(-0.7, 0);
      m_intakeArm.set(-0.5);
    } else {
      m_myRobot.arcadeDrive(0, 0);
      m_myRobot.arcadeDrive(0, 0);
      m_intakeArm.set(0);
      m_intake.set(0);
    }
  }
}