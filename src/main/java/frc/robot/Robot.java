// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Imports that allow the usage of REV Spark Max motor controllers
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

// Import NetworkTables
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleSubscriber;

// Import SmartDashboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Import XBoxController
import edu.wpi.first.wpilibj.XboxController;

// Import FRC Defaults
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  // XBoxController
  XboxController controller = new XboxController(0);
  
  // Timer related variables
  private final Timer m_timer = new Timer();
  Double timeElapsed;
  Double autonomousStartTime;

  // Define Motors from device IDs
  CANSparkBase leftRear = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkBase leftFront = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkBase rightRear = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkBase rightFront = new CANSparkMax(2, MotorType.kBrushless);

  private final DifferentialDrive m_drivetrain = new DifferentialDrive(leftFront, rightFront);

  CANSparkBase m_launchWheel = new CANSparkMax(9, MotorType.kBrushless);
  CANSparkBase m_launchWheel2 = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkBase m_feedWheel = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkBase m_feedWheel2 = new CANSparkMax(8, MotorType.kBrushless);

  // How many amps can an individual motor use.
  static final int Motor_CurrentLimit = 60;

  // Percent output to run the feeder and launcher motors when intaking note
  static final double Feed_Speed = -.4;

  // Percent output to run the launcher when expelling note
  static final double Launch_Speed = 1.0;

  // Network Tables Subscription
  NetworkTable visiontable;
  DoubleSubscriber visionsubx;
  DoubleSubscriber visionsuby;
  Double X_Axis;
  Double Y_Axis;
  Integer teleautonomous;

  // Robot class constructor 
  public Robot() {

  }

  // This function is run when the robot is first started
  @Override
  public void robotInit() {
    // Subscribe to Network Table Vision and get X and Y value subscriptions
    visiontable = NetworkTableInstance.getDefault().getTable("Vision");
    visionsubx = visiontable.getDoubleTopic("X_Axis").subscribe(0.00);
    visionsuby = visiontable.getDoubleTopic("Y_Axis").subscribe(0.00);

    // Apply current limit to the drivetrain motors
    leftRear.setSmartCurrentLimit(Motor_CurrentLimit);
    leftFront.setSmartCurrentLimit(Motor_CurrentLimit);
    rightRear.setSmartCurrentLimit(Motor_CurrentLimit);
    rightFront.setSmartCurrentLimit(Motor_CurrentLimit);

    // Ramp up drive motors over 0.5 seconds (Should smooth out driving)
    leftRear.setOpenLoopRampRate(0.5);
    leftFront.setOpenLoopRampRate(0.5);
    rightRear.setOpenLoopRampRate(0.5);
    rightFront.setOpenLoopRampRate(0.5);

    // Set the rear wheels to follow the same commands as the front wheels
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    // Invert the Left Front Wheel as motors are facing opposite directions
    leftFront.setInverted(true);

    // Apply the current limit to the launching mechanism
    m_feedWheel.setSmartCurrentLimit(Motor_CurrentLimit);
    m_feedWheel2.setSmartCurrentLimit(Motor_CurrentLimit);
    m_launchWheel.setSmartCurrentLimit(Motor_CurrentLimit);
    m_launchWheel2.setSmartCurrentLimit(Motor_CurrentLimit);

    // Invert the launch and feed wheels on one side
    m_launchWheel.setInverted(false);
    m_feedWheel.setInverted(false);
    m_launchWheel2.setInverted(true);
    m_feedWheel2.setInverted(true);

    // Temporarily disable this feed wheel until it is fixed
    m_feedWheel2.disable();

    teleautonomous = 0; 
  }

  // This function is called every 20 ms, no matter the mode. 
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
  }

  // This function is run once each time the robot enters autonomous mode. 
  @Override
  public void autonomousInit() {
    m_timer.restart();

    leftRear.setIdleMode(IdleMode.kBrake);
    leftFront.setIdleMode(IdleMode.kBrake);
    rightRear.setIdleMode(IdleMode.kBrake);
    rightFront.setIdleMode(IdleMode.kBrake);

    autonomousStartTime = Timer.getFPGATimestamp();
  }

  // This function is called periodically during autonomous. 
  @Override
  public void autonomousPeriodic() {
    // get elapsed time
    timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

    // get current rapberry pi drive controller values
    X_Axis = visionsubx.get();
    Y_Axis = visionsuby.get();

    // Drive around based on those values
    m_drivetrain.curvatureDrive(Y_Axis, X_Axis, true);

  }

  // This function is called once each time the robot enters teleoperated mode. 
  @Override
  public void teleopInit() {
    // Set all motors to coast
    leftRear.setIdleMode(IdleMode.kBrake);
    leftFront.setIdleMode(IdleMode.kBrake);
    rightRear.setIdleMode(IdleMode.kBrake);
    rightFront.setIdleMode(IdleMode.kBrake);
    m_launchWheel.setIdleMode(IdleMode.kCoast);
    m_feedWheel.setIdleMode(IdleMode.kBrake);
    m_launchWheel2.setIdleMode(IdleMode.kCoast);
    m_feedWheel2.setIdleMode(IdleMode.kBrake);  
  }

  // This function is called periodically during teleoperated mode. 
  @Override
  public void teleopPeriodic() {

    // Launch Button - Left Bumper (and not Right Bumper)
    if (controller.getLeftBumper() && !controller.getRightBumper()) {     
      m_launchWheel.set(Launch_Speed);
      m_launchWheel2.set(Launch_Speed);
      m_feedWheel.set(Launch_Speed);
      m_feedWheel2.set(Launch_Speed);     
    // Feed Button - Right Bumper
    } else if (controller.getRightBumper()) {
      m_launchWheel.set(Feed_Speed);
      m_launchWheel2.set(Feed_Speed);
      m_feedWheel.set(Feed_Speed);
      m_feedWheel2.set(Feed_Speed);
    } else {
      m_launchWheel.stopMotor();
      m_launchWheel2.stopMotor();
      m_feedWheel.stopMotor();
      m_feedWheel2.stopMotor();
    }

    // If you press the start button, toggle teleautonmous on or off
    if (controller.getStartButtonReleased())
      teleautonomous += 1;



    if (teleautonomous % 3 == 0) {
      // Drive with Left Controller Stick
      m_drivetrain.arcadeDrive(controller.getLeftY(),controller.getLeftX());
    } else if (teleautonomous % 3 == 1) {
      // get current rapberry pi drive controller values
      X_Axis = visionsubx.get();
      Y_Axis = visionsuby.get();
      m_drivetrain.curvatureDrive(controller.getLeftY(), controller.getLeftX() + X_Axis, true);
    } else if (teleautonomous % 3  == 2) {
      // get current rapberry pi drive controller values
      X_Axis = visionsubx.get();
      Y_Axis = visionsuby.get();
      
      m_drivetrain.curvatureDrive(Y_Axis, X_Axis, true);
    }
      
  }

  // This function is called once each time the robot enters test mode. 
  @Override
  public void testInit() {}

  // This function is called periodically during test mode. 
  @Override
  public void testPeriodic() {}
}
