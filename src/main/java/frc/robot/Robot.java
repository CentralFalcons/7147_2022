// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Math;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import javax.tools.ForwardingFileObject;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import org.ejml.dense.row.decompose.hessenberg.TridiagonalDecompositionHouseholder_CDRM;

import com.ctre.phoenix.motorcontrol.NeutralMode;
//This link specifies how to download the REV library https://www.revrobotics.com/content/sw/max/sdk/REVRobotics.json
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADIS16448_IMU;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  CANSparkMax m_intakearm = new CANSparkMax(6, MotorType.kBrushless);
  MotorController m_intakebar = new WPI_VictorSPX(5);

  WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(1);
  WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(2);
  MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);

  WPI_TalonSRX m_frontRight = new WPI_TalonSRX(3);
  WPI_TalonSRX m_rearRight = new WPI_TalonSRX(4);
  MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_rearRight);

  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  XboxController driver_controller = new XboxController(1);
  XboxController arm_controller = new XboxController(0);

  double referenceAngle;
  ADIS16448_IMU gyro = new ADIS16448_IMU();
  /* Variables for timed actions */
  /* How many cycles should you run the bumper */
  int bumperRunTime;
  // Time the match started
  double matchStartTime;
  // Time autonomous started
  double autoStartTime;
  boolean turnFinished = false;
  boolean secondTurnFinished = false;
  double turnFinishedTime;
  double secondTurnFinishedTime;
  double[] lastSpeeds = {0, 0, 0, 0, 0};

  /* Tunable constants */
  static double ARM_SPEED = 0.3; // 0 - 1: Speed of raising / lowering arm
  static double BAR_SPEED = 1.0; // 0 - 1: Speed of intake bar
  static double DRIVE_SENSITIVITY = 1.0; // 0 - 1: Speed of drive motors
  static double TARGETANGLE = 180;
  static double TARGETANGLEWINDOW = 3;
  static double TURNPOWER = .6;
  static double FINISHED_SPEED = 0.1; // angular speed to finish the turn
  static double AUTO_SPEED = 0.5; // speed of drive motors during auto

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // The currnt amount of time elapsed since the start of autonomous mode is
    // determined by referencing this.
    matchStartTime = System.currentTimeMillis();

    /*
     * Under full load the system browned out at our Saline competition in 2022.
     * This was likely due to a faulty motor, so the amperage to each motor is here
     * limited to 40 Amps.
     */
    m_frontLeft.configSupplyCurrentLimit(new
      SupplyCurrentLimitConfiguration(true, 40.0, 40.0, 0.01));
    m_frontRight.configSupplyCurrentLimit(new
      SupplyCurrentLimitConfiguration(true, 40.0, 40.0, 0.01));
    m_rearLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
    40.0, 40.0, 0.01));
    m_rearRight.configSupplyCurrentLimit(new
    SupplyCurrentLimitConfiguration(true, 40.0, 40.0, 0.01));

    // This code is copypasta, and its use is not clear to me.
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_drive.tankDrive(0, 0); // Make sure the robot isn't moving right away!
    m_left.setInverted(true); // The left motors are inverted so we can drive staright.
    m_intakearm.setIdleMode(CANSparkMax.IdleMode.kBrake); // The arm falls violently without braking from the NEO motor.
    m_frontLeft.setNeutralMode(NeutralMode.Brake);
    m_frontRight.setNeutralMode(NeutralMode.Brake);
    m_rearLeft.setNeutralMode(NeutralMode.Brake);
    m_rearRight.setNeutralMode(NeutralMode.Brake);
  }

  public double timeChange(double mm) {
    // This function changes mm to miliseconds
    // The coefficients here were determined in a callibration experiment on a waxed
    // tile floor, without bumpers.
    return Math.floor((mm - 42) / 0.45381); //0.5 speed
    //return Math.floor((mm + 202.4) / 3.654); // 1.0 speed
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    autoStartTime = System.currentTimeMillis();
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    /* Calibrate the gyroscope */
    gyro.calibrate();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    double angle = gyro.getAngle();

    // TODO: Smartdash board should be used for logging.
    System.out.println("Auto is working.");
    System.out.println("Currently pointing:" + angle);
    double elapsedTime = System.currentTimeMillis() - autoStartTime; // This timestamp is used for the robot actions
    /*
     * This autonomous mode code is designed to start with the robot flush to the
     * lower port with one pre-loaded cargo.
     * The robot scores, backs up, and stops.
     * First, the robot spins the intake backwards to score for 2 seconds.
     * Second, the robot reverses for 3 meters.
     * Third, the robot stops.
     */
    int SHOOTTIME = 500; 
    if(elapsedTime < SHOOTTIME){
      m_intakebar.set(-1);
    }else if(elapsedTime > SHOOTTIME & elapsedTime < SHOOTTIME + timeChange(47.625*25.4*2.5) & !turnFinished){
      if(elapsedTime < SHOOTTIME + timeChange(12*25.4)){
        m_intakearm.set(-0.1);
      }
      m_intakebar.set(0);
      m_drive.tankDrive(AUTO_SPEED, AUTO_SPEED);
      /*
    }else if(elapsedTime > SHOOTTIME + timeChange(47.625*25.4) & !turnFinished){
      m_intakearm.stopMotor();
      if (angle < TARGETANGLE - TARGETANGLEWINDOW ) {
        m_drive.tankDrive(-TURNPOWER, TURNPOWER);
      } else if (angle > TARGETANGLE + TARGETANGLEWINDOW) {
        m_drive.tankDrive(TURNPOWER, -TURNPOWER);
      } else {
        m_drive.stopMotor();// tankDrive(0, 06);
        lastSpeeds[0] = lastSpeeds[1];
        lastSpeeds[1] = lastSpeeds[2];
        lastSpeeds[2] = lastSpeeds[3];
        lastSpeeds[3] = lastSpeeds[4];
        lastSpeeds[4] = gyro.getRate();
        double currentSpeed = (lastSpeeds[0] + lastSpeeds[1] + lastSpeeds[2] + lastSpeeds[3] + lastSpeeds[4])/5;
        if (currentSpeed < FINISHED_SPEED){
          turnFinishedTime = System.currentTimeMillis();
          turnFinished = true;
          System.out.println("Turn Finished.");
        }
      }
    }else if((System.currentTimeMillis() - turnFinishedTime) < timeChange(35*25.4)){
      m_drive.tankDrive(-AUTO_SPEED, -AUTO_SPEED);
      m_intakebar.set(0.6);
    /*
    }else if((System.currentTimeMillis() - turnFinishedTime) > timeChange(35*25.4) & !secondTurnFinished){
      // Turn back around
      m_intakebar.stopMotor();
      if (angle < 360 - TARGETANGLEWINDOW ) {
        m_drive.tankDrive(-TURNPOWER, TURNPOWER);
      } else if (angle > 360 + TARGETANGLEWINDOW) {
        m_drive.tankDrive(TURNPOWER, -TURNPOWER);
      } else {
        m_drive.stopMotor();
        lastSpeeds[0] = lastSpeeds[1];
        lastSpeeds[1] = lastSpeeds[2];
        lastSpeeds[2] = lastSpeeds[3];
        lastSpeeds[3] = lastSpeeds[4];
        lastSpeeds[4] = gyro.getRate();
        double currentSpeed = (lastSpeeds[0] + lastSpeeds[1] + lastSpeeds[2] + lastSpeeds[3] + lastSpeeds[4])/5;
        if (currentSpeed < FINISHED_SPEED){
          secondTurnFinishedTime = System.currentTimeMillis();
          secondTurnFinished = true;
          System.out.println("Second Turn Finished.");
        }
      }
    }else if((System.currentTimeMillis() - secondTurnFinishedTime) < timeChange(1500)){
      // Drive forward
      m_intakearm.set(0.8);
      m_drive.tankDrive(-0.8,-0.8);
    }else if((System.currentTimeMillis() - secondTurnFinishedTime) > timeChange(1500) &
              (System.currentTimeMillis() - secondTurnFinishedTime) < timeChange(2000)){
      // Score
      m_intakebar.set(-1);
      m_intakearm.set(0);
    */
    }else{
      m_drive.stopMotor();
      m_intakebar.stopMotor();
      m_intakearm.stopMotor();
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /*
     * Y and the left bumper raise the arm.
     * A lowers the arm.
     * X runs the intake to pick up balls.
     * Y runs the intake to shoot balls.
     */
    m_drive.tankDrive(DRIVE_SENSITIVITY * driver_controller.getLeftY(),
        DRIVE_SENSITIVITY * driver_controller.getRightY());
    if (arm_controller.getYButton()) {
      m_intakearm.set(ARM_SPEED);
    } else if (arm_controller.getAButton()) {
      m_intakearm.set(-ARM_SPEED);
    } else if (arm_controller.getRightBumper()) {
      m_intakearm.set(ARM_SPEED);
    } else {
      m_intakearm.set(0);
    }
    if (arm_controller.getXButton()) {
      m_intakebar.set(BAR_SPEED);
    } else if (arm_controller.getBButton()) {
      m_intakebar.set(-BAR_SPEED);
    } else {
      m_intakebar.set(0);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
