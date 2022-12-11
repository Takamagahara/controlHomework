// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.naming.ldap.Control;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  final int mainSMID = 16;
  final int followerSMID = 17;
  CANSparkMax mainSM;
  CANSparkMax followerSM;

  PS4Controller controller; // GenericHID supports PS5Controller as ps5 controller.

  final int buttonsDriverPort = 1;

  SparkMaxLimitSwitch forwardLimitSwitch = mainSM.getForwardLimitSwitch(Type.kNormallyOpen);
  SparkMaxLimitSwitch reverseLimitSwitch = mainSM.getReverseLimitSwitch(Type.kNormallyOpen);

  ArmFeedforward feedforward;

  //feedforward variables
  double kS = 0.12;
  double kG = 0.65;
  double kV = 49.52;
  double kA = 0.0;

  double currentPosition;
  double resetPosition;

  double forwardPosition, reversePosition;
  
  enum ArmPosition {
    COLLECT,
    SHOOT,
    movetoSHOOT,
    movetoCOLLECT, 
  }

  ArmPosition apoz; // apoz is expected to start either in COLLECT or SHOOT state
  RelativeEncoder encoder;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    mainSM = new CANSparkMax(mainSMID, MotorType.kBrushless);
    followerSM = new CANSparkMax(followerSMID, MotorType.kBrushless);
    
    followerSM.follow(mainSM, true); // defined as followed and inverted

    feedforward = new ArmFeedforward(kS, kG, kV, kA);

    controller = new PS4Controller(buttonsDriverPort);

    currentPosition = encoder.getPosition();

    if (forwardLimitSwitch.isPressed()) {
      apoz = ArmPosition.COLLECT;
      encoder.setPosition(-107.26);
      forwardPosition = -107.26;
    } else {
      apoz = ArmPosition.SHOOT;
      encoder.setPosition(35.60);
      reversePosition = 35.60;
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  void moveArm(double speed) {
    double speedFactor = 0;

    if (apoz == ArmPosition.movetoSHOOT || apoz == ArmPosition.movetoCOLLECT) {
      speedFactor = (apoz == ArmPosition.movetoCOLLECT) ? 1 : -1;
    } 

    speed *= speedFactor;

    mainSM.set(speed); // followerSM is following mainSM
  }


  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {    
    if (forwardLimitSwitch.isPressed() || reverseLimitSwitch.isPressed())
      moveArm(0); // insted of deprecated "enableLimitSwitch" function;

    switch (apoz) {
      case COLLECT:
        if (controller.getCrossButtonPressed()) {
          apoz = ArmPosition.movetoSHOOT;
          moveArm(0.1);      
        }
        break;

      case movetoSHOOT:
        if (controller.getCrossButtonPressed()) {
          apoz = ArmPosition.movetoCOLLECT;
          moveArm(0.1);
        }
        break;
      
      case SHOOT:
      if (controller.getCrossButtonPressed()) {
        apoz = ArmPosition.movetoCOLLECT;
        moveArm(0.1);
      }
      break;

      case movetoCOLLECT:
      if (controller.getCrossButtonPressed()) {
        apoz = ArmPosition.movetoSHOOT;
        moveArm(0.1);
      }
      break;
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

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
