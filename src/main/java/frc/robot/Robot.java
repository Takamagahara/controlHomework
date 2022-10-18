// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.naming.ldap.Control;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

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

    mainSM = new CANSparkMax(mainSMID, MotorType.kBrushed);
    followerSM = new CANSparkMax(followerSMID, MotorType.kBrushed);

    controller = new PS4Controller(buttonsDriverPort);
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

    if (apoz == ArmPosition.owREVERSE || apoz == ArmPosition.owFORWARD) {
      speedFactor = (apoz == ArmPosition.owFORWARD) ? 1 : -1;
    } 

    speed *= speedFactor;

    mainSM.set(speed);
    followerSM.set(speed);
  }

  enum ArmPosition {
    FORWARD,
    REVERSE,
    owREVERSE, // on the way from forward to reverse (forward -> reverse)
    owFORWARD, // on the way from reverse to forward (reverse -> forward)
    _unrecognized; // fail safe (?)
  }

  ArmPosition apoz; // put value in SmartDashboard (?)

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SparkMaxLimitSwitch buttonStatusForwardRaw = mainSM.getForwardLimitSwitch(Type.kNormallyOpen);
    SparkMaxLimitSwitch buttonStatusReverseRaw = followerSM.getReverseLimitSwitch(Type.kNormallyOpen);
    if (buttonStatusForwardRaw.isPressed()) {
      apoz = ArmPosition.FORWARD;
      moveArm(0);
    } else if (buttonStatusReverseRaw.isPressed()) {
      apoz = ArmPosition.REVERSE;
      moveArm(0);
    } else {
      apoz = ArmPosition._unrecognized;
      moveArm(0);
    }

    if (controller.getCrossButtonPressed() && apoz == ArmPosition.FORWARD) {
      // owREVERSE
      apoz = ArmPosition.owREVERSE;
      moveArm(0.10);
    } else if (controller.getCrossButtonPressed() && apoz == ArmPosition.REVERSE) {
      // owFORWARD
      apoz = ArmPosition.owFORWARD;
      moveArm(0.10);
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
