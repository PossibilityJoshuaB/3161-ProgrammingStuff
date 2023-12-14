// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private DifferentialDrive m_myRobot;
  private Joystick joystick;

  private final CANSparkMax m_leftMotor = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax m_leftFollower = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax m_rightMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_rightFollower = new CANSparkMax(2, MotorType.kBrushless);

  private double startTime;
  @Override
  public void robotInit() {
    m_leftFollower.restoreFactoryDefaults();
    m_leftMotor.restoreFactoryDefaults();
    m_rightFollower.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();

    m_leftFollower.follow(m_leftMotor);
    m_rightFollower.follow(m_rightMotor);

    m_leftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
    joystick = new Joystick(0);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    double time = Timer.getFPGATimestamp();

    if(time-startTime < 2) {
      m_leftFollower.set(0.1);
      m_leftMotor.set(0.1);
      m_rightFollower.set(-0.1);
      m_rightMotor.set(-0.1);
    } else {
      m_leftFollower.set(0);
      m_leftMotor.set(0);
      m_rightFollower.set(0);
      m_rightMotor.set(0);
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(-joystick.getY() - joystick.getX(), joystick.getY() - joystick.getX());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
