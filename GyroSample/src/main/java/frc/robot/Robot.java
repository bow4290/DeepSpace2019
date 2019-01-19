/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
//import sun.jvm.hotspot.code.AdapterBlob;


/**
 * This is a sample program to demonstrate how to use a gyro sensor to make a
 * robot drive straight. This program uses a joystick to drive forwards and
 * backwards while the gyro is used for direction keeping.
 */
public class Robot extends TimedRobot {
  private static final double kAngleSetpoint = 0;
  private static final double kP = 100.005; // propotional turning constant

  // gyro calibration constant, may need to be adjusted;
  // gyro value of 360 is set to correspond to one full revolution
  private static final double kVoltsPerDegreePerSecond = 0.0128;

  private static final int kLeftMotorPort = 2;
  private static final int kRightMotorPort = 3;
  private static final int kGyroPort = 0;
  private static final int kJoystickPort = 0;
  private NetworkTable table = NetworkTable.getTable("datatable");


  private final DifferentialDrive m_myRobot
      = new DifferentialDrive(new PWMVictorSPX(kLeftMotorPort),
      new PWMVictorSPX(kRightMotorPort));
 // public static AnalogGyro m_gyro;
  public static ADXRS450_Gyro m_gyro;
  private final Joystick m_joystick = new Joystick(kJoystickPort);

  @Override
  public void robotInit() {
    //m_gyro = new AnalogGyro(kGyroPort);]
    m_gyro =  new ADXRS450_Gyro();
    double angleGyro = m_gyro.getAngle();
    double turningValue = (kAngleSetpoint - angleGyro) * kP;
    table.putNumber("Current Gyro", angleGyro);
    table.putNumber("Turning Value", turningValue);
    

    // m_gyro.setSensitivity(kVoltsPerDegreePerSecond);
  }

  /**
   * The motor speed is set from the joystick while the RobotDrive turning
   * value is assigned from the error between the setpoint and the gyro angle.
   */
  @Override
  public void teleopPeriodic() {
    double angleGyro = m_gyro.getAngle();
    double turningValue = (kAngleSetpoint - angleGyro) * kP;

    // Invert the direction of the turn if we are going backwards
    // turningValue = Math.copySign(turningValue, m_joystick.getY());
    //m_myRobot.arcadeDrive(m_joystick.getY(), turningValue);
    table.putNumber("Current Gyro", angleGyro);
    table.putNumber("Turning Value", turningValue);
    

  }
}
