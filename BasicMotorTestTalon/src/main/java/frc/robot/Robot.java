/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and speed controller inputs also
 * range from -1 to 1 making it easy to work together.
 */
public class Robot extends TimedRobot {
  private static final int kMotorPort = 5;
  private static final int kJoystickPort = 0;

  private Talon m_motor;
  private Joystick m_joystick;

  @Override
  public void robotInit() {
    m_motor = new Talon(kMotorPort);
    m_joystick = new Joystick(kJoystickPort);
  }

  @Override
  public void teleopPeriodic() {
    m_motor.set(-.2);
  }
}
