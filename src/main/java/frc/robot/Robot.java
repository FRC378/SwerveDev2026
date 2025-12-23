// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {

    //*************************** INIT ******************************
    System.out.println("Robot Init");
    System.out.println(" ~~~ SwerveDev2026 ~~~~ ");


    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {

    WriteToSmartDashboard();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    System.out.println("DisabledInit");
  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void disabledExit() {

  }

  @Override
  public void autonomousInit() {
    System.out.println("AutonomousInit");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    System.out.println("TeleopInit");

  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}


  //---------------------------------------------------------------------------
  public void WriteToSmartDashboard() {

    SmartDashboard.putNumber("Match Time", Timer.getMatchTime() );
    SmartDashboard.putNumber("TimeStamp",  Timer.getTimestamp() );
    SmartDashboard.putNumber("FPGATimeStamp",  Timer.getFPGATimestamp() );

    //******   XBox Controller
    SmartDashboard.putNumber("LeftX",  RobotContainer.m_driver.getLeftX() );
    SmartDashboard.putNumber("LeftY",  RobotContainer.m_driver.getLeftY() );
    SmartDashboard.putNumber("RightX",  RobotContainer.m_driver.getRightX() );
    SmartDashboard.putNumber("RightY",  RobotContainer.m_driver.getRightY() );


    SmartDashboard.putNumber("Volts", RobotContainer.m_pdp.getVoltage() );


  }





}
