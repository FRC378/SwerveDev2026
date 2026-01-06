// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class CmdDriveClearAll extends Command {

  public CmdDriveClearAll() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("CmdDriveClearAll");
    RobotContainer.m_drivetrain.ResetDriveEncoders();
    RobotContainer.m_drivetrain.ResetTurnEncoders();
    RobotContainer.m_drivetrain.ZeroGyro();
    RobotContainer.m_drivetrain.ResetOdometry();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    //Wait for Gyro to be Calibrated here?
    return true;
  }
}
