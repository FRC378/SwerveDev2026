// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;


public class CmdDriveWithGamepad extends Command {
  /** Creates a new CmdDriveWithGamepad. */
  public CmdDriveWithGamepad() {

    addRequirements(RobotContainer.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("CmdDriveWithGamepad Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    final double DEADBAND = 0.05;

    double leftY  = -edu.wpi.first.math.MathUtil.applyDeadband(RobotContainer.m_driver.getLeftY(),  DEADBAND);
    double leftX  = -edu.wpi.first.math.MathUtil.applyDeadband(RobotContainer.m_driver.getLeftX(),  DEADBAND);
    double rightX = -edu.wpi.first.math.MathUtil.applyDeadband(RobotContainer.m_driver.getRightX(), DEADBAND);


    //Define default Drive Power
    double xyScaleValue  = 0.3;
    double rScaleValue   = 0.5;  

    //Apply scaling
    double fwdrev    = leftY  * xyScaleValue; 
    double rightleft = leftX  * xyScaleValue;    
    double rotate    = rightX * rScaleValue;


    //Call Drive!
    RobotContainer.m_drivetrain.drive(fwdrev, rightleft, rotate, RobotContainer.m_drivetrain.GetDriveType() );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("CmdDriveWithGamepad End");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //Never ends!
  }
}
