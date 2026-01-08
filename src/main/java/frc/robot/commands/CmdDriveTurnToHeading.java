// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.DriveType;

public class CmdDriveTurnToHeading extends Command {

  private double m_heading;
  private double m_power;


  public CmdDriveTurnToHeading(double heading, double power) {
    m_heading = heading;
    m_power = power;
    addRequirements(RobotContainer.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double MAX_POWER = m_power;   // % of kMaxRotation in Drivetrain
    final double MIN_POWER = 0.01;     // % of kMaxRotation in Drivetrain
    final double TURN_Kp   = 0.05;

    double delta_angle = m_heading - RobotContainer.m_drivetrain.GetOdometryHeading();   // [-180 to +180]

    //180 discontinuity handler
    if(  delta_angle >  180.0) delta_angle -= 360.0; 
    if(  delta_angle < -180.0) delta_angle += 360.0;

    double turn_power = Math.abs( delta_angle * TURN_Kp );

    if( turn_power > MAX_POWER ) turn_power = MAX_POWER;
    if( turn_power < MIN_POWER ) turn_power = MIN_POWER;

    if( delta_angle < 0)
    RobotContainer.m_drivetrain.Drive(0,0,  -turn_power,  DriveType.ROBOTCENTRIC);
    else
    RobotContainer.m_drivetrain.Drive(0,0,   turn_power,  DriveType.ROBOTCENTRIC);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_drivetrain.Drive(0, 0, 0, DriveType.ROBOTCENTRIC);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double delta_angle = m_heading - RobotContainer.m_drivetrain.GetOdometryHeading();

    if(  Math.abs(delta_angle) < 1.5 )
      return true;

    return false;
  }
}

