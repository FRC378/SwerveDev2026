// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.DriveType;

//   Drive to Absolute coordinate on field, with no regard to current position.
//       
//                         ^  +x
//               front     |
//             +-------+   |
//             |       |   |
//             |   +   |   |
//             |       |   |
//             +-------+   |
//                         |
//     +y <----------------0
//
//   Coordinate System:   
//      +X = Forward
//      +Y = Left
//      +r = CCW rotation
//
//  Heading: Direction FRONT should face during maneuver
//  Speed:   % max speed (range 0 - 1.0)
//  Stop:    Stop when point reached?
//  Timeout: Seconds until timeout (0=disabled)
//
public class CmdDriveToAbsolutePoint extends Command {

  private double m_finalX;
  private double m_finalY;
  private double m_finalH;

  private double m_speed;
  private boolean m_stop;
  private double m_timeout;

  private boolean m_closeEnough;

  private final Timer m_timer = new Timer();

  public CmdDriveToAbsolutePoint(double x, double y, double heading, double speed, boolean stop, double timeout) {
    m_finalX = x;
    m_finalY = y;
    m_finalH = heading;

    m_speed = speed;   //range [0:1.0] - Percent of maximum drive speed
    m_stop = stop;
    m_timeout = timeout;

    m_closeEnough = false;

    addRequirements(RobotContainer.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_closeEnough = false;

    //Start Timer if Timeout is set
    if (m_timeout > 0.0) {
        m_timer.reset();
        m_timer.start();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //-------------------------------------
    //  Directional Computations

    //distance
    double delta_x = m_finalX - RobotContainer.m_drivetrain.GetOdometryX();
    double delta_y = m_finalY - RobotContainer.m_drivetrain.GetOdometryY();
    double distance = Math.hypot(delta_x, delta_y);

    //Are we close enough?
    final double CLOSE_ENOUGH = 1.0; 
    if (distance <= CLOSE_ENOUGH) {
      m_closeEnough = true;
    }

    //Super simple deceleration 
    final double MIN_SPEED = 0.05;   //min speed value
    final double DECEL_DISTANCE = 5.0;   //Distance (inches) to start applying slowdwon

    double speed_adjust = MIN_SPEED + m_speed * (distance / DECEL_DISTANCE);

    if (speed_adjust > m_speed) speed_adjust = m_speed;

    //Unit vectors
    double ux = delta_x / distance;
    double uy = delta_y / distance;

    //Apply vectoring
    double vx = ux * speed_adjust;
    double vy = uy * speed_adjust;

    //-------------------------------------
    //  Rotational correction

    //Min turn power is 0.0625.
    //  Set Kp to reach 0.05 turn power at 1 deg error 
    final double TURN_MAX_VELOCITY = .25; 
    final double TURN_Kp = (0.01 / 1.0);

    double delta_angle = m_finalH - RobotContainer.m_drivetrain.GetGyroYaw();  //getGyroYaw returns [-inf to +inf ]

    double vr = Math.abs(delta_angle * TURN_Kp);

    //Limit max drive
    if (vr > TURN_MAX_VELOCITY) vr = TURN_MAX_VELOCITY;

    //-------------------------------------
    //  Write solution to drivetrain
    //    + delta angle:  +vr to correct (CCW)
    //    - delta angle:  -vr to correct (CW)

    if (delta_angle < 0)
      RobotContainer.m_drivetrain.Drive(vx, vy, -vr, DriveType.FIELDCENTRIC);
    else
      RobotContainer.m_drivetrain.Drive(vx, vy, vr, DriveType.FIELDCENTRIC);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_stop)
      RobotContainer.m_drivetrain.Drive(0, 0, 0, DriveType.FIELDCENTRIC);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Check Distance
    if (m_closeEnough) {
        System.out.println("CmdDriveToAbsolutePoint: CloseEnough");
        return true;
    }

    //Check timer
    if ((m_timeout > 0.0) && m_timer.hasElapsed(m_timeout)) {
        m_timer.stop();
        System.out.println("CmdDriveToAbsolutePoint: Timeout");
        return true;
    }

    return false;
  }
}
