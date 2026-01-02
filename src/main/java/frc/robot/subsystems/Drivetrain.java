// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  public static final double kMaxVelocity = 12.0;   // Feet per sec
  public static final double kMaxRotation = 270.0;  // degrees per second


  // Coordinates of swerve modules for kinematic and odometry calculations
  //  Positive x values represent moving toward the front of the robot whereas positive y values represent moving toward the left of the robot.
  //  
  //    BL        FL    +Y
  //     +--------+      ^
  //     |        >>     |     
  //     |        >>    -|----> +X
  //     +--------+      |
  //    BR        FR
  //
  private final Translation2d m_frontLeftLocation  = new Translation2d(  Constants.DRIVEBASE_LENGTH/2.0 ,   Constants.DRIVEBASE_WIDTH/2.0 );
  private final Translation2d m_frontRightLocation = new Translation2d(  Constants.DRIVEBASE_LENGTH/2.0 ,  -Constants.DRIVEBASE_WIDTH/2.0 );
  private final Translation2d m_backLeftLocation   = new Translation2d( -Constants.DRIVEBASE_LENGTH/2.0 ,   Constants.DRIVEBASE_WIDTH/2.0 );
  private final Translation2d m_backRightLocation  = new Translation2d( -Constants.DRIVEBASE_LENGTH/2.0 ,  -Constants.DRIVEBASE_WIDTH/2.0 );

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics( m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  // Swerve Modules
  private SwerveModule m_frontLeft =  new SwerveModule( Constants.FRONTLEFT_DRIVE_CAN_ID, Constants.FRONTLEFT_TURN_CAN_ID, Constants.FRONTLEFT_ENCODER_ID,Constants.FRONTLEFT_ENCODER_OFFSET,  "FL");;



  public Drivetrain() {
    System.out.println("Drivetrain Init");

    m_frontLeft.ResetTurnEncoder();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void drive(double xValue, double yValue, double rValue  )
  {

    //Velocity control
    final double xSpeed = xValue * kMaxVelocity;
    final double ySpeed = yValue * kMaxVelocity;
    final double rSpeed = rValue * Math.toRadians(kMaxRotation);

    final boolean fieldRelative = false;

    //Magic Voodoo call to calculate swerve math!
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds( xSpeed, ySpeed, rSpeed, new Rotation2d(0.0) )
                    : new ChassisSpeeds(xSpeed, ySpeed, rSpeed),
                0.02));

    //Normalize velocities
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxVelocity);

    //Set Desired States
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    //m_frontRight.setDesiredState(swerveModuleStates[1]);
    //m_backLeft.setDesiredState(swerveModuleStates[2]);
    //m_backRight.setDesiredState(swerveModuleStates[3]);

    double vel = Math.hypot(xSpeed, ySpeed);
    SmartDashboard.putNumber("xyVelocity", vel);

  }


  public void ForceAllTurnAngle( double angle ) {
    Rotation2d rot = Rotation2d.fromDegrees(angle);
    m_kinematics.resetHeadings(rot,rot,rot,rot);
  }
  public void ForcePark()
  {
    m_kinematics.resetHeadings( Rotation2d.fromDegrees(-45),   //FL      
                                Rotation2d.fromDegrees(45),   //FR
                                Rotation2d.fromDegrees(45),   //BL
                                Rotation2d.fromDegrees(-45)    //br
                              );
  }
  


}
