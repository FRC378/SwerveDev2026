// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import pabeles.concurrency.ConcurrencyOps.Reset;

import com.ctre.phoenix6.hardware.Pigeon2;

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
  private SwerveModule m_frontLeft  =  new SwerveModule( Constants.FRONTLEFT_DRIVE_CAN_ID, Constants.FRONTLEFT_TURN_CAN_ID, Constants.FRONTLEFT_ENCODER_ID, Constants.FRONTLEFT_ENCODER_OFFSET,  "FL");;
  private SwerveModule m_frontRight =  new SwerveModule( Constants.FRONTRIGHT_DRIVE_CAN_ID,Constants.FRONTRIGHT_TURN_CAN_ID,Constants.FRONTRIGHT_ENCODER_ID,Constants.FRONTRIGHT_ENCODER_OFFSET, "FR");;
  private SwerveModule m_backLeft   =  new SwerveModule( Constants.BACKLEFT_DRIVE_CAN_ID,  Constants.BACKLEFT_TURN_CAN_ID,  Constants.BACKLEFT_ENCODER_ID,  Constants.BACKLEFT_ENCODER_OFFSET,   "BL");;
  private SwerveModule m_backRight  =  new SwerveModule( Constants.BACKRIGHT_DRIVE_CAN_ID, Constants.BACKRIGHT_TURN_CAN_ID, Constants.BACKRIGHT_ENCODER_ID, Constants.BACKRIGHT_ENCODER_OFFSET,  "BR");;


  //Pigenon2 Gyro
  private final Pigeon2 m_gyro = new Pigeon2( Constants.PIGEON_CAN_ID, "rio");


  //Odometry
  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });



  //Swerve Drive Debug (AvantageScope)
  private final StructArrayPublisher<SwerveModuleState> publisher;

  public Drivetrain() {
    System.out.println("Drivetrain Init");

    m_gyro.reset();    

    ResetTurnEncoders();  //Aligns SparkMax turn encoders to absolute encoders
    ResetDriveEncoders();

    //Publisher init
    publisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Update Odometry
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });


    //Gyro
    SmartDashboard.putBoolean("GyroConnected", IsGyroConnected() );
    SmartDashboard.putNumber("GyroYaw",  GetGyroYaw() );

    //Odometry
    SmartDashboard.putNumber( "odoX", GetOdometryX() ); 
    SmartDashboard.putNumber( "odoY", GetOdometryY() ); 
    SmartDashboard.putNumber( "odoH", GetOdometryHeading() ); 

  }


  public void drive(double xValue, double yValue, double rValue  )
  {

    //Velocity control
    final double xSpeed = xValue * kMaxVelocity;
    final double ySpeed = yValue * kMaxVelocity;
    final double rSpeed = rValue * Math.toRadians(kMaxRotation) * 3.28; //wpilib units fudge factor

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
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    double vel = Math.hypot(xSpeed, ySpeed);
    SmartDashboard.putNumber("xyVelocity", vel);

    publisher.set(new SwerveModuleState[] {
      swerveModuleStates[0],
      swerveModuleStates[1],
      swerveModuleStates[2],
      swerveModuleStates[3]
    });

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
  

  public void ResetDriveEncoders() {
    m_frontLeft.ResetDriveEncoder();
    m_frontRight.ResetDriveEncoder();
    m_backLeft.ResetDriveEncoder();
    m_backRight.ResetDriveEncoder();
  }

  public void ResetTurnEncoders() {
    m_frontLeft.ResetTurnEncoder();
    m_frontRight.ResetTurnEncoder();
    m_backLeft.ResetTurnEncoder();
    m_backRight.ResetTurnEncoder();
  }

  // --- Gyro ---
  // +CCW,  -CW
  public boolean IsGyroConnected()
  {
    return m_gyro.isConnected();
  }
  public double GetGyroYaw()            //yaw: -inf to +inf
  {
    return m_gyro.getYaw().getValueAsDouble(); 
  }
  public void ZeroGyro()
  {
    m_gyro.setYaw( 0 );
    System.out.println("ZeroGyro");
  }


  // --- Odometry ---
  public void ResetOdometry()
  {
    System.out.println("ResetOdometry");

    //Reset to (0,0,0)
    m_odometry.resetPosition(
        new Rotation2d(),
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        },
        new edu.wpi.first.math.geometry.Pose2d()
    );

  }
  public double GetOdometryX()
  {
    return m_odometry.getPoseMeters().getX();
  }
  public double GetOdometryY()
  {
    return m_odometry.getPoseMeters().getY();
  }
  public double GetOdometryHeading()
  {
    return m_odometry.getPoseMeters().getRotation().getDegrees();    //Outputs [-180 to +180]
  }




}
