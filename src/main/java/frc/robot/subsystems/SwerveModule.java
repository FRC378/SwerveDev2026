// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;


public class SwerveModule extends SubsystemBase {

  //  Swerve Modules is a MK4i L2 with Billet wheels
  //  www.swervedrivespecialties.com/products/mk4i-swerve-module
  //
  // MK4i has 150/7:1 turn ratio
  //SparkMax reports back in rotations
  //  (360 degrees * rotations) / ( 150/7:1 )   results in turn angle in degrees from SparkMax Rotations
  final private double TURN_ENCODER_FACTOR =  ( (double)(360.0 / (150.0 / 7.0) ) );     //DEGREES per rotation

  //MK4i L2 has 6.75 reduction
  //  Neo Freespeed 5820 RPM
  //  Tire Diameter = 4inches, perimeter = 4PI  (2*pi*r)
  //  Therefore,Max free speed = 5820 /60   / 6.75   * 4PI / 12 = 15 ft/sec
  // SparkMax reports back in rotations
  //   Diameter * pi / 6.75   results in drive distance in inches from SparkMax Rotations 
  //  ** Due to tire wear, wheel diameter was measured at 3 7/8
  final private double DRIVE_ENCODER_FACTOR  = ( (double)( (3.875) * Math.PI)/ (6.75) );    //INCHES per rotation

  // SparkMax reports back in rotations per minute,  12 inches per foot,  60 seconds in a minute
  //  velocity = DRIVE_ENCODER_FACTOR / ( ft_per_inch * seconds_per_minute )
  final private double DRIVE_VELOCITY_FACTOR = ( (double)( DRIVE_ENCODER_FACTOR / (60.0 * 12.0) ) );  // ft/sec
  final private double DRIVE_VELOCITY_MAX    = ( (double)15.0); //ft/sec




  private String m_moduleID;
  private double m_absEncOffset;

  private SparkMax m_driveMotor;
  private RelativeEncoder m_driveEncoder;
  private SparkClosedLoopController  m_drivePID;

  private SparkMax m_turnMotor;
  private RelativeEncoder m_turnEncoder;
  private SparkClosedLoopController  m_turnPID;

  private AnalogEncoder m_analogEncoder;





  public SwerveModule( int driveCanID, int turnCanID, int encoderID, double absEncOffset, String moduleID ) {

    m_moduleID     = moduleID;
    m_absEncOffset = absEncOffset;

    System.out.println("Starting " + m_moduleID + " SwerveModule");


    //Instanciate the Motors
    m_driveMotor   = new SparkMax(driveCanID, MotorType.kBrushless);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_drivePID     = m_driveMotor.getClosedLoopController();

    m_turnMotor    = new SparkMax(turnCanID, MotorType.kBrushless);
    m_turnEncoder  = m_turnMotor.getEncoder();
    m_turnPID      = m_turnMotor.getClosedLoopController();

    m_analogEncoder = new AnalogEncoder(encoderID);


    //Configurators
    SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
    SparkMaxConfig turnMotorConfig  = new SparkMaxConfig();

    //-- Drive Motor Configuration --
    driveMotorConfig
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake)
        .inverted(false)  
        .closedLoopRampRate(0.3);

    driveMotorConfig.encoder
        .positionConversionFactor( DRIVE_ENCODER_FACTOR  )
        .velocityConversionFactor( DRIVE_VELOCITY_FACTOR );    

    driveMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.05, 0, 0)
        .velocityFF( 0.075 )          // 0.7 pwr = 9.7ft/sec  = .075-ish
        .outputRange(-.95, 0.95);



    //-- Turn Motor Configuration --
    turnMotorConfig
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake)
        .inverted(true) //Positive = CCW Rotation
        .closedLoopRampRate(0.1); 

    turnMotorConfig.encoder
        .positionConversionFactor( TURN_ENCODER_FACTOR )
        .velocityConversionFactor( TURN_ENCODER_FACTOR / 60.0); 

    turnMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid( 0.015, 0, 0)
        .outputRange(-0.5, 0.5)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0,360);



    //Apply configurations
    m_driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turnMotor.configure(turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    m_analogEncoder.setInverted(false);


  }

  @Override
  public void periodic() {

    //Turn Encoder
    SmartDashboard.putNumber(m_moduleID + "-TurnEnc",   GetTurnEncoderPosition() ); 
    SmartDashboard.putNumber(m_moduleID + "-TurnAbs",   GetTurnEncoderAbsolutePosition() ); 

    //Drive
    SmartDashboard.putNumber(m_moduleID + "-DrvVel",    GetDriveVelocity()  ); 
    SmartDashboard.putNumber(m_moduleID + "-DrvEnc",    GetDriveEncoderPosition() ); 
    
    //Drive Motor Temperature
    SmartDashboard.putNumber(m_moduleID + "-DrvTemp",   GetDriveTemp() ); 

  }


  //Manual control (Testing)
  public void SetDriveMotorPower(double power ) {
    m_driveMotor.set(power);
  }
  public void SetTurnMotorPower(double power ) {
    m_turnMotor.set(power);
  }

  private double BoundDegrees ( double angle ) {
    double fmodAngle = angle % 360.0;
    return fmodAngle<0?(fmodAngle+360.0):fmodAngle;
  }


  //---  Swerve Modue State Control -----
  public void setDesiredState(SwerveModuleState desiredState) {
    
    var currTurnAngle = new Rotation2d( Math.toRadians( GetTurnEncoderPosition() ) );

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(currTurnAngle);

    // Scale speed by cosine of angle error.
    desiredState.cosineScale(currTurnAngle);

    SetTurnAngle( desiredState.angle.getDegrees() );
    SetDriveVelocity( desiredState.speedMetersPerSecond );  //Velocity in FEET PER SECOND.
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        GetDriveEncoderPosition(), 
        new Rotation2d( Math.toRadians(GetTurnEncoderPosition()) )  );
  }





  //---  ABSOLUTE TURN ENCODER -----
  public double GetTurnEncoderAbsolutePosition( ) {
    // Offset Corrected position 
    return BoundDegrees( (360.0 * m_analogEncoder.get()) - m_absEncOffset  );
    // Encoder.get returns range [0,1], multiply by 360 for degrees
  }


  //---  TURN MOTOR -----
  public void ResetTurnEncoder()
  {
    m_turnEncoder.setPosition( GetTurnEncoderAbsolutePosition() );
  }
  public double GetTurnEncoderPosition()
  {
    return BoundDegrees( m_turnEncoder.getPosition() );
    //return m_turnEncoder.GetPosition(); //Return +/- infinite degress
  }

  //Set Turn Angle
  // Angle of swerve wheel, in degrees
  //Uses SparkMax internal PID
  void SetTurnAngle( double angle )
  {
    m_turnPID.setReference( angle, ControlType.kPosition );
  }


  //---  DRIVE MOTOR -----

  void ResetDriveEncoder() {
      m_driveEncoder.setPosition(0);
      //m_maxVelocity = 0;
  }
  double GetDriveEncoderPosition()
  {
      return m_driveEncoder.getPosition();
  }
  //Set Drive Velocity
  //  Velocity of wheel, in ft/sec
  // Uses SparkMax internal PID
  void SetDriveVelocity( double speed )
  {
    m_drivePID.setReference(speed, ControlType.kVelocity );
  }
  double GetDriveVelocity( )
  {
    return m_driveEncoder.getVelocity();
  }
  double GetDriveTemp()
  {
    return m_driveMotor.getMotorTemperature();
  }








}
