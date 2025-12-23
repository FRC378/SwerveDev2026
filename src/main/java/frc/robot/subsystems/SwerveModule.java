// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SwerveModule extends SubsystemBase {

  private String m_moduleID;

  private SparkMax m_driveMotor;
  private SparkMax m_turnMotor;





  public SwerveModule( int driveCanID, int turnCanID, int encoderID, double absEncOffset, String moduleID ) {

    m_moduleID = moduleID;

    System.out.println("Starting " + m_moduleID + " SwerveModule");


    //Instanciate the Motors
    m_driveMotor = new SparkMax(driveCanID, MotorType.kBrushless);
    m_turnMotor  = new SparkMax(turnCanID, MotorType.kBrushless);

    //Configurators
    SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
    SparkMaxConfig turnMotorConfig  = new SparkMaxConfig();

   //-- Drive Motor Configuration --
   driveMotorConfig
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake)
        .inverted(false)  
        .openLoopRampRate(0.3);       //ToDo:   Make ClosedLoop


    //-- Turn Motor Configuration --
    turnMotorConfig
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake)
        .inverted(true) //Positive = CCW Rotation
        .openLoopRampRate(0.1);       //ToDo:   Make ClosedLoop


    //Apply configurations
    m_driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turnMotor.configure(turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  //Manual control (Testing)
  public void setDriveMotorPower(double power ) {
    m_driveMotor.set(power);
  }
  public void setTurnMotorPower(double power ) {
    m_turnMotor.set(power);
  }



  

}
