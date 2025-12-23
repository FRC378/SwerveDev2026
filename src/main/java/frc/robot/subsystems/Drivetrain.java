// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private SwerveModule m_frontLeft;


  public Drivetrain() {
    System.out.println("Drivetrain Init");

    m_frontLeft = new SwerveModule( 32, 31, 0,0.0,  "FL");



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void drive(double xValue, double yValue, double rValue  )
  {

    m_frontLeft.setDriveMotorPower(xValue);
    m_frontLeft.setTurnMotorPower(rValue);

  }







}
