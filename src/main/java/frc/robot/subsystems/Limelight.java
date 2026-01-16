// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  private String m_llname;  //The "Name" of the limemight camera


  /** Creates a new Limelight. */
  public Limelight( String llName ) {

    System.out.println("Starting " + llName + " Limelight");

    m_llname = llName;
  }

  @Override
  public void periodic() {
    RunLimelight();
  }


  public int GetTargetId()
  {
     return (int)NetworkTableInstance.getDefault().getTable(m_llname).getEntry("tid").getInteger(0); 
  } 

  public int IsTargetValid()
  {
     return (int)NetworkTableInstance.getDefault().getTable(m_llname).getEntry("tv").getInteger(0); 
  } 


  private void RunLimelight() {

    //SmartDashboard.putNumber(m_llname + "-TID",   GetTargetId() ); 
    //SmartDashboard.putBoolean(m_llname + "-Valid",IsTargetValid() );

  }

}
