// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;


public class CmdPrintText extends InstantCommand {

  private String m_text;

  public CmdPrintText( String text ) {

    m_text = text;  

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println( m_text );
  }
}
