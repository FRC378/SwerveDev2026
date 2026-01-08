// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class GrpTest1 extends SequentialCommandGroup {
  /** Creates a new GrpCmd1. */
  public GrpTest1() {

    addCommands(

      new CmdPrintText("GrpTest1 Start"),
      new CmdDriveClearAll(),

      new WaitCommand(1.0),

      new CmdDriveTurnToHeading(90.0, 0.5),
      new WaitCommand(1.0),

      new CmdDriveTurnToHeading(0.0, 0.5),
      new WaitCommand(1.0),

      new CmdDriveTurnToHeading(-90.0, 0.5),
      new WaitCommand(1.0),

      new CmdDriveTurnToHeading(0.0, 0.5),
      new WaitCommand(1.0),

      new CmdPrintText("CmdDriveToAbsolutePoint"),
      new CmdDriveToAbsolutePoint( 10, 0,  0, 0.1, false, 0),
      new CmdDriveToAbsolutePoint( 10, 10, 0, 0.1, false, 0),
      new CmdDriveToAbsolutePoint(  0, 10, 0, 0.1, false, 0),
      new CmdDriveToAbsolutePoint(  0, 0,  0, 0.1, true,  0),


      new CmdPrintText("GrpTest1 End")
    );
  }
}
