// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CmdDriveClearAll;
import frc.robot.commands.CmdDriveForcePark;
import frc.robot.commands.CmdDriveForceTurnAngle;
import frc.robot.commands.CmdDriveTypeToggle;
import frc.robot.commands.CmdDriveWithGamepad;
import frc.robot.commands.CmdDriveZeroGyro;
import frc.robot.commands.CmdPrintText;
import frc.robot.commands.GrpTest1;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {


  //****************Controllers*******************
  public static final CommandXboxController m_driver = new CommandXboxController(0);


  //****************Subsystems*******************
  //Make public & static to access "globally"
  public static PowerDistribution m_pdp  = new PowerDistribution(2, PowerDistribution.ModuleType.kCTRE);
  public static Drivetrain  m_drivetrain = new Drivetrain();





  public RobotContainer() {

    //****************Default Commands**************
    m_drivetrain.setDefaultCommand( new CmdDriveWithGamepad() );


    //****************Smartdashboard Buttons**************

    SmartDashboard.putData( "CmdDriveClearAll",  new CmdDriveClearAll());
    SmartDashboard.putData( "DriveToggle",  new CmdDriveTypeToggle());


    SmartDashboard.putData( "0",  new CmdDriveForceTurnAngle(0.0));
    SmartDashboard.putData( "90", new CmdDriveForceTurnAngle(90.0));
    SmartDashboard.putData( "45", new CmdDriveForceTurnAngle(45.0));

    SmartDashboard.putData( "GrpTest1", new GrpTest1());

    


    configureBindings();
  }

  private void configureBindings() {

    //Driver Buttons
    m_driver.a().onTrue( new CmdPrintText("A Button ON"));
    m_driver.b().onTrue( new CmdPrintText("B Button ON"));

    m_driver.x().onTrue( new CmdDriveForcePark() );
    m_driver.y().onTrue( new CmdDriveForceTurnAngle( 0.0) );

    m_driver.start().onTrue( new CmdDriveZeroGyro() );  
    

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
