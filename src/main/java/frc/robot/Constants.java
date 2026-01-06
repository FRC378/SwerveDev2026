// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  //** CAN IDs **
  public static final int    PIGEON_CAN_ID    = 10;

  


  //********** DRIVETRAIN CONSTANTS **********

  public static final double DRIVEBASE_WIDTH  =  Units.inchesToMeters(21.2);       //inches, Left to Right
  public static final double DRIVEBASE_LENGTH =  Units.inchesToMeters(21.5);       //inches, Front to Rear

  //-------------------------------------------
  public static final int    FRONTLEFT_DRIVE_CAN_ID   =  32;
  public static final int    FRONTLEFT_TURN_CAN_ID    =  31;
  public static final int    FRONTLEFT_ENCODER_ID     =  2; 
  public static final double FRONTLEFT_ENCODER_OFFSET = (219.8);
  //-------------------------------------------
  public static final int    FRONTRIGHT_DRIVE_CAN_ID   = 33;
  public static final int    FRONTRIGHT_TURN_CAN_ID    = 34;
  public static final int    FRONTRIGHT_ENCODER_ID     = 0;
  public static final double FRONTRIGHT_ENCODER_OFFSET = (27.7);
  //-------------------------------------------
  public static final int    BACKLEFT_DRIVE_CAN_ID     = 24;
  public static final int    BACKLEFT_TURN_CAN_ID      = 23;
  public static final int    BACKLEFT_ENCODER_ID       = 3;
  public static final double BACKLEFT_ENCODER_OFFSET   = (317.3);
  //-------------------------------------------
  public static final int    BACKRIGHT_DRIVE_CAN_ID    = 21;
  public static final int    BACKRIGHT_TURN_CAN_ID     = 22;
  public static final int    BACKRIGHT_ENCODER_ID      = 1;
  public static final double BACKRIGHT_ENCODER_OFFSET  = (112.4);
  //-------------------------------------------



}
