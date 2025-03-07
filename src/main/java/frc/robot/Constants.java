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
  public static final class DriveConstants {
    public static final int LEFT_LEADER_ID = 3;
    public static final int LEFT_FOLLOWER_ID = 4;
    public static final int RIGHT_LEADER_ID = 1;
    public static final int RIGHT_FOLLOWER_ID = 2;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

    public static final double wheelRadiusInches = 3.0;
    public static final double wheelDiameterInches = 2 * wheelRadiusInches;
    public static final double wheelCircumferenceInches = wheelDiameterInches * Math.PI;

    // chassis configuration in meters
    public static final double wheelRadiusMeters = Units.inchesToMeters(wheelRadiusInches);
    public static final double wheelDiameterMeters = Units.inchesToMeters(wheelDiameterInches);
    public static final double wheelCircumferenceMeters = Units.inchesToMeters(wheelCircumferenceInches);

    // speed references meters/sec
    public static final double walkingSpeedMetersPerSec = 1.0;  
    public static final double maxSpeedMetersPerSec = 3.0;  // FIXME
    public static final int kEncoderCountsPerRevolution = 2048 * 4;     // one rotation is 8192 ticks of the hardware encoder
    public static final double kDrivePositionConversionFactor = DriveConstants.wheelCircumferenceMeters / kEncoderCountsPerRevolution;
    public static final double kDriveVelocityConversionFactor = DriveConstants.wheelCircumferenceMeters / kEncoderCountsPerRevolution;     // RPM (per minute)
    
    public static final double reakKp = 0.1; // was: 0.0001
    public static final double realKd = 0.0;
    public static final double realKi = 0.0;
    
  }
  
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int coralToReef = 2;
    public static final int intakeGamePiece = 3;
    public static final int armUp = 6;
    public static final int armDown = 4;
   // public static final int armDownDebouncer = 7;
  }
  public static final class ArmConstants{
    public static final int ARM_MOTOR_ID = 5;
    public static final int ARM_MOTOR_CURRENT_LIMIT = 9;
    public static final double ARM_MOTOR_VOLTAGE_COMP = 10;
    public static final double ARM_SPEED_DOWN = -0.5;
    public static final double ARM_SPEED_UP = 0.5;
    public static final double ARM_HOLD_DOWN = 0.0;
    public static final double ARM_HOLD_UP = -0.0;
  }

  public static final class ArmRollerConstants {
      public static final int ROLLER_MOTOR_ID = 6;
      public static final int ROLLER_MOTOR_CURRENT_LIMIT = 9;
      public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
      public static final double ROLLER_EJECT_VALUE = 0.44;

      public static final double rollerGamePieceInSpeed = -0.5;
      public static final double rollerCoralOutSpeed = 0.5;
  }
}

