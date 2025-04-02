// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
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
    public static final double maxSpeedMetersPerSec = 3.0; // FIXME
    public static final int kEncoderCountsPerRevolution = 2048 * 4; // one rotation is 8192 ticks of the hardware
                                                                    // encoder
    public static final double kDrivePositionConversionFactor = DriveConstants.wheelCircumferenceMeters
        / kEncoderCountsPerRevolution;
    public static final double kDriveVelocityConversionFactor = DriveConstants.wheelCircumferenceMeters
        / kEncoderCountsPerRevolution; // RPM (per minute)

    public static final double reakKp = 0.1; // was: 0.0001
    public static final double realKd = 0.0;
    public static final double realKi = 0.0;

  }

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int coralToReefAutomated = 2;
    public static final int coralToReef = 5;
    public static final int intakeGamePiece = 3;
    public static final int armUp = 6;
    public static final int armDown = 4;
    // public static final int armDownDebouncer = 7;
    //public static final int elevatorToL1 = 9;
    public static final int elevatorToL2 = 7;
    public static final int elevatorToL3 = 8;
    public static final int elevatorToTop = 9;
    public static final int scoreL1Coral = 1;


  }

  public static final class ArmConstants {
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

    public static final double rollerGamePieceInSpeed = -0.8;
    public static final double rollerCoralOutSpeed = 0.5;
  }

  public static final class ElevatorConstants {
    public static enum ElevatorPosition {
      BOTTOM(0.0),
      INTAKE_PREP(0.55),
      INTAKE(0.355),
      ALGAE_L2(0.884),
      ALGAE_L3(1.234),

      L1(0.0),
      L2(0.0),
      L3(0.367),
      L4(1.27),
      TOP(0.78);//Intake 0.386

      public final double value;

      private ElevatorPosition(double value) {
        this.value = value;
      }
    }

    public static final double MOTION_LIMIT = 0.3;

    public static final double SCORING_MOVEMENT = -0.25;

    public static final int MOTOR_ID = 7;
    public static final boolean MOTOR_INVERTED = true;

    public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNEO(1);
    public static final double GEARING = 5.0;
    public static final double MASS_KG = Units.lbsToKilograms(20);
    public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.32) / 2.0; // TODO
    public static final double DRUM_CIRCUMFERENCE = 2.0 * Math.PI * DRUM_RADIUS_METERS;
    public static final double ENCODER_ROTATIONS_TO_METERS = DRUM_CIRCUMFERENCE / GEARING;

    public static final double MIN_HEIGHT_METERS = 0.005; // TODO
    public static final double MAX_HEIGHT_METERS = 1.57; // TODO

    public static final int CURRENT_LIMIT = 60;

    public static final double kP = 50; // TODO
    public static final double kI = 0; // TODO
    public static final double kD = 5; // TODO
    public static final double kS = 0.095388; // TODO
    public static final double kG = 0.54402; // TODO
    public static final double kV = 7.43; // TODO
    public static final double kA = 1.0; // TODO
    public static final double TOLERANCE = 0.02;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 1.3; // TODO
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3; // TODO
    public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
        MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        
  }

  //Coral Arm
  public static final class CoralArm {
    public static enum ArmPosition {
      BOTTOM(-Math.PI / 2.0 + Units.degreesToRadians(5)), 
      HORIZONTAL(0),
      L1(0),
      L2(-Units.degreesToRadians(55)), // reef angle
      L3(-Units.degreesToRadians(55)),
      L4(1.033),
      TOP(Math.PI / 2.0);
      //TOP(0);//-1.234

      public final double value;

      private ArmPosition(double value) {
        this.value = value;
      }
    }

    public static final double MOTION_LIMIT = -0.7;
    public static final double SCORING_MOVEMENT = -0.8;

    public static final int MOTOR_ID = 9;
    public static final boolean MOTOR_INVERTED = true;

    public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNEO(1);
    public static final double GEARING = 40.0; // TODO
    public static final double MASS_KG = Units.lbsToKilograms(10); // TODO
    public static final double COM_DISTANCE_METERS = Units.inchesToMeters(6); // TODO
    public static final double MOI = SingleJointedArmSim.estimateMOI(COM_DISTANCE_METERS, MASS_KG);
    public static final double ENCODER_ROTATIONS_TO_METERS = 2 * Math.PI / GEARING;

    public static final double MIN_ANGLE_RADIANS = -Math.PI / 2.0;
    public static final double MAX_ANGLE_RADIANS = Math.PI / 2.0;

    public static final int CURRENT_LIMIT = 50;

    public static final double kP = 5; // TODO
    public static final double kI = 0; // TODO
    public static final double kD = 0; // TODO
    public static final double  kS= 0.017964; // TODO
    public static final double kG = 0.321192; // TODO
    //public static final double kV = 0.876084;// TODO
    public static final double kV = 0.5;
    //public static final double kA = 0.206676;// TODO
    public static final double kA = 0.1;
    public static final double TOLERANCE = 0.1;//0.02

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 2; // TODO
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1; // TODO
    public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
        MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
  }

}
