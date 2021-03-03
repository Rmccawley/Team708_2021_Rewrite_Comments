// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

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
    public static final int kFrontLeftDriveMotorPort = 13;
    public static final int kRearLeftDriveMotorPort = 12;
    public static final int kFrontRightDriveMotorPort = 14;
    public static final int kRearRightDriveMotorPort = 11;

    public static final int kFrontLeftTurningMotorPort = 17;
    public static final int kRearLeftTurningMotorPort = 16;
    public static final int kFrontRightTurningMotorPort = 18;
    public static final int kRearRightTurningMotorPort = 15;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kRearRightDriveEncoderReversed = true;

    //offset in degrees
    public static final double kRearLeftOffset = 34.2;
    public static final double kRearRightOffset = 93.2;
    public static final double kFrontLeftOffset = 68.2;
    public static final double kFrontRightOffset = 136.4;

    public static final double kTrackWidth = 0.444;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.444;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    //public static final double ksVolts = 1;
    //public static final double kvVoltSecondsPerMeter = 0.8;
    //public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 6;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 10 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 8 * Math.PI;

    public static final int kEncoderCPR = 42;
    public static final double kEncoderRatio = .119;
    public static final double kWheelDiameterMeters = 0.1;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        ((kEncoderCPR * kEncoderRatio) / Math.PI * (kWheelDiameterMeters) / 3.056814908981323);
        //(kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) 4096;

    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = 0.1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 6;
    public static final double kMaxAccelerationMetersPerSecondSquared = 6;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2*Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2*Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
