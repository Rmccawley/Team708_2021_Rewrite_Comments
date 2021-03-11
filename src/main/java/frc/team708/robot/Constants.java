package frc.team708.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public final class Constants {

  public static final class RobotConstants {
    public static final int kRobot = 0; // snowflake = 1, competition = 0
  }

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

    public static final boolean kFrontLeftDriveEncoderReversed = (RobotConstants.kRobot == 0) ? false : true;
    public static final boolean kRearLeftDriveEncoderReversed = (RobotConstants.kRobot == 0) ? false : true;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kRearRightDriveEncoderReversed = true;

    // offset in degrees
    public static final double kRearLeftOffset = (RobotConstants.kRobot == 0) ? -86 : 34.2;
    public static final double kRearRightOffset = (RobotConstants.kRobot == 0) ? -125.5 : 93.2;
    public static final double kFrontLeftOffset = (RobotConstants.kRobot == 0) ? -22.4 : 68.2;
    public static final double kFrontRightOffset = (RobotConstants.kRobot == 0) ? -24.1 : 136.4;

    public static final double kTrackWidth = (RobotConstants.kRobot == 0) ? 0.6731 : 0.444;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = (RobotConstants.kRobot == 0) ? 0.8382 : 0.444;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = true;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The RobotPy Characterization Toolsuite provides a convenient tool for
    // obtaining these
    // values for your robot.
    // public static final double ksVolts = 1;
    // public static final double kvVoltSecondsPerMeter = 0.8;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 25;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 10 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 8 * Math.PI;

    public static final int kEncoderCPR = 42;
    public static final double kEncoderRatio = .119;
    public static final double kWheelDiameterMeters = 0.1;
    public static final double kDriveEncoderDistancePerPulse = ((kEncoderCPR * kEncoderRatio) / Math.PI
        * (kWheelDiameterMeters) / 3.056814908981323);

    public static final double kTurningEncoderDistancePerPulse = (2 * Math.PI) / (double) 4096;

    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = 1 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 1 * Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ControllerConstants {

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double kDriverDeadBandLeftX = 0.2;
    public static final double kDriverDeadBandRightX = 0.2;
    public static final double kDriverDeadBandLeftY = 0.2;
    public static final double kDriverDeadBandRightY = 0.2;
  }

  public static final class HopperConstants {

    public static final int kHopperMotor = 28;
    public static final double kHopperSpeed = 4000;

  }

  public static final class SpinnerConstants {

    public static final int kSpinnerMotor = 41;
    public static final int kLittlePecker = 3;
    public static final double kIntakeSpeed = 0.5;

  }

  public static final class ShooterConstants {

    public static int kShooterShootMotor = 21;
    public static final int kShooterShootMotor2 = 22;
    public static final int kFeederFeedMotor = 25;

    public static final double kShooterWheelSpeedLong = 3900.0;
    public static final double kShooterWheelSpeedShort = 1850.0;

    public static int kHoodSolenoid = 0;

  }

  public static final class TurretConstants {

    public static final int kTurretMotor = 0;

    public static final int kTurretEncoderCount = 32768;
    public static final double kTurretEncoderStartingPos = -2614;

    public static final double kTurretRotationMaxSpeed = 9625;

    public static final double kTurretOnTargetTolerance = 1;
    public static final int kTurretSafeTolerance = 2;

  }

  public static final class VisionProcessorConstants {

    public static final int kVisionLedOn = 0;
    public static final int kVisionLedOff = 1;

  }
}
