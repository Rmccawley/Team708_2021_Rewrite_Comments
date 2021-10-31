package frc.team708.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import frc.team708.robot.Constants.AutoConstants;
import frc.team708.robot.Constants.DriveConstants;
import frc.team708.robot.commands.auto.InvertedSwerveCommand;
import frc.team708.robot.commands.auto.SwerveCommand;
import frc.team708.robot.commands.drive.CancelDriveCommand;
import frc.team708.robot.commands.hopper.RotateHopperCommand;
import frc.team708.robot.commands.hopper.StopHopperCommand;
import frc.team708.robot.commands.intake.InitIntakeCommand;
import frc.team708.robot.commands.intake.StartIntakeCommand;
import frc.team708.robot.commands.intake.StopIntakeCommand;
import frc.team708.robot.commands.shooter.ShooterPreloadCommand;
import frc.team708.robot.commands.shooter.StopShooterCommand;
import frc.team708.robot.commands.turret.UpdateAngleCommand;
import frc.team708.robot.subsystems.DriveSubsystem;
import frc.team708.robot.subsystems.Hopper;
import frc.team708.robot.subsystems.Spinner;
import frc.team708.robot.subsystems.Shooter;
import frc.team708.robot.subsystems.Turret;
import frc.team708.robot.subsystems.VisionProcessor;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class RobotContainer {
  // The robot's subsystems
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private static final Hopper m_hopper = new Hopper();
  private static final Spinner m_spinner = new Spinner();
  public static final Shooter m_shooter = new Shooter();
  private static final Turret m_turret = new Turret();
  public static final VisionProcessor m_visionProcessor = new VisionProcessor();
  public static final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    CameraServer.getInstance().startAutomaticCapture();

    OI.configureButtonBindings(m_robotDrive, m_hopper, m_spinner, m_shooter, m_turret, m_visionProcessor);

    // Configure default commands
    m_shooter.setDefaultCommand(new ShooterPreloadCommand(m_shooter));
    // m_shooter.setDefaultCommand(new StopShooterCommand(m_shooter));
    m_turret.setDefaultCommand(new UpdateAngleCommand(m_turret));
    // m_spinner.setDefaultCommand(new (m_spinner));
    m_spinner.setDefaultCommand(new InitIntakeCommand(m_spinner));
    m_hopper.setDefaultCommand(new StopHopperCommand(m_hopper));
    m_robotDrive.setDefaultCommand(new RunCommand(

        () -> m_robotDrive.drive(-m_robotDrive.getSpeedCoeff() * OI.getDriverY(GenericHID.Hand.kLeft),
            -m_robotDrive.getSpeedCoeff() * OI.getDriverX(GenericHID.Hand.kLeft),
            -m_robotDrive.getSpeedCoeff() * 25 / 10 * OI.getDriverX(GenericHID.Hand.kRight), true),
        m_robotDrive));

    m_chooser.setDefaultOption("nothing", new CancelDriveCommand(m_robotDrive));
    // m_chooser.addOption("0 -> 1 in x",
    //     new SwerveCommand(m_robotDrive, createTrejectory(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    //         List.of(new Translation2d(0.5, 0)), new Pose2d(1, 0, Rotation2d.fromDegrees(0)))));
    // m_chooser.addOption("0 -> 1 in y",
    //     new SwerveCommand(m_robotDrive, createTrejectory(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    //         List.of(new Translation2d(0, 0.5)), new Pose2d(0, 1, Rotation2d.fromDegrees(0)))));
    // m_chooser.addOption("0 -> 1 in x invert",
    //     new InvertedSwerveCommand(m_robotDrive, m_spinner, createTrejectory(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    //         List.of(new Translation2d(0.5, 0)), new Pose2d(1, 0, Rotation2d.fromDegrees(0)))));
    m_chooser.addOption("driveoffline",
        new InvertedSwerveCommand(m_robotDrive, m_spinner, createTrejectory(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(0, 0.5)), new Pose2d(0, 1, Rotation2d.fromDegrees(0)))));
    m_chooser.addOption("3 Ball Auto",
        new ThreeBallAuto(m_robotDrive, m_spinner, createTrejectory(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(0, 0.5)), new Pose2d(0, 3, Rotation2d.fromDegrees(0)))));
    SmartDashboard.putData("Auto Chooser", m_chooser);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Finds the trajectory on the rio
   *
   * @param json the mane of the json
   * @return the trajectory
   * 
   */
  public Trajectory findTrajectory(String json) {

    String trajectoryJSON = "paths/" + json + ".wpilib.json";
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    return trajectory;
  }

  public Trajectory createTrejectory(Pose2d start, List<Translation2d> waypoints, Pose2d stop) {

    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    return TrajectoryGenerator.generateTrajectory(start, waypoints, stop, config);
  }

  public void sendToDashboard() {
    m_robotDrive.sendToDashboard();
    m_shooter.sendToDashboard();
    m_spinner.sendToDashboard();
    SmartDashboard.putData(m_shooter);
    // SmartDashboard.putNumber("LX", 25 * OI.getDriverX(GenericHID.Hand.kLeft));
    // SmartDashboard.putData("reset Gyro", new resetGyroCommand(m_robotDrive));
    // SmartDashboard.putData("turn to 0", new turnToCommand(0, m_robotDrive));
    // SmartDashboard.putData("turn to 90", new turnToCommand(90, m_robotDrive));
    // SmartDashboard.putData("turn to 180", new turnToCommand(180, m_robotDrive));
    // SmartDashboard.putData("turn to 270", new turnToCommand(270, m_robotDrive));
    // SmartDashboard.putData("Cancel", new cancelDriveCommand(m_robotDrive));
    SmartDashboard.putData(m_robotDrive);
  }
}
