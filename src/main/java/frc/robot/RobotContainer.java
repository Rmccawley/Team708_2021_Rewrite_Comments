package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.hopper.StopHopperCommand;
import frc.robot.commands.intake.StartIntakeCommand;
import frc.robot.commands.shooter.ShooterPreloadCommand;
import frc.robot.commands.shooter.StopShooterCommand;
import frc.robot.commands.turret.UpdateAngleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.VisionProcessor;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

public class RobotContainer {
  // The robot's subsystems
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private static final Hopper m_hopper = new Hopper();
  private static final Spinner m_spinner = new Spinner();
  public static final Shooter m_shooter = new Shooter();
  private static final Turret m_turret = new Turret();
  public static final VisionProcessor m_visionProcessor = new VisionProcessor();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    OI.configureButtonBindings(m_robotDrive, m_hopper, m_spinner, m_shooter, m_turret, m_visionProcessor);

    // Configure default commands
    m_shooter.setDefaultCommand(new ShooterPreloadCommand(m_shooter));
    //m_shooter.setDefaultCommand(new StopShooterCommand(m_shooter));
    m_turret.setDefaultCommand(new UpdateAngleCommand(m_turret));
    m_spinner.setDefaultCommand(new StartIntakeCommand(m_spinner));
    m_hopper.setDefaultCommand(new StopHopperCommand(m_hopper));
    m_robotDrive.setDefaultCommand(new RunCommand(

        () -> m_robotDrive.drive(10 * OI.getDriverY(GenericHID.Hand.kLeft), -10 * OI.getDriverX(GenericHID.Hand.kLeft),
            25 * OI.getDriverX(GenericHID.Hand.kRight), true),
        m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    // String trajectoryJSON = "paths/two.wpilib.json";
    // Trajectory trajectory = new Trajectory();
    // try {
    // Path trajectoryPath =
    // Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    // trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    // } catch (IOException ex) {
    // DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON,
    // ex.getStackTrace());
    // }
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(2, 2), new Translation2d(4, 0)),
        new Pose2d(0, 0, new Rotation2d(0)), config);

    var thetaController = new ProfiledPIDController(0.2, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, m_robotDrive::getPose,
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(0.2, 0, 0), new PIDController(0.2, 0, 0), thetaController, m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen();// -> m_robotDrive.drive(0, 0, 0, false));
  }

  public void sendToDashboard() {
    m_robotDrive.sendToDashboard();
    SmartDashboard.putNumber("LX", 25 * OI.getDriverX(GenericHID.Hand.kLeft));
    // SmartDashboard.putData("reset Gyro", new resetGyroCommand(m_robotDrive));
    // SmartDashboard.putData("turn to 0", new turnToCommand(0, m_robotDrive));
    // SmartDashboard.putData("turn to 90", new turnToCommand(90, m_robotDrive));
    // SmartDashboard.putData("turn to 180", new turnToCommand(180, m_robotDrive));
    // SmartDashboard.putData("turn to 270", new turnToCommand(270, m_robotDrive));
    // SmartDashboard.putData("Cancel", new cancelDriveCommand(m_robotDrive));
    // SmartDashboard.putData(m_robotDrive);
  }
}
