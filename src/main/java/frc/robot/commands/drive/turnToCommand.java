package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

/** A command that will turn the robot to the specified angle. */
public class turnToCommand extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public turnToCommand(double targetAngleDegrees, DriveSubsystem drive) {
    super(
        new PIDController(0.2, 0, 0),
        //(Math.abs(Rotation2d.fromDegrees(targetAngleDegrees).minus(drive.getRotation()).getDegrees()) < 100 ) ? new PIDController(0, 0, 0) : new PIDController(0.1, 0, 0),
        drive::getHeading,
        targetAngleDegrees,
        output -> drive.drive(0, 0, -2 * output, true),
        drive);

    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(1.5, 10);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
