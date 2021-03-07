// package frc.robot.commands;

// import edu.wpi.first.wpilibj.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.DriveSubsystem;

// public class turnToCommand extends CommandBase {

//     private final DriveSubsystem m_DriveSubsystem;
//     private final double angle;
//     private double startingAngle;
//     private final PIDController m_PIDController = new PIDController(10, 0, 0);

//     public turnToCommand(DriveSubsystem subsystem, double angle) {
        
//         m_DriveSubsystem = subsystem;
//         this.angle = angle;
//         addRequirements(m_DriveSubsystem);
//     }

//     @Override
//     public void initialize() {
//         startingAngle = m_DriveSubsystem.getHeading();
//         m_PIDController.setTolerance(2);
//         m_PIDController.enableContinuousInput(0, 360);

//     }

//     @Override
//     public void execute() {
        
//         m_DriveSubsystem.drive(0, 0, -m_PIDController.calculate(m_DriveSubsystem.getHeading(), angle), true);
        
//         // if (((angle - startingAngle) > 181) || ((-179 < (angle - startingAngle)) && (angle - startingAngle) < -1)) {
//         //     m_DriveSubsystem.drive(0, 0, 10, true);
//         // } else {
//         //     m_DriveSubsystem.drive(0, 0, -10, true);
//         // }
//     }

//     // Make this return true when this Command no longer needs to run execute()
//     public boolean isFinished() {
//         return m_PIDController.atSetpoint();
//         }

//     public void end() {
//         m_DriveSubsystem.drive(0, 0, 0, true);
//     }
// }

package frc.robot.commands;

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
        (Math.abs(Rotation2d.fromDegrees(targetAngleDegrees).minus(drive.getRotation()).getDegrees()) < 100 ) ? new PIDController(0, 0, 0) : new PIDController(0.1, 0, 0),
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
