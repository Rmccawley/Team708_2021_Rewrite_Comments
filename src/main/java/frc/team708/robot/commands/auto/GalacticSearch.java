package frc.team708.robot.commands.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.team708.robot.Constants.AutoConstants;
import frc.team708.robot.Constants.DriveConstants;
import frc.team708.robot.commands.intake.StartIntakeCommand;
import frc.team708.robot.commands.intake.StopIntakeCommand;
import frc.team708.robot.subsystems.DriveSubsystem;
import frc.team708.robot.subsystems.Spinner;

public class GalacticSearch extends SequentialCommandGroup {



    public GalacticSearch(DriveSubsystem dSubsystem, Spinner dSpinner, Trajectory trajectory) {
       
        var thetaController = new ProfiledPIDController(0.25, 0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, dSubsystem::getPose,
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(10, 0, 0), new PIDController(10, 0, 0), thetaController,
                dSubsystem::setModuleStates, dSubsystem);

        addCommands(
                new ResetDriveCommand(trajectory, dSubsystem),
                new StartIntakeCommand(dSpinner),
                swerveControllerCommand.andThen(() -> dSubsystem.drive(0, 0, 0, false)),
                new StopIntakeCommand(dSpinner));

    }

}
