package frc.team708.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team708.robot.subsystems.DriveSubsystem;

public class InvertDriveCommand extends CommandBase {

    private final DriveSubsystem m_DriveSubsystem;

    public InvertDriveCommand(DriveSubsystem subsystem) {
        m_DriveSubsystem = subsystem;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        m_DriveSubsystem.invertDrive();;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}