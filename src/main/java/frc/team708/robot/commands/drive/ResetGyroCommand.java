package frc.team708.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team708.robot.subsystems.DriveSubsystem;

public class ResetGyroCommand extends CommandBase {

    private final DriveSubsystem m_DriveSubsystem;

    public ResetGyroCommand(DriveSubsystem subsystem) {
        m_DriveSubsystem = subsystem;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        m_DriveSubsystem.zeroHeading();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
