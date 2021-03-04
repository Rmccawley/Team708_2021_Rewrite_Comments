package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class turnToCommand extends CommandBase {

    private final DriveSubsystem m_DriveSubsystem;
    private final double angle;
    private double startingAngle;

    public turnToCommand(DriveSubsystem subsystem, double angle) {
        m_DriveSubsystem = subsystem;
        this.angle = angle;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        startingAngle = m_DriveSubsystem.getHeading() % 360;

    }

    @Override
    public void execute() {
        if ((startingAngle - angle) > 0) {
            m_DriveSubsystem.drive(0, 0, -10, true);
        } else {
            m_DriveSubsystem.drive(0, 0, 10, true);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        if (Math.abs(startingAngle - angle) < 1.0) {
            return true;
        } else {
            return false;
        }
    }

    public void end() {
        m_DriveSubsystem.drive(0,0,0,true);
    }
}
