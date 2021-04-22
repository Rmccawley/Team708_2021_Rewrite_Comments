package frc.team708.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team708.robot.subsystems.Shooter;

public class StopShooterCommand extends CommandBase{
    
    private final Shooter m_shooter;

    public StopShooterCommand(Shooter shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.stopShooter();
        m_shooter.feederOff();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
