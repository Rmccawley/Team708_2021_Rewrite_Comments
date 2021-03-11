package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterPreloadCommand extends CommandBase {
    
    private final Shooter m_shooter;

    public ShooterPreloadCommand(Shooter shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.feederPreLoad();
        m_shooter.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
