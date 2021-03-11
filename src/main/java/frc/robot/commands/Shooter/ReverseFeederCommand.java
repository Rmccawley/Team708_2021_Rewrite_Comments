package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ReverseFeederCommand extends CommandBase {

    private final Shooter m_shooter;

    public ReverseFeederCommand(Shooter shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.feederUnload();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
