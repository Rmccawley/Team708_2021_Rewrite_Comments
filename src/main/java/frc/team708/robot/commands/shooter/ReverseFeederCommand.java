package frc.team708.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team708.robot.subsystems.Shooter;

public class ReverseFeederCommand extends CommandBase {

    private final Shooter m_shooter;

    public ReverseFeederCommand(Shooter shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        m_shooter.feederUnload();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
