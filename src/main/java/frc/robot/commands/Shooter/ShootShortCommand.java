package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootShortCommand extends CommandBase {

    private final Shooter m_shooter;

    public ShootShortCommand(Shooter shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        m_shooter.shootShort();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
