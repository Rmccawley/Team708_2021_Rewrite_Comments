package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootLongCommand extends CommandBase {

    private final Shooter m_shooter;

    public ShootLongCommand(Shooter shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        m_shooter.shootLong();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
