package frc.team708.robot.commands.turret;

import frc.team708.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class UpdateAngleCommand extends CommandBase {

    private final Turret m_turret;

    public UpdateAngleCommand(Turret turret) {
        m_turret = turret;
        addRequirements(m_turret);
    }

    @Override
    public void execute() {
        m_turret.updateAngle();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

    
