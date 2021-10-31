package frc.team708.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import frc.team708.robot.subsystems.Shooter;
import frc.team708.robot.subsystems.Spinner;

public class ShootShortCommand extends CommandBase {

    private final Shooter m_shooter;
    private final Spinner m_spinner;

    public ShootShortCommand(Shooter shooter, Spinner spinner) {
        m_shooter = shooter;
        m_spinner = spinner;
        addRequirements(m_shooter);
        addRequirements(m_spinner);
    }

    @Override
    public void execute() {
        m_shooter.shootShort();
        m_spinner.camSolenoid.set(DoubleSolenoid.Value.kForward); // I
        m_spinner.pivotSolenoid.set(DoubleSolenoid.Value.kReverse); // O
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
