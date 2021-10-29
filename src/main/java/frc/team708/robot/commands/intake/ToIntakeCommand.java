package frc.team708.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team708.robot.subsystems.Hopper;
import frc.team708.robot.subsystems.Shooter;
import frc.team708.robot.subsystems.Spinner;

public class ToIntakeCommand extends CommandBase {

    private final Spinner m_spinner;
    private final Hopper m_hopper;
    private final Shooter m_shooter;

    public ToIntakeCommand(Spinner spinner, Hopper hopper, Shooter shooter) {
        m_spinner = spinner;
        m_hopper = hopper;
        m_shooter = shooter;
        addRequirements(m_spinner, m_hopper, m_shooter);
    }

    @Override
    public void initialize() {
        m_spinner.toIntake();
        m_hopper.moveMotorCounterClockwise();
        m_shooter.feederPreLoad();
        m_shooter.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}