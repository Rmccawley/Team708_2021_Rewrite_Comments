package frc.team708.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team708.robot.subsystems.Hopper;
import frc.team708.robot.subsystems.Shooter;
import frc.team708.robot.subsystems.Spinner;

public class StopAllCommand extends CommandBase {

    private final Shooter m_shooter;
    private final Spinner m_spinner;
    private final Hopper m_hopper;

    public StopAllCommand(Shooter shooter, Spinner spinner, Hopper hopper) {
        m_shooter = shooter;
        m_spinner = spinner;
        m_hopper = hopper;
        addRequirements(m_shooter, m_hopper, m_spinner);
    }

    @Override
    public void execute() {

        m_shooter.stopShooter();
        m_shooter.feederOff();
        m_hopper.stopMotor();
        m_spinner.StopMotorIntake();

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
