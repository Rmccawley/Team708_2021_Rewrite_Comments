package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner;

public class StopIntakeCommand extends CommandBase {

    private final Spinner m_spinner;

    public StopIntakeCommand(Spinner spinner) {
        m_spinner = spinner;
        addRequirements(m_spinner);
    }

    @Override
    public void initialize() {
       m_spinner.StopMotorIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
