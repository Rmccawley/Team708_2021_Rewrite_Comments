package frc.team708.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team708.robot.subsystems.Spinner;

public class StopIntakeOnceCommand extends CommandBase {

    private final Spinner m_spinner;

    public StopIntakeOnceCommand(Spinner spinner) {
        m_spinner = spinner;
        addRequirements(m_spinner);
    }

    @Override
    public void initialize() {
       m_spinner.StopMotorIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
