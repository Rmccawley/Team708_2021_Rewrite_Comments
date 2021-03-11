package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner;

public class ReverseIntakeCommand extends CommandBase {

    private final Spinner m_spinner;

    public ReverseIntakeCommand(Spinner spinner) {
        m_spinner = spinner;
        addRequirements(m_spinner);
    }

    @Override
    public void initialize() {
       m_spinner.reverseIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
