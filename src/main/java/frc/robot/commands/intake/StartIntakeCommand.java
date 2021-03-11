package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner;

public class StartIntakeCommand extends CommandBase {

    private final Spinner m_spinner;

    public StartIntakeCommand(Spinner spinner) {
        m_spinner = spinner;
        addRequirements(m_spinner);
    }

    @Override
    public void initialize() {
       m_spinner.moveIntakeForward();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
