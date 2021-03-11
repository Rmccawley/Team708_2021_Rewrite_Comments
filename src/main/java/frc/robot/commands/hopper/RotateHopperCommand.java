package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;

public class RotateHopperCommand extends CommandBase {

    private final Hopper m_hopper;

    public RotateHopperCommand(Hopper hopper) {
        m_hopper = hopper;
        addRequirements(m_hopper);
    }

    @Override
    public void initialize() {
        m_hopper.moveMotorCounterClockwise();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
