package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.StopAllCommand;
import frc.robot.commands.Shooter.ReverseFeederCommand;
import frc.robot.commands.Shooter.ShootLongCommand;
import frc.robot.commands.Shooter.ShootShortCommand;
import frc.robot.commands.drive.*;
import frc.robot.commands.hopper.RotateHopperCommand;
import frc.robot.commands.intake.ReverseIntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.VisionProcessor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.util.AxisUp;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {

	// Gamepads
	public final static XboxController driverGamepad = new XboxController(ControllerConstants.kDriverControllerPort); // Driver
	public final static XboxController operatorGamepad = new XboxController(
			ControllerConstants.kOperatorControllerPort);// Operator

	/*
	 * Driver JoystickButton
	 */

	public static final JoystickButton aButtonDriver = new JoystickButton(driverGamepad, Button.kA.value);
	public static final JoystickButton bButtonDriver = new JoystickButton(driverGamepad, Button.kB.value);
	public static final JoystickButton xButtonDriver = new JoystickButton(driverGamepad, Button.kX.value);
	public static final JoystickButton yButtonDriver = new JoystickButton(driverGamepad, Button.kY.value);
	public static final JoystickButton backButtonDriver = new JoystickButton(driverGamepad, Button.kBack.value);
	public static final JoystickButton l3ButtonDriver = new JoystickButton(driverGamepad, Button.kStickLeft.value);

	public static final JoystickButton xButtonOperator = new JoystickButton(operatorGamepad, Button.kX.value);
	public static final JoystickButton leftBumperOperator = new JoystickButton(operatorGamepad,
			Button.kBumperLeft.value);
	public static final JoystickButton aButtonOperator = new JoystickButton(operatorGamepad, Button.kA.value);
	public static final JoystickButton yButtonOperator = new JoystickButton(operatorGamepad, Button.kY.value);
	public static final JoystickButton backButtonOperator = new JoystickButton(operatorGamepad, Button.kBack.value);

	public static final AxisUp leftTriggerOperator = new AxisUp(operatorGamepad, Axis.kLeftTrigger.value);

	public OI() {

	}

	private static double deadBand(double val, double deadband) {
		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
		// if (deadband == 0){
		// return val;
		// }
		// else{
		// if (val <= -deadband){
		// return (double) Math.pow(-2.0, ((-5*val)-5));
		// }
		// else if ((val > -deadband) && (val < deadband)){
		// return (double) 0.0;
		// }
		// else{
		// return (double) Math.pow(2.0, (5*val-5));
		// }
		// }
	}

	public static double getDriverX(Hand hand) {

		if (hand.equals(Hand.kLeft)) {
			return deadBand(driverGamepad.getX(Hand.kLeft), ControllerConstants.kDriverDeadBandLeftX);
		} else {
			return deadBand(driverGamepad.getX(Hand.kRight), ControllerConstants.kDriverDeadBandRightX);
		}
	}

	public static double getDriverY(Hand hand) {
		if (hand.equals(Hand.kLeft)) {
			return deadBand(driverGamepad.getY(Hand.kLeft), ControllerConstants.kDriverDeadBandLeftY);
		} else {
			return deadBand(driverGamepad.getY(Hand.kRight), ControllerConstants.kDriverDeadBandRightY);
		}

	}

	public static void configureButtonBindings(DriveSubsystem m_robotDrive, Hopper m_hopper, Spinner m_spinner,
			Shooter m_shooter, Turret m_turret, VisionProcessor m_visionProcesor) {

		// Driver button commands
		backButtonDriver.whenPressed(new resetGyroCommand(m_robotDrive));
		aButtonDriver.whenPressed(new turnToCommand(180, m_robotDrive));
		bButtonDriver.whenPressed(new turnToCommand(270, m_robotDrive));
		xButtonDriver.whenPressed(new turnToCommand(90, m_robotDrive));
		yButtonDriver.whenPressed(new turnToCommand(0, m_robotDrive));
		l3ButtonDriver.whenPressed(new cancelDriveCommand(m_robotDrive));

		// Operator button commands
		leftBumperOperator.whileHeld(new StopAllCommand(m_shooter, m_spinner, m_hopper));
		xButtonOperator.whenPressed(new RotateHopperCommand(m_hopper));
		aButtonOperator.whileHeld(new ReverseIntakeCommand(m_spinner));
		leftTriggerOperator.whenActive(new ShootLongCommand(m_shooter));
		yButtonOperator.whenPressed(new ShootShortCommand(m_shooter));
		backButtonOperator.whileHeld(new ReverseFeederCommand(m_shooter));

	}
}