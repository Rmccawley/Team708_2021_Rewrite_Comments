package frc.team708.robot;

import frc.team708.robot.Constants.ControllerConstants;
import frc.team708.robot.commands.StopAllCommand;
import frc.team708.robot.commands.drive.*;
import frc.team708.robot.commands.hopper.RotateHopperCommand;
import frc.team708.robot.commands.intake.ExtendHangerCommand;
import frc.team708.robot.commands.intake.RetractHangerCommand;
import frc.team708.robot.commands.intake.ReverseIntakeCommand;
import frc.team708.robot.commands.intake.StopIntakeCommand;
import frc.team708.robot.commands.intake.ToColorCommand;
import frc.team708.robot.commands.intake.ToHangerCommand;
import frc.team708.robot.commands.intake.ToIntakeCommand;
import frc.team708.robot.commands.shooter.ReverseFeederCommand;
import frc.team708.robot.commands.shooter.ShootLongCommand;
import frc.team708.robot.commands.shooter.ShootShortCommand;
import frc.team708.robot.subsystems.DriveSubsystem;
import frc.team708.robot.subsystems.Hopper;
import frc.team708.robot.subsystems.Spinner;
import frc.team708.robot.subsystems.VisionProcessor;
import frc.team708.robot.subsystems.Shooter;
import frc.team708.robot.subsystems.Turret;
import frc.team708.robot.util.AxisUp;
import frc.team708.robot.util.DPadButton;
import frc.team708.robot.util.DPadButton.Direction;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


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
	public static final JoystickButton rBumperDriver = new JoystickButton(driverGamepad, Button.kBumperRight.value);
	public static final DPadButton dPadUpDriver = new DPadButton(driverGamepad, Direction.UP);
	public static final DPadButton dPadDownDriver = new DPadButton(driverGamepad, Direction.DOWN);

	public static final JoystickButton xButtonOperator = new JoystickButton(operatorGamepad, Button.kX.value);
	public static final JoystickButton leftBumperOperator = new JoystickButton(operatorGamepad, Button.kBumperLeft.value);
	public static final JoystickButton aButtonOperator = new JoystickButton(operatorGamepad, Button.kA.value);
	public static final JoystickButton yButtonOperator = new JoystickButton(operatorGamepad, Button.kY.value);
	public static final JoystickButton bButtonOperator = new JoystickButton(operatorGamepad, Button.kB.value);
	public static final JoystickButton backButtonOperator = new JoystickButton(operatorGamepad, Button.kBack.value);
	public static final JoystickButton rightBumperOperator = new JoystickButton(operatorGamepad, Button.kBumperRight.value);
	public static final JoystickButton startButtonOperator = new JoystickButton(operatorGamepad, Button.kStart.value);
	public static final DPadButton dPadUpOperator = new DPadButton(operatorGamepad, Direction.UP);
	public static final DPadButton dPadRightOperator = new DPadButton(operatorGamepad, Direction.RIGHT);
	public static final DPadButton dPadDownOperator = new DPadButton(operatorGamepad, Direction.DOWN);

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

	// public static double getOperatorX(Hand hand) {
	// 	if (hand.equals(Hand.kLeft)) {
	// 		return deadBand(operatorGamepad.getX(Hand.kLeft), ControllerConstants.kDriverDeadBandLeftX);
	// 	} else {
	// 		return deadBand(operatorGamepad.getX(Hand.kRight), ControllerConstants.kDriverDeadBandRightX);
	// 	}
	// }

	public static double getOperatorY(Hand hand) {
		if (hand.equals(Hand.kLeft)) {
			return deadBand(operatorGamepad.getY(Hand.kLeft), ControllerConstants.kDriverDeadBandLeftY);
		} else {
			return deadBand(operatorGamepad.getY(Hand.kRight), ControllerConstants.kDriverDeadBandRightY);
		}
	}

	public static void configureButtonBindings(DriveSubsystem m_robotDrive, Hopper m_hopper, Spinner m_spinner,
			Shooter m_shooter, Turret m_turret, VisionProcessor m_visionProcesor) {

		// Driver button commands
		backButtonDriver.whenPressed(new ResetGyroCommand(m_robotDrive));
		// aButtonDriver.whenPressed(new TurnToCommand(180, m_robotDrive));
		// bButtonDriver.whenPressed(new TurnToCommand(270, m_robotDrive));
		// xButtonDriver.whenPressed(new TurnToCommand(90, m_robotDrive));
		// yButtonDriver.whenPressed(new TurnToCommand(0, m_robotDrive));
		l3ButtonDriver.whenPressed(new CancelDriveCommand(m_robotDrive));
		rBumperDriver.whileHeld(new ReverseFeederCommand(m_shooter));

		aButtonDriver.whenHeld(new ShootShortCommand(m_shooter, m_spinner)); //CHANGED WITH ShootShortCommand
		bButtonDriver.whenHeld(new ShootShortCommand(m_shooter, m_spinner)); //CHANGED WITH ShootShortCommand
		xButtonDriver.whenHeld(new ShootLongCommand(m_shooter, m_spinner)); //CHANGED WITH ShootLongCommand
		yButtonDriver.whenHeld(new ShootLongCommand(m_shooter, m_spinner)); //CHANGED WITH ShootLongCommand


		dPadUpDriver.whenPressed(new IncreaseSpeedCommand(m_robotDrive));
		dPadDownDriver.whenPressed(new DecreaseSpeedCommand(m_robotDrive));

		// Operator button commands
		leftBumperOperator.whileHeld(new StopAllCommand(m_shooter, m_spinner, m_hopper));

		rightBumperOperator.toggleWhenPressed(new StopIntakeCommand(m_spinner));

		//WORKING
		xButtonOperator.whenPressed(new RotateHopperCommand(m_hopper));

		//WORKING
		aButtonOperator.whileHeld(new ReverseIntakeCommand(m_spinner));
		aButtonOperator.whenReleased(new ReverseIntakeCommand(m_spinner));

		bButtonOperator.whileHeld(new ExtendHangerCommand(m_spinner));
		startButtonOperator.whileHeld(new RetractHangerCommand(m_spinner));
		
		bButtonOperator.whenReleased(new StopIntakeCommand(m_spinner));
		startButtonOperator.whenReleased(new StopIntakeCommand(m_spinner));

		leftTriggerOperator.whileActiveOnce(new ShootLongCommand(m_shooter, m_spinner)); //CHANGED WITH ShootLongCommand
		//TODO ADD whenReleased command to automate dropping

		yButtonOperator.whenHeld(new ShootShortCommand(m_shooter, m_spinner)); //CHANGED WITH ShootShortCommand

		//WORKING
		backButtonOperator.whileHeld(new ReverseFeederCommand(m_shooter));

		//WORKING
		dPadDownOperator.whenPressed(new ToIntakeCommand(m_spinner, m_hopper, m_shooter));
		dPadRightOperator.whenPressed(new ToColorCommand(m_spinner));
		dPadUpOperator.whenPressed(new ToHangerCommand(m_spinner));

	}
}