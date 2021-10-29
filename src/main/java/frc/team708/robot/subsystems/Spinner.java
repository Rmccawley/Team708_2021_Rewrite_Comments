package frc.team708.robot.subsystems;

import frc.team708.robot.Constants.SpinnerConstants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.team708.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Spinner extends SubsystemBase {

    public CANSparkMax spinnerMotor;
    public CANEncoder spinnerEncoder;
    private Solenoid spinnerSolenoid;
    public CANPIDController spinnerPID;

    private double intakeSpeed = 0.5;
    private boolean pistonExtend;
    private double spinnerMotorSpeed = .3;

    public CANSparkMax intakeMotor;

    public DoubleSolenoid camSolenoid;
    public DoubleSolenoid pivotSolenoid;

    public Solenoid shifterHanger;
    public Solenoid lockHanger;

    private boolean intakeIn = true;
    public boolean inHangerPosition = false;
    public boolean inIntakePosition = false;
    public boolean stopHanger = false;

    private double motordirection = .5; // intake Motor speed
    private double intakeMotorSpeed; // start with motor spinning forward

    public Spinner() {
        spinnerMotor = new CANSparkMax(SpinnerConstants.kSpinnerMotor, MotorType.kBrushless);
        spinnerEncoder = spinnerMotor.getEncoder();
        spinnerSolenoid = new Solenoid(SpinnerConstants.kLittlePecker);
        spinnerPID = spinnerMotor.getPIDController();
        spinnerPID.setP(0.1);
        spinnerPID.setI(0);
        spinnerPID.setD(0);
        spinnerPID.setFF(0.1);
        spinnerPID.setIZone(0);
        spinnerPID.setOutputRange(-0.42, 0.42);
        spinnerMotor.setIdleMode(IdleMode.kBrake);
        spinnerEncoder.setPosition(0);
        spinnerMotor.setInverted(false);
        spinnerSolenoid.set(false);
        
        camSolenoid = new DoubleSolenoid(IntakeConstants.armCam0, IntakeConstants.armCam1);
        pivotSolenoid = new DoubleSolenoid(IntakeConstants.armPivot0, IntakeConstants.armPivot1);

        shifterHanger = new Solenoid(IntakeConstants.hangerEngage);

        unlockHanger();

        toColorFromIntake();
    }

    public void SpinMotor(double speed) {
        spinnerMotor.set(spinnerMotorSpeed);
    }

    public void spinnerMotorStop() {
        spinnerMotor.set(0);
    }

    public void resetSpinnerEncoder() {
        spinnerEncoder.setPosition(0.0);
    }

    public double getSpinMotorCount() {
        return (spinnerEncoder.getPosition());
    }

    public boolean getPistonPosition() {
        return pistonExtend;
    }

    public void pistonExtend() {
        pistonExtend = true;
        spinnerSolenoid.set(pistonExtend);
    }

    public void pistonRetract() {
        pistonExtend = false;
        spinnerSolenoid.set(pistonExtend);
    }

    public void moveIntakeForward() {
        spinnerMotor.set(SpinnerConstants.kIntakeSpeed);
    }

    public void reverseIntake() {
        spinnerMotor.set(-SpinnerConstants.kIntakeSpeed);
    }

    public void toggleMotorIntake() {
        intakeSpeed *= -1;
        spinnerMotor.set(intakeSpeed);
    }

    public void StopMotorIntake() {
        spinnerMotor.set(0);
    }


    public void toIntake() {
        unlockHanger();
        pistonRetract();
        camSolenoid.set(DoubleSolenoid.Value.kForward); // I
        pivotSolenoid.set(DoubleSolenoid.Value.kReverse); // O
        moveMotorIntakeOut();
        inHangerPosition = false;
        inIntakePosition = true;
    }


    public void toHanger() {
        if (Timer.getMatchTime() <= 35) {
            StopMotorIntake();
            pistonRetract();
            camSolenoid.set(DoubleSolenoid.Value.kForward); // I
            pivotSolenoid.set(DoubleSolenoid.Value.kForward); // I
            lockHanger();
            inHangerPosition = true;
            inIntakePosition = false;
            resetSpinnerEncoder();
        }
    }

    public void toColor() {
        if (inIntakePosition) {
            toColorFromIntake();
        }
        else if (inHangerPosition){
            toColorFromHanger();
        }
    }

    public void toColorFromIntake() {
        camSolenoid.set(DoubleSolenoid.Value.kReverse); // O
        pivotSolenoid.set(DoubleSolenoid.Value.kForward); // I
        unlockHanger();
        StopMotorIntake();
        resetSpinnerEncoder();
        inHangerPosition = false;
        inIntakePosition = false;
    }

    public void toColorFromHanger() {
        camSolenoid.set(DoubleSolenoid.Value.kReverse); // O
        pivotSolenoid.set(DoubleSolenoid.Value.kReverse); // O
        unlockHanger();
        StopMotorIntake();
        inHangerPosition = false;
        inIntakePosition = false;
    }

    public void shiftToHanger() {
        if (inHangerPosition)
            shifterHanger.set(false);
        else
            shifterHanger.set(true);
    }

    private boolean notExtended() {
        return (getSpinMotorCount() < 160);
    }

    private boolean notRetracted() {
        return (getSpinMotorCount() > 05);
    }

    public void moveHanger(double Y) {
        if (inHangerPosition) {
            if (Y < 0 && notExtended())
                spinnerMotor.set(-Y);
            // Robot.spinner.spinnerPID.setReference(50, ControlType.kPosition); //169 max
            else if (Y > 0 && notRetracted())
                // Robot.spinner.spinnerPID.setReference(20, ControlType.kPosition); //5 min
                spinnerMotor.set(-Y);
            else
                spinnerMotor.set(0);
            stopHanger = true;
        }
    }

    public void stopHanger() {
        if (inHangerPosition) {
            spinnerMotor.set(0.0);
            stopHanger = false;
        }
    }

    public void lockHanger() {
        shifterHanger.set(false);
    }

    public void unlockHanger() {
        shifterHanger.set(true);
    }

    public boolean getIntakePosition() {
        return intakeIn;
    }

    public void intakeToggleMotor() {
        if (intakeMotorSpeed != 0)
            intakeMotorSpeed = 0 * motordirection;
        else
            intakeMotorSpeed = 1 * motordirection;

        spinnerMotor.set(intakeMotorSpeed); // turns motor off
    }

    public void moveMotorIntakeIn() {
        intakeIn = true;
        spinnerMotor.set(0); // turns motor off
    }

    public void moveMotorIntakeOut() {
        intakeIn = false;
        spinnerMotor.set(motordirection); // turns motor on
    }

    public void moveColorWheel() {
        intakeIn = false;
        spinnerMotor.set(-.3); // turns motor on
    }

    public void sendToDashboard() {
        SmartDashboard.putBoolean("Hanger extended", !notExtended());
        SmartDashboard.putBoolean("Hanger retracted", !notRetracted());
        SmartDashboard.putNumber("FMS Match Time", Timer.getMatchTime());
        SmartDashboard.putNumber("Hanger Get Reference", spinnerEncoder.getPosition());
    }
}
