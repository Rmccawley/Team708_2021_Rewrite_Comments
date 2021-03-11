package frc.team708.robot.subsystems;

import frc.team708.robot.Constants.SpinnerConstants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spinner extends SubsystemBase {

    public CANSparkMax spinnerMotor;
    public CANEncoder spinnerEncoder;
    private Solenoid spinnerSolenoid;
    public CANPIDController spinnerPID;

    private double intakeSpeed = 0.5;
    private boolean pistonExtend;
    private double spinnerMotorSpeed = .3;

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
}
