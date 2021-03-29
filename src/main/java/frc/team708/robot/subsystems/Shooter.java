package frc.team708.robot.subsystems;

import com.revrobotics.CANAnalog;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.team708.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {

    public CANSparkMax shooterMotor, shooterMotor2, feederMotor;
    public CANEncoder shooterEncoder, shooterEncoder2, feederEncoder;
    private CANPIDController shooterPIDController, shooterPIDController2; // feederPIDController;

    public TalonSRX turretMotor;
    public double targetSpeed;

    public Solenoid hoodSolenoid;
    public boolean shooterSet = false;
    public boolean hoodUp = false;

    // private double feederMotorSpeed = Constants.kFEEDERMOTORSPEED;

    int turretEncoderReverseFactor = 1;
    private CANAnalog analogSensor;
    boolean preloaded = false;
    public boolean shooting = false;
    public boolean findtarget = false;

    public Shooter() {

        shooterMotor = new CANSparkMax(ShooterConstants.kShooterShootMotor, MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(ShooterConstants.kShooterShootMotor2, MotorType.kBrushless);
        feederMotor = new CANSparkMax(ShooterConstants.kFeederFeedMotor, MotorType.kBrushless);

        shooterMotor.setInverted(false);
        shooterMotor2.setInverted(true);

        shooterMotor.setIdleMode(IdleMode.kCoast);
        shooterMotor2.setIdleMode(IdleMode.kCoast);

        feederMotor.setIdleMode(IdleMode.kBrake);

        shooterEncoder = shooterMotor.getEncoder();
        shooterPIDController = shooterMotor.getPIDController();
        shooterPIDController.setP(.0005);
        shooterPIDController.setI(0.0000002);
        shooterPIDController.setD(0); // .00006
        shooterPIDController.setFF(.0002);
        shooterPIDController.setIZone(0);
        shooterPIDController.setOutputRange(-1, 1);

        shooterEncoder2 = shooterMotor2.getEncoder();
        shooterPIDController2 = shooterMotor2.getPIDController();
        shooterPIDController2.setP(.0005);
        shooterPIDController2.setI(0.0000002);
        shooterPIDController2.setD(0); // .00006
        shooterPIDController2.setFF(.0002);
        shooterPIDController2.setIZone(0);
        shooterPIDController2.setOutputRange(-1, 1);

        analogSensor = feederMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);

        hoodSolenoid = new Solenoid(ShooterConstants.kHoodSolenoid);
        hoodSolenoid.set(hoodUp); // Starts with hood down
    }

    public void feederOn() {
        shooting = true;
        feederMotor.set(1.0);
        // Robot.hopper.moveMotorCounterClockwise();
    }

    public void feederOff() {
        feederMotor.set(0);
        shooting = false;
    }

    public void feederUnload() {
        int i;
        for (i = 1; i < 8000; i++)
            feederMotor.set(-.3);
        feederMotor.set(0);
        shooting = false;
    }

    public void feederPreLoad() {
        if (analogSensor.getPosition() > 1.3 && preloaded == false) {
            feederMotor.set(0);
            preloaded = true;
            feederUnload();
        } else if (!preloaded)
            feederMotor.set(0.4);

    }

    public void stopShooter() {
        shooterMotor.stopMotor();
        shooterMotor2.stopMotor();
        shooting = false;
        findtarget = false;
    }

    public void feederAutoShoot() {
        if (isShooterAtSpeed()) {
            shooting = true;
            feederMotor.set(1.0);
            // Robot.hopper.moveMotorCounterClockwise();
        }
    }

    public void shootLong() {
        targetSpeed = ShooterConstants.kShooterWheelSpeedLong;
        moveHoodUp();
        findtarget = true;
        shooterPIDController.setReference(ShooterConstants.kShooterWheelSpeedLong, ControlType.kVelocity);
        shooterPIDController2.setReference(ShooterConstants.kShooterWheelSpeedLong, ControlType.kVelocity);
        feederAutoShoot();
    }

    public void shootShort() {
        targetSpeed = ShooterConstants.kShooterWheelSpeedShort;
        moveHoodDown();
        findtarget = true;
        shooterPIDController.setReference(ShooterConstants.kShooterWheelSpeedShort, ControlType.kVelocity);
        shooterPIDController2.setReference(ShooterConstants.kShooterWheelSpeedShort, ControlType.kVelocity);
        feederAutoShoot();
    }

    public boolean isShooterAtSpeed() {
        if ((Math.abs(shooterEncoder.getVelocity()) > (targetSpeed) * 0.9))
            return true;
        else
            return false;
    }

    public void moveHoodUp() {
        hoodUp = true;
        hoodSolenoid.set(hoodUp);
    }

    public void moveHoodDown() {
        hoodUp = false;
        hoodSolenoid.set(hoodUp);
    }

    public void sendToDashboard() {

        // SmartDashboard.putNumber("Theor. RPM",
        // ((determineShooterSpeed(Robot.visionprocessor.getDistance())*Constants.kSHOOTER_MAXSPEED)));
        // SmartDashboard.putNumber("Theor. RPM %",
        // ((determineShooterSpeed(Robot.visionprocessor.getDistance()))));
        // SmartDashboard.putNumber("Shooter RPM",
        // determineShooterSpeed(Robot.visionprocessor.getDistance()));
        // SmartDashboard.putBoolean("Shooter Hood up", hoodUp);
        // SmartDashboard.putNumber("Shooter1 velocity", shooterEncoder.getVelocity());
        // SmartDashboard.putNumber("Shooter2 velocity", shooterEncoder2.getVelocity());
        // SmartDashboard.putNumber("Shooter target speed", targetSpeed);
        // SmartDashboard.putNumber("Feeder Has Ball", analogSensor.getPosition());
        // SmartDashboard.putNumber("Feeder Motor Temp",
        // feederMotor.getMotorTemperature());
        SmartDashboard.putBoolean("Shooter Target Speed Achieved", isShooterAtSpeed());
        SmartDashboard.putBoolean("Feeder Preload", preloaded);

    }

}