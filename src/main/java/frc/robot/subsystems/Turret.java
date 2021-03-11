package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants.TurretConstants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends SubsystemBase {

    public TalonSRX turretMotor;
    public boolean useLimelight = true;

    int turretEncoderReverseFactor = 1;
    int nextsample = 0;
    int samplerate = 3;
    boolean ignorePigeon = false;
    double onedegree = TurretConstants.TURRET_ENCODER_COUNT / 360;
    double normalized = 0;
    double TURRET_MAX_ROTATION = 360;
    double requestedAngleInEnc = 0;
    double requestedAngleInDegress = 0;

    public Turret() {

        turretMotor = new TalonSRX(TurretConstants.kturretMotor);
        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        turretMotor.setSensorPhase(true);
        turretMotor.setInverted(false);
        turretMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 10);
        turretMotor.enableVoltageCompensation(true);
        turretMotor.setNeutralMode(NeutralMode.Brake);

        turretMotor.configVoltageCompSaturation(12, 10);
        turretMotor.configNominalOutputForward(0.0, 10);
        turretMotor.configNominalOutputReverse(0.0, 10);
        turretMotor.configAllowableClosedloopError(0, 0, 10);
        turretMotor.configMotionAcceleration((int) (TurretConstants.TURRET_ROTATION_MAX_SPEED * 5.0), 10); // jnp
        turretMotor.configMotionCruiseVelocity((int) (TurretConstants.TURRET_ROTATION_MAX_SPEED * 5.0), 10);// jnp
        turretMotor.selectProfileSlot(0, 0);
        turretMotor.config_kP(0, 2.1, 10);// 4
        turretMotor.config_kI(0, 0, 10);
        turretMotor.config_kD(0, 21, 10);// 80
        turretMotor.config_kF(0, (.75 * 1023.0) / TurretConstants.TURRET_ROTATION_MAX_SPEED, 10);
        turretMotor.set(ControlMode.MotionMagic, turretMotor.getSelectedSensorPosition(0));
    }

    public synchronized void updateAngle() {
        // calcuate the angle of the turret and add it to Tx
        // double turretPos = (turretMotor.getSelectedSensorPosition(0)) / onedegree; //
        // turret is at this degree
        double cameraAngle = RobotContainer.m_visionProcessor.getRotate(); // target is Tx degrees
        double robotAngle = RobotContainer.m_robotDrive.getHeading(); // Angle in degrees robot is at
        double turretAngle = (turretMotor.getSelectedSensorPosition(0)) / onedegree; // turret is at this degree

        double rotateToTarget = (turretAngle - cameraAngle); // calc numberof degrees to target
        double toEncoderCount = (rotateToTarget * onedegree); // % 360 //calc number of encoder tickets for degrees

        // if (Robot.visionprocessor.seesTarget()) {// && Math.abs(turretAngle) <
        // TURRET_MAX_ROTATION )
        // if (!(rotateToTarget > 270 || rotateToTarget < -90))
        // turretMotor.set(ControlMode.MotionMagic, toEncoderCount);
        // } //turn turret to encoder value to find target
        // else if (Math.abs(robotAngle) <= 15){
        // turretMotor.set(ControlMode.MotionMagic, (robotAngle *
        // onedegree)+Constants.TURRET_ENCODER_STARTING_POS);
        // }

        if (RobotContainer.m_visionProcessor.seesTarget()) // && Math.abs(turretAngle) < TURRET_MAX_ROTATION )
            requestedAngleInEnc = toEncoderCount;
        else
            // requestedAngleInEnc = (robotAngle *
            // onedegree)+Constants.TURRET_ENCODER_STARTING_POS;
            turretMotor.set(ControlMode.MotionMagic, TurretConstants.TURRET_ENCODER_STARTING_POS);

        requestedAngleInDegress = requestedAngleInEnc / onedegree;

        if (requestedAngleInDegress > 280 || requestedAngleInDegress < -100) {
            requestedAngleInDegress = (Math.abs(requestedAngleInDegress) - 360)
                    * Integer.signum((int) requestedAngleInDegress);
            requestedAngleInEnc = requestedAngleInDegress * onedegree;
        }

        if (RobotContainer.m_shooter.findtarget)
            turretMotor.set(ControlMode.MotionMagic, requestedAngleInEnc); // turn turret to encoder value to find
                                                                           // target

        // turretMotor.set(ControlMode.MotionMagic, angle / (2 * Math.PI *
        // Constants.TURRET_ENCODER_COUNT));
        // turretMotor.set(ControlMode.MotionMagic, ((Robot.swerve.getPigeonRotation() +
        // normalized) * onedegree));

        SmartDashboard.putNumber("turret_toEncoderCount", toEncoderCount);
        SmartDashboard.putNumber("turret_Angle", turretAngle);
        SmartDashboard.putNumber("turret_Camera", cameraAngle);
        SmartDashboard.putNumber("turret_Robot_Angle", robotAngle);
        SmartDashboard.putNumber("turret_Rotateto", rotateToTarget);
        SmartDashboard.putNumber("turret_Requested_angle_In_Enc", requestedAngleInEnc);
        SmartDashboard.putNumber("turret_Requested_angle_In_Dec", requestedAngleInDegress);
    }

    public synchronized void resetTurret() {
        turretMotor.set(ControlMode.MotionMagic, TurretConstants.TURRET_ENCODER_STARTING_POS);
        // turretMotor.set(ControlMode.MotionMagic,
        // turretMotor.getSelectedSensorPosition(0));
    }

    synchronized void reset(Rotation2d actual_rotation) {
        turretMotor.set(ControlMode.MotionMagic,
                actual_rotation.getRadians() / (2 * Math.PI * TurretConstants.TURRET_ENCODER_COUNT));
    }

    public synchronized Rotation2d getAngle() {
        return new Rotation2d(
                TurretConstants.TURRET_ENCODER_COUNT * turretMotor.getSelectedSensorPosition() * 2 * Math.PI);
    }

    public synchronized boolean getForwardLimitSwitch() {
        return turretMotor.isFwdLimitSwitchClosed() == 1 ? true : false;
    }

    public synchronized boolean getReverseLimitSwitch() {
        return turretMotor.isRevLimitSwitchClosed() == 1 ? true : false;
    }

    public synchronized double getSetpoint() {
        return turretMotor.getClosedLoopTarget() * TurretConstants.TURRET_ENCODER_COUNT * 360.0;
    }

    private synchronized double getError() {
        return turretMotor.getClosedLoopError() * TurretConstants.TURRET_ENCODER_COUNT * 360.0;
    }

    public synchronized boolean isOnTarget() {
        return (Math.abs(getError()) < TurretConstants.kTurretOnTargetTolerance);
    }

    public synchronized boolean isSafe() {
        return (turretMotor.getClosedLoopTarget() == 0 && Math.abs(getAngle().getDegrees()
                * TurretConstants.TURRET_ENCODER_COUNT * 360.0) < TurretConstants.kTurretSafeTolerance);

    }
}