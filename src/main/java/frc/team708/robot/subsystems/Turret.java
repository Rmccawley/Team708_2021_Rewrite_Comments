package frc.team708.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.team708.robot.Constants.TurretConstants;
import frc.team708.robot.RobotContainer;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    public TalonSRX turretMotor;
    public boolean useLimelight = true;

    int turretEncoderReverseFactor = 1;
    int nextsample = 0;
    int samplerate = 3;
    boolean ignorePigeon = false;
    double onedegree = TurretConstants.kTurretEncoderCount / 360;
    double normalized = 0;
    double TURRET_MAX_ROTATION = 360;
    double requestedAngleInEnc = 0;
    double requestedAngleInDegress = 0;

    public Turret() {

        turretMotor = new TalonSRX(TurretConstants.kTurretMotor);
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
        turretMotor.configMotionAcceleration((int) (TurretConstants.kTurretRotationMaxSpeed * 5.0), 10); // jnp
        turretMotor.configMotionCruiseVelocity((int) (TurretConstants.kTurretRotationMaxSpeed * 5.0), 10);// jnp
        turretMotor.selectProfileSlot(0, 0);
        turretMotor.config_kP(0, 2.1, 10);// 4
        turretMotor.config_kI(0, 0, 10);
        turretMotor.config_kD(0, 21, 10);// 80
        turretMotor.config_kF(0, (.75 * 1023.0) / TurretConstants.kTurretRotationMaxSpeed, 10);
        turretMotor.set(ControlMode.MotionMagic, turretMotor.getSelectedSensorPosition(0));
    }

    public synchronized void updateAngle() {
        double cameraAngle = RobotContainer.m_visionProcessor.getRotate(); // target is Tx degrees
        double turretAngle = (turretMotor.getSelectedSensorPosition(0)) / onedegree; // turret is at this degree

        double rotateToTarget = (turretAngle - cameraAngle); // calc numberof degrees to target
        double toEncoderCount = (rotateToTarget * onedegree); // % 360 //calc number of encoder tickets for degrees

        if (RobotContainer.m_visionProcessor.seesTarget()) // && Math.abs(turretAngle) < TURRET_MAX_ROTATION )
            requestedAngleInEnc = toEncoderCount;
        else
            turretMotor.set(ControlMode.MotionMagic, TurretConstants.kTurretEncoderStartingPos);

        requestedAngleInDegress = requestedAngleInEnc / onedegree;

        if (requestedAngleInDegress > 280 || requestedAngleInDegress < -100) {
            requestedAngleInDegress = (Math.abs(requestedAngleInDegress) - 360)
                    * Integer.signum((int) requestedAngleInDegress);
            requestedAngleInEnc = requestedAngleInDegress * onedegree;
        }

        if (RobotContainer.m_shooter.findtarget)
            turretMotor.set(ControlMode.MotionMagic, requestedAngleInEnc); // turn turret to encoder value to find
                                                                           // target

        SmartDashboard.putBoolean("turret_SeesTarget", RobotContainer.m_visionProcessor.seesTarget());
        SmartDashboard.putNumber("turret_toEncoderCount", toEncoderCount);
        SmartDashboard.putNumber("turret_Angle", turretAngle);
        SmartDashboard.putNumber("turret_Camera", cameraAngle);
        //SmartDashboard.putNumber("turret_Robot_Angle", robotAngle);
        SmartDashboard.putNumber("turret_Rotateto", rotateToTarget);
        SmartDashboard.putNumber("turret_Requested_angle_In_Enc", requestedAngleInEnc);
        SmartDashboard.putNumber("turret_Requested_angle_In_Dec", requestedAngleInDegress);
    }

    public synchronized void resetTurret() {
        turretMotor.set(ControlMode.MotionMagic, TurretConstants.kTurretEncoderStartingPos);
    }

    synchronized void reset(Rotation2d actual_rotation) {
        turretMotor.set(ControlMode.MotionMagic,
                actual_rotation.getRadians() / (2 * Math.PI * TurretConstants.kTurretEncoderCount));
    }

    public synchronized Rotation2d getAngle() {
        return new Rotation2d(
                TurretConstants.kTurretEncoderCount * turretMotor.getSelectedSensorPosition() * 2 * Math.PI);
    }

    public synchronized boolean getForwardLimitSwitch() {
        return turretMotor.isFwdLimitSwitchClosed() == 1 ? true : false;
    }

    public synchronized boolean getReverseLimitSwitch() {
        return turretMotor.isRevLimitSwitchClosed() == 1 ? true : false;
    }

    public synchronized double getSetpoint() {
        return turretMotor.getClosedLoopTarget() * TurretConstants.kTurretEncoderCount * 360.0;
    }

    private synchronized double getError() {
        return turretMotor.getClosedLoopError() * TurretConstants.kTurretEncoderCount * 360.0;
    }

    public synchronized boolean isOnTarget() {
        return (Math.abs(getError()) < TurretConstants.kTurretOnTargetTolerance);
    }

    public synchronized boolean isSafe() {
        return (turretMotor.getClosedLoopTarget() == 0 && Math.abs(getAngle().getDegrees()
                * TurretConstants.kTurretEncoderCount * 360.0) < TurretConstants.kTurretSafeTolerance);

    }
}