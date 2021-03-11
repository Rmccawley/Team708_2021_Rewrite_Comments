package frc.robot.subsystems;

import com.revrobotics.CANAnalog;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.StatusFrame;
// import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.SpeedControllerGroup;


public class Shooter extends SubsystemBase {

    public CANSparkMax shooterMotor, shooterMotor2, feederMotor;
    public CANEncoder  shooterEncoder, shooterEncoder2, feederEncoder;
    private CANPIDController shooterPIDController, shooterPIDController2; // feederPIDController;

    public TalonSRX turretMotor;
    public double   targetSpeed;

    public Solenoid hoodSolenoid;
    public boolean  shooterSet = false;
    public boolean  hoodUp     = false;

    // private double  feederMotorSpeed       = Constants.kFEEDERMOTORSPEED;
            
    int turretEncoderReverseFactor = 1;
    private CANAnalog analogSensor;
    boolean preloaded = false;
    public boolean shooting = false;
    public boolean findtarget = false;

    public Shooter(){

        shooterMotor  = new CANSparkMax(ShooterConstants.kshooterShootMotor,  MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(ShooterConstants.kshooterShootMotor2, MotorType.kBrushless);
        feederMotor   = new CANSparkMax(ShooterConstants.kfeederFeedMotor,    MotorType.kBrushless);

        shooterMotor.setInverted(false);
        shooterMotor2.setInverted(true);

        shooterMotor.setIdleMode(IdleMode.kCoast);
        shooterMotor2.setIdleMode(IdleMode.kCoast);

        feederMotor.setIdleMode(IdleMode.kBrake);

        shooterEncoder = shooterMotor.getEncoder();
        shooterPIDController = shooterMotor.getPIDController();
        shooterPIDController.setP(.0005);
        shooterPIDController.setI(0.0000002);
        shooterPIDController.setD(0); //.00006
        shooterPIDController.setFF(.0002);
        shooterPIDController.setIZone(0);
        shooterPIDController.setOutputRange(-1, 1);

        shooterEncoder2 = shooterMotor2.getEncoder();
        shooterPIDController2 = shooterMotor2.getPIDController();
        shooterPIDController2.setP(.0005);
        shooterPIDController2.setI(0.0000002);
        shooterPIDController2.setD(0); //.00006
        shooterPIDController2.setFF(.0002);
        shooterPIDController2.setIZone(0);
        shooterPIDController2.setOutputRange(-1, 1);

        analogSensor = feederMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);

        hoodSolenoid = new Solenoid(ShooterConstants.kHoodSolenoid);
        hoodSolenoid.set(hoodUp); //Starts with hood down
    }
    
    public void feederOn(){
            shooting = true;
            feederMotor.set(1.0);
            //Robot.hopper.moveMotorCounterClockwise();       
    }

    public void feederOff(){
        feederMotor.set(0);
        shooting = false;
    }

    public void feederUnload(){
        int i;
        for (i=1; i<8000; i++)
            feederMotor.set(-.3);
        feederMotor.set(0);
        shooting = false;
    }

    public void feederPreLoad(){
        if (analogSensor.getPosition() > 1.3 && preloaded == false){
            feederMotor.set(0);
            preloaded = true;
            feederUnload();
        }
        else
         if (!preloaded) feederMotor.set(0.4);
           
    }
    public void stopShooter() {
        shooterMotor.stopMotor();
        shooterMotor2.stopMotor();
        shooting = false;
        findtarget = false;
    }

    public void feederAutoShoot(){
        if (isShooterAtSpeed()) {
            shooting = true;
            feederMotor.set(1.0);
            //Robot.hopper.moveMotorCounterClockwise();
        }    
    }

    // private void setTargetSpeed(double speed) {
    //     targetSpeed=speed;
    // }
    
    // public double determineShooterSpeed(double distance){
    //    if (distance >= Constants.kHOODANGLE_LONGSHOT) {
    //     //    moveHoodUp();
    //        return(Constants.kSHOOTER_WHEELSPEED_LONG);
    //     }
    //     else
    //     {
    //     //    moveHoodDown();
    //        return(Constants.kSHOOTER_WHEELSPEED_SHORT);
    //     }
    // }

    public void shootLong(){
        targetSpeed = ShooterConstants.kSHOOTER_WHEELSPEED_LONG;
        moveHoodUp();
        findtarget = true;
        shooterPIDController.setReference(ShooterConstants.kSHOOTER_WHEELSPEED_LONG, ControlType.kVelocity);
        shooterPIDController2.setReference(ShooterConstants.kSHOOTER_WHEELSPEED_LONG, ControlType.kVelocity);
    }

    public void shootShort(){
        targetSpeed = ShooterConstants.kSHOOTER_WHEELSPEED_SHORT;
        moveHoodDown();
        findtarget = true;
        shooterPIDController.setReference(ShooterConstants.kSHOOTER_WHEELSPEED_SHORT, ControlType.kVelocity);
        shooterPIDController2.setReference(ShooterConstants.kSHOOTER_WHEELSPEED_SHORT, ControlType.kVelocity);
    }

    // public void shootAutoLong(){
    //     double RPM = determineShooterSpeed(Robot.visionprocessor.getDistance());
    //     setTargetSpeed(RPM); 
    //     findtarget = true;

    //     shooterPIDController.setReference(Constants.kSHOOTER_WHEELSPEED_LONG, ControlType.kVelocity);
    //     shooterPIDController2.setReference(Constants.kSHOOTER_WHEELSPEED_LONG, ControlType.kVelocity);
    // }

    public boolean isShooterAtSpeed(){
        if ((Math.abs(shooterEncoder.getVelocity())>(targetSpeed)*0.9))
            return true;
        else
            return false;
    }

    // public boolean getHoodPosition(){
    //     return hoodUp;
    // }

    public void moveHoodUp(){
        hoodUp = true;
        hoodSolenoid.set(hoodUp);
    }

    public void moveHoodDown(){
        hoodUp = false;
        hoodSolenoid.set(hoodUp);
    }

    // public void toggleHood(){
    //     hoodUp = !hoodUp;
    //     hoodSolenoid.set(hoodUp);
    // }
    
    public void sendToDashboard() {
        
        // SmartDashboard.putNumber("Theor. RPM", ((determineShooterSpeed(Robot.visionprocessor.getDistance())*Constants.kSHOOTER_MAXSPEED)));
        // SmartDashboard.putNumber("Theor. RPM %", ((determineShooterSpeed(Robot.visionprocessor.getDistance()))));
        // SmartDashboard.putNumber("Shooter RPM", determineShooterSpeed(Robot.visionprocessor.getDistance()));
        // SmartDashboard.putBoolean("Shooter Hood up", hoodUp);
        // SmartDashboard.putNumber("Shooter1 velocity", shooterEncoder.getVelocity());
        // SmartDashboard.putNumber("Shooter2 velocity", shooterEncoder2.getVelocity());
        // SmartDashboard.putNumber("Shooter target speed", targetSpeed);
        // SmartDashboard.putNumber("Feeder Has Ball", analogSensor.getPosition());
        // SmartDashboard.putNumber("Feeder Motor Temp", feederMotor.getMotorTemperature());
        SmartDashboard.putBoolean("Shooter Target Speed Achieved", isShooterAtSpeed());
        SmartDashboard.putBoolean("Feeder Preload", preloaded);

    }
    
}