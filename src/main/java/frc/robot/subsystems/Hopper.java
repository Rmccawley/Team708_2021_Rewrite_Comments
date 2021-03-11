package frc.robot.subsystems;


import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.HopperConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

    public CANSparkMax hopperMotor;
    public CANEncoder  hopperEncoder;
    private CANPIDController hopperPIDController;

    private boolean hopperClockwise;
    private double hopperspeed = HopperConstants.kHopperSpeed; //speed of Hooper

    public Hopper(){
        hopperMotor = new CANSparkMax(HopperConstants.kHopperMotor, MotorType.kBrushless);
        hopperMotor.setInverted(false);

        hopperEncoder = hopperMotor.getEncoder();
        hopperPIDController = hopperMotor.getPIDController();
        hopperMotor.setIdleMode(IdleMode.kCoast);
        hopperPIDController.setP(0.0001);
        hopperPIDController.setI(0.0000002);
        hopperPIDController.setD(0);
        hopperPIDController.setIZone(0);
        hopperPIDController.setFF(.000002);  //.1
        hopperPIDController.setOutputRange(-1,1);

    }

    public void moveMotorClockwise(){
        // hopperMotor.set(0);  
        hopperPIDController.setReference(hopperspeed*-1, ControlType.kVelocity);
        hopperClockwise = true;
    }
    public void moveMotorCounterClockwise(){
        // hopperMotor.set(0);  
        hopperPIDController.setReference(hopperspeed, ControlType.kVelocity);
        hopperClockwise = false;
    }
    public void stopMotor(){
        hopperMotor.set(0.0);
    }

    // uses same speed as above, sets it to negative
    public void reverseMotor(){
        if (hopperClockwise)
            moveMotorCounterClockwise();
        else
            moveMotorClockwise();
    }
}