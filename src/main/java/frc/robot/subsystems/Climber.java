//This is the climber code!! 
package frc.robot.subsystems;

// imports
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;

public class Climber extends SubsystemBase {
    //Motors
    private final SparkMax climberMotor;
    private final DigitalInput limitSwitch;
    private final RelativeEncoder climbEncoder;

    //Constructors
    public Climber() {
        climberMotor = new SparkMax(Constants.DriveConstants.kClimberMotorCanId, MotorType.kBrushless);
        limitSwitch = new DigitalInput(2);
        climbEncoder = climberMotor.getEncoder();

    }

    //methods for the limit switch and encoder
    public boolean notAtLimit() {
        return limitSwitch.get();
    }

    public boolean atLimit() {
        return !limitSwitch.get();
    }

    public void resetEncoder() {
        climbEncoder.setPosition(0);
    }

    public double getEncoderPosition() {
        return climbEncoder.getPosition();
    }

    //for the homing command
    public void homeClimber() {
        climberMotor.set(-0.1);
    }

    //Methods 
     // this tells the motor to stop if it ever exeeds a set limit to keep it from overextending itself
    public void moveClimber(double speed) {
        if ((climbEncoder.getPosition() < Constants.SubsystemConstants.kClimberMax && speed > 0
        ||
        climbEncoder.getPosition() > Constants.SubsystemConstants.kClimberMin && speed < 0)) {

            climberMotor.set(speed);

        } else {
            climberMotor.set(0.0);
        }
    }

    public void stopMotor() {
        climberMotor.set(0.0);
    
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Position", climbEncoder.getPosition());
    }


    
}
