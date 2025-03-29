package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final SparkMax intake;

    public Intake(){
        intake = new SparkMax(Constants.DriveConstants.kIntakeCanId, MotorType.kBrushless);
    }

    public void setSpeed(double speed){
        intake.set(speed);
    }
}
