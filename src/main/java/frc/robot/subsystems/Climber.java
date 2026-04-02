//This is the climber code!! 
package frc.robot.subsystems;

// imports
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    //Motors
    private final SparkMax climberMotor;

    //Constructors
    public Climber() {
        climberMotor = new SparkMax(Constants.DriveConstants.kClimberMotorCanId, MotorType.kBrushless);

    }

    //Methods 
    public void extendClimber() {
        climberMotor.set(Constants.SubsystemConstants.kMoveClimber);

    }

    public void reverseClimber() {
        climberMotor.set(Constants.SubsystemConstants.kReverseClimber);

    }

    public void stopMotor() {
        climberMotor.set(0.0);
    
    }
    
}
