// This is the intake for the robot. 
// Moves the intake motor out, activates scooper, retracts
// PlatyPirates 9181 written by Barbara ;)

package frc.robot.subsystems;

// imports
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    //motors
    private final SparkMax intakeMotor;
    private final SparkMax scooperMotor;
    // TODO: add intakeMotor and scooperMotor IDs to constants

    // constructor activates once when the robot turns on
    public Intake() {
        intakeMotor = new SparkMax(0, MotorType.kBrushless); // TODO: replace 0 with real CAN ID
        scooperMotor = new SparkMax(0, MotorType.kBrushless); // TODO: replace 0 with real CAN ID
    }
    
    // methods
    public void extendArm() {
        intakeMotor.set(1.0);
    }

     public void retractArm() {
        intakeMotor.set(-1.0); 
    }

    public void stopMotors() {
        intakeMotor.set(0.0);
        scooperMotor.set(0.0);
    }

    public void stopIntake() {
        intakeMotor.set(0.0);
    }

    public void stopScooper() {
        scooperMotor.set(0.0);
    }

    public void spinScooper() {
        scooperMotor.set(1.0);
    }

}
