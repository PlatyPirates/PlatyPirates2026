// This is the intake for the robot. 
// Moves the intake motor out, activates scooper, retracts
// PlatyPirates 9181 written by Barbara ;)

package frc.robot.subsystems;

// imports
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    //motors
    private final SparkFlex intakeMotor;
    private final SparkMax scooperMotor;
    // TODO: add intakeMotor and scooperMotor IDs to constants

    // constructor activates once when the robot turns on
    public Intake() {
        intakeMotor = new SparkFlex(10, SparkLowLevel.MotorType.kBrushless);
        scooperMotor = new SparkMax(11, MotorType.kBrushless);
    }
    
    // methods
    public void extendArm() {
        intakeMotor.set(0.10);
    }

    public void retractArm() {
        intakeMotor.set(-0.10);
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
        scooperMotor.set(0.5);
    }

    public void reverseScooper() {
        scooperMotor.set(-0.5);

    }

}
