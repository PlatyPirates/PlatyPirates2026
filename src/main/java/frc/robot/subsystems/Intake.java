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
    // constructor activates once when the robot turns on
    public Intake() {
        intakeMotor = new SparkFlex(Constants.DriveConstants.kIntakeArmMotorCanId, SparkLowLevel.MotorType.kBrushless);
        scooperMotor = new SparkMax(Constants.DriveConstants.kIntakeScooperMotorCanId, MotorType.kBrushless);
    }
    
    // methods
    public void extendArm() {
        intakeMotor.set(Constants.SubsystemConstants.kIntakeArmExtend);
    }

    public void retractArm() {
        intakeMotor.set(Constants.SubsystemConstants.kIntakeArmRetract);
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
        scooperMotor.set(Constants.SubsystemConstants.kScooperMotorSpeed);
    }

    public void reverseScooper() {
        scooperMotor.set(Constants.SubsystemConstants.kScooperMotorReverse);

    }

}
