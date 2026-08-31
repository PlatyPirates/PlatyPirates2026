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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Intake extends SubsystemBase {

    //fields
    private final SparkFlex intakeMotor;
    private final SparkMax scooperMotor;

    private PIDController pid;

    // constructor activates once when the robot turns on
    public Intake() {
        intakeMotor = new SparkFlex(Constants.DriveConstants.kIntakeArmMotorCanId, SparkLowLevel.MotorType.kBrushless);
        scooperMotor = new SparkMax(Constants.DriveConstants.kIntakeScooperMotorCanId, MotorType.kBrushless);
        
        pid = new PIDController(0.3, 0.016, 0.2);
        pid.setTolerance(0.01);
    }

    public void setPID(){
        double p = SmartDashboard.getNumber("Intake kP", 0.0);
        double i = SmartDashboard.getNumber("Intake kI", 0.0);
        double d = SmartDashboard.getNumber("Intake kD", 0.0);

        System.out.println("Setting values: P = " + p + ", I = " + i + ", D = " + d);
        pid = new PIDController(p, i, d);
        pid.setTolerance(0.01);
        pid.setIZone(2.0);
    }
    
    // methods
    public void extendArm() {
        intakeMotor.set(Constants.SubsystemConstants.kIntakeArmExtend);
    }

    public void retractArm() {
        intakeMotor.set(Constants.SubsystemConstants.kIntakeArmRetract);
    }

    // TODO: make a method to get the Vortex encoder value
    public double getPosition(){
        return 0.0; // CHANGE THIS!
    }

    public void squeeze(){
        double inValue = 1.0;
        double speed = pid.calculate(getPosition(), inValue);
        intakeMotor.set(speed); //add a constant multiplier here if it's going way too fast!
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
