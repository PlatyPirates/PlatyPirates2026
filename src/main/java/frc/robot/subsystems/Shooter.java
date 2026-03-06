// This is the code for the shooter. We have a movable turret and active hood this year
// PlatyPirates team 9181 - written by Barbara

package frc.robot.subsystems;

//imports
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class Shooter extends SubsystemBase {

    //motors-Kracken
    private final TalonFX flywheelMotor1;
    private final TalonFX flywheelMotor2;
    private final DutyCycleOut shooterControl = new DutyCycleOut(0);

    //motors-Sparkmax
    private final SparkMax feedBall;

    //constructor
    public Shooter() {
        flywheelMotor1 = new TalonFX(0); //TODO Add real CAN IDs
        flywheelMotor2 = new TalonFX(0); //TODO come back with real CAN ID
        feedBall = new SparkMax(0, MotorType.kBrushless); //TODO come back with real CAN ID

    }
    
    //methods
    public void baseMotor() {
        feedBall.set(1.0);
    }

    public void shoot() {
        flywheelMotor1.setControl(shooterControl.withOutput(1.0));
        flywheelMotor2.setControl(shooterControl.withOutput(1.0));
    }

    public void stopFlywheels() {
        flywheelMotor1.setControl(shooterControl.withOutput(0.0));
        flywheelMotor2.setControl(shooterControl.withOutput(0.0));
    }

    public void stopFeed() {
        feedBall.set(0.0);
    }

    public void reverseFeed() {
        feedBall.set(-1.0);
    }

}
