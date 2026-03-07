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
        flywheelMotor1 = new TalonFX(Constants.DriveConstants.kFlywheelMotor1CanId);
        flywheelMotor2 = new TalonFX(Constants.DriveConstants.kFlywheelMotor2CanId);
        feedBall = new SparkMax(Constants.DriveConstants.kFeedBallMotorCanId, MotorType.kBrushless);

    }
    
    //methods
    public void baseMotor() {
        feedBall.set(Constants.SubsystemConstants.kFeedBallMotorSpeed);
    }

    public void shoot() {
        flywheelMotor1.setControl(shooterControl.withOutput(Constants.SubsystemConstants.kFlywheel1Speed));
        flywheelMotor2.setControl(shooterControl.withOutput(Constants.SubsystemConstants.kFlywheel2Speed));
    }

    public void stopFlywheels() {
        flywheelMotor1.setControl(shooterControl.withOutput(0.0));
        flywheelMotor2.setControl(shooterControl.withOutput(0.0));
    }

    public void stopFeed() {
        feedBall.set(0.0);
    }

    public void reverseFeed() {
        feedBall.set(Constants.SubsystemConstants.kFeedBallMotorReverse);
    }

    public void reverseFlywheels() {
        flywheelMotor1.setControl(shooterControl.withOutput(Constants.SubsystemConstants.kFlywheel1Reverse));
        flywheelMotor2.setControl(shooterControl.withOutput(Constants.SubsystemConstants.kFlywheel2Reverse));
    }

}
