// This is the subsystem for the movable hood of our shooter
// PlatyPirates team 9181 - Written by Barbara

package frc.robot.subsystems;

// imports
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;

public class Hood extends SubsystemBase {

    // motors 
    private final SparkMax hoodMotor;

    // constructor
    public Hood() {
        hoodMotor = new SparkMax(Constants.DriveConstants.kHoodMotorCanId, MotorType.kBrushless);
    }

    // methods
    public void moveHood(double speed) {
        hoodMotor.set(speed);
    }
    
    public void stopHood() {
        hoodMotor.set(0.0);
    }


     
}