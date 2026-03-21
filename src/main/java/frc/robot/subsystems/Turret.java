// This is for the shooter's turret. 
// This should allow the shooter to rotate 360 degress so we can shoot anywhere on the field
// PlatyPirates team 9181 - Written by Barbara
 
package frc.robot.subsystems;

// imports
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

    // motors
    private final SparkMax turretMotor;

    // constructor
    public Turret() {
        turretMotor = new SparkMax(Constants.DriveConstants.kTurretMotorCanId, MotorType.kBrushless);

    }

    // methods
    public void moveTurret() {
        turretMotor.set(Constants.SubsystemConstants.kTurretSpeed);
    }

    public void stopTurret() {
        turretMotor.set(0.0);
    }

    
}