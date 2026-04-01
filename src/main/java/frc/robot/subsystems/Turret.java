// This is for the shooter's turret. 
// This should allow the shooter to rotate 360 degress so we can shoot anywhere on the field
// PlatyPirates team 9181 - Written by Barbara
 
package frc.robot.subsystems;

// imports
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;


public class Turret extends SubsystemBase {

    // motors
    private final SparkMax turretMotor;
    private final RelativeEncoder turretEncoder;
    private final DigitalInput limitSwitch;

    // constructor
    public Turret() {
        turretMotor = new SparkMax(Constants.DriveConstants.kTurretMotorCanId, MotorType.kBrushless);
        turretEncoder = turretMotor.getEncoder();
        limitSwitch = new DigitalInput(1);
        
    }

    public void resetEncoder() {
        turretEncoder.setPosition(0);
    }
    
    //for the homing routine tells us when we hit the limit switch
    public boolean notAtLimit() {
        return limitSwitch.get();
    }

    public boolean atLimit() {
        return !limitSwitch.get();
    }

    // methods
    // this one below tells the motor to stop if it gets past a set limit in order to keep the turret from overextending itself
    public void moveTurret(double speed) {
        if ((turretEncoder.getPosition() < Constants.SubsystemConstants.kTurretMax && speed > 0)
        ||
        (turretEncoder.getPosition() > Constants.SubsystemConstants.kTurretMin && speed < 0)) {

            turretMotor.set(speed);

        } else {
            turretMotor.set(0.0);
        }

    }

    public void stopTurret() {
        turretMotor.set(0.0);
    }
    
    //sets the speed for the homing routine. Doesn't have soft limits for this reason.
    public void homeTurret() {
        turretMotor.set(-0.1);
    }

    //shows the position of the turret in the Driver Station
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Position", turretEncoder.getPosition());
    }

    
}