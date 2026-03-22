// This is the subsystem for the movable hood of our shooter
// PlatyPirates team 9181 - Written by Barbara

package frc.robot.subsystems;

// imports
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;

public class Hood extends SubsystemBase {

    // motors 
    private final SparkMax hoodMotor;
    private final RelativeEncoder hoodEncoder;

    // constructor
    public Hood() {
        hoodMotor = new SparkMax(Constants.DriveConstants.kHoodMotorCanId, MotorType.kBrushless);
        hoodEncoder = hoodMotor.getEncoder();
    
    }

    // methods
    // this tells the motor to stop if it ever exeeds a set limit to keep it from overextending itself
    public void moveHood(double speed) {
        if ((hoodEncoder.getPosition() < Constants.SubsystemConstants.kHoodMax && speed > 0) 
        || 
         (hoodEncoder.getPosition() > Constants.SubsystemConstants.kHoodMin && speed < 0.0)) {

            hoodMotor.set(speed);

        } else {
            hoodMotor.set(0.0);
        }
    }
    
    public void stopHood() {
        hoodMotor.set(0.0);
    }

//shows the position of the hood in the Driver Station. This is helpful for tuning and such
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Position", hoodEncoder.getPosition());
    }


     
}