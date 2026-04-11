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
import edu.wpi.first.wpilibj.DigitalInput;

public class Hood extends SubsystemBase {

    // motors 
    private final SparkMax hoodMotor;
    private final RelativeEncoder hoodEncoder;
    private final DigitalInput limitSwitch;
  
    // constructor
    public Hood() {
        hoodMotor = new SparkMax(Constants.DriveConstants.kHoodMotorCanId, MotorType.kBrushless);
        hoodEncoder = hoodMotor.getEncoder();
        limitSwitch = new DigitalInput(0);
    
    }

    // this is for the limits on the hood
    public boolean notAtLimit() {
        return limitSwitch.get();
    }

    public boolean atLimit() {
        return !limitSwitch.get();
    }

    public void resetEncoder() {
        hoodEncoder.setPosition(0);
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
    
    //sets the speed for the homing routine. It doesn't have the soft limits because it would break done when trying to home
    public void homeHood() {
        hoodMotor.set(-0.1);
    }

    @Override
    public void periodic() {
        if (atLimit()) {
            resetEncoder();
        } 
        SmartDashboard.putNumber("Hood Position", hoodEncoder.getPosition());
        SmartDashboard.putBoolean("Hood At Limit", atLimit());
    }


     
}
