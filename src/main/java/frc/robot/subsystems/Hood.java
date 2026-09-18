// This is the subsystem for the movable hood of our shooter
// PlatyPirates team 9181 - Written by Barbara
package frc.robot.subsystems;

// imports
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.opencv.core.Mat;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

public class Hood extends SubsystemBase {

    // motors 
    private final SparkMax hoodMotor;
    private final RelativeEncoder hoodEncoder;
    private final DigitalInput limitSwitch;
    private final PIDController pidHood;
    private static boolean isHome = false;
    private static boolean atTargetPos = false;
    private static boolean hubMode = true;
    private static double hubPosition = -7;
    private static double fairyPosition = -20;
    private double adjustmentPosition;

    // constructor
    public Hood() {
        hoodMotor = new SparkMax(Constants.DriveConstants.kHoodMotorCanId, MotorType.kBrushless);
        hoodEncoder = hoodMotor.getEncoder();
        limitSwitch = new DigitalInput(0);

        pidHood = new PIDController(Constants.SubsystemConstants.KHoodP,Constants.SubsystemConstants.KHoodI,Constants.SubsystemConstants.KHoodD);
        pidHood.setTolerance(0.01);
        pidHood.setIZone(1.0);
    
    }


    public void home(){
        if(notAtLimit()){
            hoodMotor.set(0.2);
        } else {
            stopHood();
            isHome = true;


        }
    }

    // this is for the limits on the hood
    public boolean notAtLimit() {
        return !limitSwitch.get();
    }

    public boolean atLimit() {
        return limitSwitch.get();
    }

    public void resetEncoder() {
        hoodEncoder.setPosition(0);
    }

    public void hubMode() {
        hubMode = true;
    }

    public void fairyMode() {
        hubMode = false;
    }

    public void setHubPos(double newHubPos) {
        hubPosition = newHubPos;
    }

    public void setFairyPos(double newFairyPos) {
        fairyPosition = newFairyPos;
    }

    public void updateHubPosition() {
        hubPosition = hoodEncoder.getPosition();
    }
    
    public void updateFairyPosition() {
        fairyPosition = hoodEncoder.getPosition();
    }

    public void beginPresetAdjustment() {
        adjustmentPosition = hubMode ? hubPosition : fairyPosition;
        atTargetPos = false;
    }

    public void adjustPreset(double joystickValue) {
        if (!atPosition(adjustmentPosition) && !atTargetPos) {
            pidHoodPos(adjustmentPosition);
            return;
        }
        atTargetPos = true;

        moveHood(joystickValue);

        // adjustmentPosition = MathUtil.clamp(
        //     adjustmentPosition + joystickValue * 0.5,
        //     Constants.SubsystemConstants.kHoodMax,
        //     Constants.SubsystemConstants.kHoodMin
        //     );
        // pidHoodPos(adjustmentPosition);
    }

    public void savePreset() {
        if (hubMode) {
            hubPosition = hoodEncoder.getPosition();
        }
        else {
            fairyPosition = hoodEncoder.getPosition();
        }
    }

    public boolean atPosition(double targetPosition) {
        return MathUtil.isNear(targetPosition, hoodEncoder.getPosition(), 0.1);
    }
    
    public void hoodShootPos() {
        double hoodPos = hubMode ? hubPosition : fairyPosition;
        pidHoodPos(hoodPos);
    }

    public void pidHoodPos(double targetPos) {
        double currPosition = hoodEncoder.getPosition();
        double maxSpeed = 0.3;

        double speed = MathUtil.clamp(pidHood.calculate(currPosition, targetPos),-maxSpeed, maxSpeed);

        moveHood(speed);

    }
        

    // methods
    // this tells the motor to stop if it ever exeeds a set limit to keep it from overextending itself
    public void moveHood(double speed) {
        double position = hoodEncoder.getPosition();
        double max = Constants.SubsystemConstants.kHoodMax;
       

        if (isHome == false){
            home();
        } 
        else{ 
            //UPPER LIMIT: -23.5
            if((notAtLimit() || speed < 0.0) && (position > max || speed > 0.0)){
                hoodMotor.set(speed);
            }
            else {
                hoodMotor.set(0.0);
            }
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
        SmartDashboard.putBoolean("Shooting Mode", hubMode);
        SmartDashboard.putNumber("Hub Hood Pos", hubPosition);
        SmartDashboard.putNumber("Fairy Hood Pos", fairyPosition);
    }


     
}
