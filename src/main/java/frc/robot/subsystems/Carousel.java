// This is the Carousel at the bottom of the hopper that feeds into the shooter
// Platypirates team 9181 - Written by Barbara

package frc.robot.subsystems;

//importing things 

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

public class Carousel extends SubsystemBase {

    // motors
    private final TalonFX carouselMotor;

    // constructor
    public Carousel() {
        carouselMotor = new TalonFX(Constants.DriveConstants.kCarouselMotorCanId);

    }
    
    // methods
    public void moveCarousel() {
        carouselMotor.set(Constants.SubsystemConstants.kCarouselMotorSpeed);
    }

    public void stopCarousel() {
        carouselMotor.set(0.0);
    }

    public void reverseCarousel() {
        carouselMotor.set(Constants.SubsystemConstants.kCarouselMotorReverse);
    }


    
}