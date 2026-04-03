// This is the Carousel at the bottom of the hopper that feeds into the shooter
// Platypirates team 9181 - Written by Barbara

package frc.robot.subsystems;

//importing things 
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Carousel extends SubsystemBase {

    // motors
    private final TalonFX carouselMotor;
    private final DutyCycleOut carouselControl = new DutyCycleOut(0);


    // constructor
    public Carousel() {
        carouselMotor = new TalonFX(Constants.DriveConstants.kCarouselMotorCanId);

    }
    
    // methods
    public void moveCarousel() {
        carouselMotor.setControl(carouselControl.withOutput(Constants.SubsystemConstants.kCarouselMotorSpeed));
    }

    public void stopCarousel() {
        carouselMotor.setControl(carouselControl.withOutput(0.0));
    }

    public void reverseCarousel() {
        carouselMotor.setControl(carouselControl.withOutput(Constants.SubsystemConstants.kCarouselMotorReverse));
    }


    
}
