// This is the Carousel at the bottom of the hopper that feeds into the shooter
// Platypirates team 9181 - Written by Barbara

package frc.robot.subsystems;

//importing things 

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;

public class Carousel extends SubsystemBase {

    // motors
    private final SparkMax carouselMotor;

    // constructor
    public Carousel() {
        carouselMotor = new SparkMax(9, MotorType.kBrushless);

    }
    
    // methods
    public void moveCarousel() {
        carouselMotor.set(1.0);
    }

    public void stopCarousel() {
        carouselMotor.set(0.0);
    }

    public void reverseCarousel() {
        carouselMotor.set(-1.0);
    }


    
}
