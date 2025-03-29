package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase{
    private AddressableLED addressableLED;
    private AddressableLEDBuffer ledBuffer;
    private Distance ledSpacing = Meters.of(1/30.0);

    private Color maroon = Color.kMaroon;

    public LEDs(int length){
        addressableLED = new AddressableLED(1);

        ledBuffer = new AddressableLEDBuffer(length); // Our full LED strip length (76)
        addressableLED.setLength(ledBuffer.getLength());

        addressableLED.setData(ledBuffer);
        addressableLED.start();
    }

    public void applyPattern(LEDPattern pattern){
        pattern.applyTo(ledBuffer);
        addressableLED.setData(ledBuffer);
    }
    
    public void maroon(){
       applyPattern(LEDPattern.solid(maroon)); 
    }

    public void red(){
        applyPattern(LEDPattern.solid(Color.kRed)); 
    }

    public void rainbow(){
        applyPattern(LEDPattern.rainbow(255, 128)); 
    }

    public void purple(){
        applyPattern(LEDPattern.solid(Color.kPurple)); 
    }

    public void green(){
        applyPattern(LEDPattern.solid(Color.kGreen)); 
    }

    public void yellow(){
        applyPattern(LEDPattern.solid(Color.kYellow)); 
    }

    public void scrollingRainbow(){
        applyPattern(LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpacing)); 
    }

    public void maroonAndWhite(){
        gradient(maroon, Color.kWhite);
    }

    public void maroonAndWhiteScrolling(){
        scrollingGradient(maroon, Color.kWhite);
    }

    public void gradient(Color color1, Color color2){
        applyPattern(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, color1, color2));
    }

    public void scrollingGradient(Color color1, Color color2){
        applyPattern(LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, color1, color2).scrollAtRelativeSpeed(Percent.per(Second).of(25)));
    }

    public void blink(Color color){
        applyPattern(LEDPattern.solid(color).blink(Seconds.of(0.5)));
    }

    public void loading(){
        applyPattern(LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kWhite, new Color()).scrollAtRelativeSpeed(Percent.per(Second).of(50)));
    }

    public void error(){
        breathe(Color.kRed);
    }

    public void breathe(Color color){
        applyPattern(LEDPattern.solid(color).breathe(Seconds.of(1)));
    }

    @Override
    public void periodic(){
        
    }

}
