package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase{
    private AddressableLED addressableLED;
    private AddressableLEDBuffer ledBuffer;
    private Color color1, color2;
    private double timestamp = Timer.getFPGATimestamp();
    private boolean solid = true;

    public LEDs(int length){
        addressableLED = new AddressableLED(0);

        ledBuffer = new AddressableLEDBuffer(length); // Our full LED strip length (76)
        addressableLED.setLength(ledBuffer.getLength());
    }
    public void setLEDs(int red, int green, int blue){
        for (int i = 0; i < ledBuffer.getLength(); i++) {
          // Sets the specified LED to the RGB values for red
          ledBuffer.setRGB(i, red, green, blue);
        }
        addressableLED.setData(ledBuffer);
        addressableLED.start();
      }
    public void setSolid(int red, int green, int blue){
        color2 = null;
        color1 = new Color(red, green, blue);
        solid = true;
    }
    public void setFlash(int red, int green, int blue){
        Color newColor = new Color(red, green, blue);
        if(!newColor.equals(color1) && !newColor.equals(color2)){
            if(color2 == null && !solid){
                color2 = newColor;
            } else {
                color1 = newColor;
            }
        }
        solid = false;
    }

    public void updateLEDs(){
        if(color2 != null){
            if(Timer.getFPGATimestamp() - timestamp >= 0.5){
                Color temp = color1;
                color1 = color2;
                color2 = temp;
                timestamp = Timer.getFPGATimestamp();
            }

            setLEDs((int)(color1.red*255), (int)(color1.green*255), (int)(color1.blue*255));
        }
    }

    @Override
    public void periodic(){
        updateLEDs();
        if(color1 != null){
            SmartDashboard.putString("Color 1", color1.toHexString());
        } else {
            SmartDashboard.putString("Color 1", "null");
        }
        if(color2 != null){
            SmartDashboard.putString("Color 2", color2.toHexString());
        } else {
            SmartDashboard.putString("Color 2", "null");
        }
    }

}
