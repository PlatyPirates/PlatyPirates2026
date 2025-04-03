package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.DriveConstants;

public class Shooter extends SubsystemBase {
    private final SparkMax m_intakeSparkMax;
    private Rev2mDistanceSensor m_distanceSensor;
    private final double distanceTolerance = 4.0;

    public Shooter() {
        m_intakeSparkMax = new SparkMax(DriveConstants.kShooterCanId, MotorType.kBrushless);

        m_distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
        m_distanceSensor.setAutomaticMode(true);
    }
    public void setSpeed(double speed) {
        double currentLimit = 100.0;
        if(m_intakeSparkMax.getOutputCurrent() < currentLimit){
            m_intakeSparkMax.set(speed);
        }
    }

    public double getOutputCurrent(){
        return m_intakeSparkMax.getOutputCurrent();
    }

    public boolean distanceTriggered(){
        return m_distanceSensor.isRangeValid() && m_distanceSensor.getRange() < distanceTolerance;
    }
}
