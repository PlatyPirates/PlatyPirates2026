package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.DriveConstants;

public class Intake extends SubsystemBase {
    private final SparkMax m_intakeSparkMax;
    public Intake() {
        m_intakeSparkMax = new SparkMax(DriveConstants.kIntakeCanId, MotorType.kBrushless);
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
}
