package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SoftLimitConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Climber extends SubsystemBase {
    private final SparkMax m_climberMotorChainSparkMax;
    private final SparkMax m_climberMotorRopeSparkMax;

    // declare encoder
    private Encoder climberEncoderChain;

    public Climber(){
        m_climberMotorChainSparkMax = new SparkMax(DriveConstants.kClimberMotorChainCanId, MotorType.kBrushless);

        m_climberMotorRopeSparkMax = new SparkMax(DriveConstants.kClimberMotorRopeCanId, MotorType.kBrushless);

        SoftLimitConfig softLimit = new SoftLimitConfig();
        softLimit.forwardSoftLimitEnabled(false);
        softLimit.forwardSoftLimit(180.0);
        softLimit.reverseSoftLimitEnabled(false);
        softLimit.reverseSoftLimit(-90.0);
        SparkBaseConfig config = new SparkMaxConfig();

        config.apply(softLimit);
        config.openLoopRampRate(0.25); // takes 0.25s for the motor to go full throttle to reduce dynamic loads
        m_climberMotorChainSparkMax.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        config.apply(softLimit);
        config.openLoopRampRate(0.25); // takes 0.25s for the motor to go full throttle to reduce dynamic loads
        m_climberMotorRopeSparkMax.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // DIO ports are 0 and 1, should probably put these in constants
        climberEncoderChain = new Encoder(0,1);
    }
    public void setSpeed(double speedChain, double speedRope){
        m_climberMotorChainSparkMax.set(speedChain);
    }
}
