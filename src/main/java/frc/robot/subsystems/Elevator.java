package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkBase;

public class Elevator extends SubsystemBase{
    private final SparkMax m_leftSparkMax;
    private final SparkMax m_rightSparkMax;
    private final DigitalInput m_limitSwitch = new DigitalInput(9);
    private RelativeEncoder encoderLeft;
    private RelativeEncoder encoderRight;
    private final PWMSparkMax m_intakeActuator;
    private final double maxHeight = 84.0;

    private final double l1 = 0.0;
    private final double l2 = 29.57;
    private final double l3 = 50.0;
    private final double l4 = 84.0;
    
    private final PIDController pid;

    private final double kg = 0.0;

    public Elevator(){
        m_leftSparkMax = new SparkMax(Constants.DriveConstants.kLeftElevatorCanId, MotorType.kBrushless);
        m_rightSparkMax = new SparkMax(Constants.DriveConstants.kRightElevatorCanId, MotorType.kBrushless);
        encoderLeft = m_leftSparkMax.getEncoder();
        encoderRight = m_rightSparkMax.getEncoder(); 
        m_intakeActuator = new PWMSparkMax(Constants.DriveConstants.kLinearActuator);
        
        SparkBaseConfig configLeft = new SparkMaxConfig();
        SparkBaseConfig configRight = new SparkMaxConfig();

        configLeft.openLoopRampRate(2);
        configRight.openLoopRampRate(2);

        m_leftSparkMax.configure(configLeft, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        m_rightSparkMax.configure(configRight, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        pid = new PIDController(0.1, 0.0, 0.0);
    }

    public void l2() {
        goToHeight(l2);
    }

    public void l4() {
        goToHeight(l4);
    }

    public void l3() {
        goToHeight(l3);
    }

    public void zero(){
        if(!m_limitSwitch.get()){
            encoderLeft.setPosition(0.0);
            encoderRight.setPosition(0.0);
        }
    }

    public void setActuatorSpeed(double speed){
        m_intakeActuator.set(speed);
    }

     public void setSpeed(double speed){
        if((maxHeight - getHeight() <= 10.0 && speed > 0) || (getHeight() <= 10.0 && speed < 0)){
            speed *= 0.5;
        }
        if((!isZero() || speed > 0) && (!(Math.abs(getHeight()) >= maxHeight) || speed < 0)){
            m_leftSparkMax.set(speed);
            m_rightSparkMax.set(speed);
        } else {
            m_leftSparkMax.set(0.0);
            m_rightSparkMax.set(0.0);
        }
     }

    public double getHeight(){
        return encoderLeft.getPosition();
        //return (encoderLeft.getPosition() + encoderRight.getPosition()) / 2.0;
    }

    public boolean fixZero(){
        if(!isZero()){
            setSpeed(-1.0);
            return false;
        } else {
            encoderLeft.setPosition(0.0);
            encoderRight.setPosition(0.0);
            return true;
        }
    }

    public void goToHeight(double height){
        double pidOutput = pid.calculate(getHeight(), height);

        setSpeed(Math.min(Math.max(pidOutput + kg, -1.0), 1.0));
    }

    public boolean isZero(){
        return !m_limitSwitch.get();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Height", getHeight());
        zero();
        //SmartDashboard.putBoolean("Limit Switch", isZero());
        //System.out.println(m_intakeActuator.getVoltage());
    }
}