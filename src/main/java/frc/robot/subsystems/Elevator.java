package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.math.BigDecimal;
import java.math.MathContext;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkBase;

public class Elevator extends SubsystemBase{
    private final SparkMax m_leftSparkMax;
    private final SparkMax m_rightSparkMax;
    private final DigitalInput m_lowerLimit = new DigitalInput(9);
    private final DigitalInput m_upperLimit = new DigitalInput(8);

    private RelativeEncoder encoderLeft;
    private RelativeEncoder encoderRight;

    private final double l1 = -0.1;
    private final double l2 = 29.57;
    private final double l3 = 50.0;
    private final double l4 = 79.0;
    private final double barge = 93.0;
    
    private PIDController pid;

    private final double kg = 0.0;

    public Elevator(){
        m_leftSparkMax = new SparkMax(Constants.DriveConstants.kLeftElevatorCanId, MotorType.kBrushless);
        m_rightSparkMax = new SparkMax(Constants.DriveConstants.kRightElevatorCanId, MotorType.kBrushless);
        encoderLeft = m_leftSparkMax.getEncoder();
        encoderRight = m_rightSparkMax.getEncoder(); 
        
        SparkBaseConfig configLeft = new SparkMaxConfig();
        SparkBaseConfig configRight = new SparkMaxConfig();

        //configLeft.openLoopRampRate(2);
        //configRight.openLoopRampRate(2);

        m_leftSparkMax.configure(configLeft, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        m_rightSparkMax.configure(configRight, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        pid = new PIDController(0.3, 0.016, 0.2);
        pid.setTolerance(0.03);
    }

    /*
     * Create PID controller for elevator
     */
    public void setPID(double p, double i, double d){
        System.out.println("Setting values: P = " + p + ", I = " + i + ", D = " + d);
        pid = new PIDController(p, i, d);
        pid.setTolerance(0.03);
        pid.setIZone(2.0);
    }

    /*
     * Drive elevator motors to go to L1 height
     */
    public void l1(){
        goToHeight(l1);
    }

    /*
     * Drive elevator motors to go to L2 height
     */
    public void l2() {
        goToHeight(l2);
    }

    /*
     * Drive elevator motors to go to L3 height
     */
    public void l3() {
        goToHeight(l3);
    }

    /*
     * Drive elevator motors to go to L4 height
     */
    public void l4() {
        goToHeight(l4);
    }

    /**
     * Drive elevator motors to go to barge height
     */
    public void barge(){
        if(getHeight() <= l4){
            goToHeight(l4);
        } else {
            goToHeight(barge);
        }
    }

    /**
     * Sets the encoder reading to 0 if we hit the bottom limit switch. Called periodically
     */
    public void zero(){
        if(!m_lowerLimit.get() && !m_upperLimit.get()){
            encoderLeft.setPosition(0.0);
            encoderRight.setPosition(0.0);
        }
    }

    /**
     * Sets the elevator speed, only if we are not trying to go past a limit switch
     *
     * @param speed, a double between -1 and 1
     */
    public void setSpeed(double speed){
        if((!isZero() && speed < 0) || (!isMax() && speed > 0)){
            m_leftSparkMax.set(speed);
            m_rightSparkMax.set(speed);
        } else {
            m_leftSparkMax.set(0.0);
            m_rightSparkMax.set(0.0);
        }
     }

    /**
     * Get current elevator height. Only uses the left encoder because that one doesn't skip like the right
     * one sometimes does. Also, the right encoder returns a negative value.
     *
     * @return height in motor rotations
     */
    public double getHeight(){
        return encoderLeft.getPosition();
        //return (encoderLeft.getPosition() + encoderRight.getPosition()) / 2.0;
    }

    /**
     * Set motor speed according to PID algo for movement to specified height. Only used internally
     *
     * @return height, in motor rotations
     */
    private void goToHeight(double height){
        double pidOutput = pid.calculate(getHeight(), height);

        setSpeed(Math.min(Math.max(pidOutput + kg, -1.0), 1.0));
    }

    /**
     * Checks if the elevator is contacting the lower limit switch
     *
     * @return true, if the lower limit switch is contacted
     */
    private boolean isZero(){
        return !m_lowerLimit.get();
    }

    /**
     * Checks if the elevator is contacting the upper limit switch
     *
     * @return true, if the upper limit switch is contacted
     */
    private boolean isMax(){
        return !m_upperLimit.get();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Height", getHeight());
        zero();
        SmartDashboard.putString("PID VALUES!!", pid.getP() + " " + pid.getI() + " " + pid.getD() + " " + new BigDecimal(pid.getError()).round(new MathContext(3)) + " " + new BigDecimal(pid.getAccumulatedError()).round(new MathContext(3)));
        SmartDashboard.putBoolean("Bottom Limit", !isZero());
        SmartDashboard.putBoolean("Upper Limit", !isMax());
        //SmartDashboard.putBoolean("Limit Switch", isZero());
        //System.out.println(m_intakeActuator.getVoltage());
    }
}