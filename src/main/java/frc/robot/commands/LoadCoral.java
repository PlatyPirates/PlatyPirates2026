package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class LoadCoral extends Command{
    //private Intake _Intake
    private Shooter _Shooter;
    private Intake _Intake;
    private LEDs _LEDs;

    enum State {
        INTAKING,
        LOADING,
        FINISHED
    }

    private static State state = State.INTAKING;

    public LoadCoral(Shooter shooter, LEDs leds, Intake intake){
        _Shooter = shooter;
        _Intake = intake;
        _LEDs = leds;
        addRequirements(shooter);
        addRequirements(leds);
    }

    @Override
    public void initialize(){
        state = State.INTAKING;
    }

    @Override
    public void execute(){
        switch(state){
            case INTAKING:
                _LEDs.setSolid(255, 255, 0);
                if(_Shooter.distanceTriggered()){
                    state = State.LOADING;
                }
                break;
            case LOADING:
                _Shooter.setSpeed(-0.1);
                _Intake.setSpeed(1.0);
                _LEDs.setSolid(255, 0, 255);
                if(!_Shooter.distanceTriggered()){
                    _Shooter.setSpeed(0.0);
                    state = State.FINISHED;
                }
                break;
            case FINISHED:
                _LEDs.setSolid(0, 255, 0);
                break;
        }
    }

}
