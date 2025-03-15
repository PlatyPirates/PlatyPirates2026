package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class LoadCoral extends Command{
    //private Intake _Intake
    private Shooter _Shooter;
    private LEDs _LEDs;

    enum State {
        INTAKING,
        LOADING,
        FINISHED
    }

    private static State state = State.INTAKING;

    public LoadCoral(Shooter shooter, LEDs leds){
        _Shooter = shooter;
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
                //run the intake
                _LEDs.setSolid(255, 255, 0);
                if(_Shooter.distanceTriggered()){
                    state = State.LOADING;
                }
                break;
            case LOADING:
                //run the shooter
                _LEDs.setSolid(255, 0, 255);
                if(!_Shooter.distanceTriggered()){
                    state = State.FINISHED;
                }
                break;
            case FINISHED:
                _LEDs.setSolid(0, 255, 0);
                break;
        }
    }

}
