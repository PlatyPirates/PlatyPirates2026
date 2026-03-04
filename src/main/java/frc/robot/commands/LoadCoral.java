/* package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class LoadCoral extends Command{
    //private Intake _Intake
    private Shooter _Shooter;
    private Intake _Intake;
    private LEDs _LEDs;
    private Elevator _elevator;

    enum State {
        INTAKING,
        LOADING,
        FINISHED
    }

    private static State state = State.INTAKING;

    public LoadCoral(Shooter shooter, LEDs leds, Intake intake, Elevator elevator){
        _Shooter = shooter;
        _Intake = intake;
        _LEDs = leds;
        _elevator = elevator;
        addRequirements(shooter, leds, elevator, intake);
    }

    @Override
    public void initialize(){
        state = State.INTAKING;
    }

    @Override
    public void execute(){
        switch(state){
            case INTAKING:
                _LEDs.yellow();
                _Intake.setSpeed(1.0);
                _elevator.setSpeed(-0.4);
                if(_Shooter.distanceTriggered()){
                    state = State.LOADING;
                }
                break;
            case LOADING:
                _Shooter.setSpeed(-0.1);
                _Intake.setSpeed(1.0);
                _LEDs.purple();
                if(!_Shooter.distanceTriggered()){
                    _Shooter.setSpeed(0.0);
                    state = State.FINISHED;
                }
                break;
            case FINISHED:
                _LEDs.green();
                break;
        }
    }

}
 */