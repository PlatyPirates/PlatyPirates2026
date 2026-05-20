//this is the command used to climb during autonomous. fyi it doesn't actually work super well. It's unfinished 
// PlatyPirates team 9181

package frc.robot.commands;

//imports
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;


public class AutoClimb extends Command {

    //fields
    private final Climber climber;
    private final DriveSubsystem drive;
    private final Timer timer = new Timer();

    //the things in bold are called states they are a method of organizing the different parts of the auto
    //what I want it to do is extend the arm first then drive to the right location then retract the arm
    enum State {
        EXTENDING,
        DRIVING,
        RETRACTING,
        FINISHED

    }

    //TODO figure this one out
    private State currentState = State.EXTENDING;

    //constructor - tells the code that these things exist so it can call back to this later
    public AutoClimb(Climber climber, DriveSubsystem drive) {
        this.climber = climber;
        this.drive = drive;
        addRequirements(climber, drive);

    }
// commands usually start with intialize and in this case it both resets and starts a timer and sets the state to extending
@Override
public void initialize() {
    timer.reset();
    timer.start();
    currentState = State.EXTENDING;

}
// next we need to execute
@Override
public void execute() {
    //pretty intuitive this switches the state
    switch (currentState) {
        //switches it to EXTENDING
        case EXTENDING:
            //this is what EXTENDING actually does
            //moves the climber at a speed set in constants is very slow
            climber.moveClimber(Constants.SubsystemConstants.kMoveClimber);
            // if the climber hits a position set by the encoder (theoretically its max height) 
            if (climber.getEncoderPosition() >= Constants.SubsystemConstants.kClimberMax) {
                // then the current state changes to DRIVING and the timer gets reset and restarted again
                currentState = State.DRIVING;
                timer.reset();
                timer.start();
                
            }
            //once it hits the max position it breaks off into this next part
            break;
        //switches over to DRIVING
        case DRIVING:
            //TODO figure out how driving actually works
            drive.drive(Constants.SubsystemConstants.kClimbDriveSpeed, 0.0, 0.0, false);
            //if the timer hits a certain number that i set in constants... and this is why it isn't working lol
            //idk how but I need to include things from pathplanner here mb
            //also its occuring to me that the timer is basically doing nothing lol.
            if (timer.get() >= Constants.SubsystemConstants.kClimbDriveTime) {
                currentState = State.RETRACTING;
                timer.reset();
                timer.start();
                
            }
            //if the requirements above are fulfilled it breaks off and switches to RETRACTING
            break;
        //this does the same thing as EXTENDING except it retracts the arm instead of extending it
        case RETRACTING:
            climber.moveClimber(Constants.SubsystemConstants.kReverseClimber);
            if (climber.getEncoderPosition() <= Constants.SubsystemConstants.kClimberMin) {
                currentState = State.FINISHED;
                timer.reset();
                timer.start();
                
            }
            
            break;
        case FINISHED:
            //this is to make sure everyhting is completely stopped and won't hurt itself once we are done climbing
            climber.moveClimber(0);
            drive.drive(0, 0, 0, false);

            break;
    }


}
//this is the next stage in most commands 
//NOTE: isFinished has to be a boolean (i think) - you can't just say 'public void' at least
@Override 
public boolean isFinished() {
    return currentState == State.FINISHED;

}
//end is just another safety measure to make sure everything is safely stopped.
@Override
public void end(boolean interrupted) {
    climber.moveClimber(0);
    drive.drive(0, 0, 0, false);

}

     
}