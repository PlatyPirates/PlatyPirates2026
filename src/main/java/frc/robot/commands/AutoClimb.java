//this is the command used to climb during autonomous. 
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

    enum State {
        EXTENDING,
        DRIVING,
        RETRACTING,
        FINISHED

    }

    private State currentState = State.EXTENDING;

    //constructor
    public AutoClimb(Climber climber, DriveSubsystem drive) {
        this.climber = climber;
        this.drive = drive;
        addRequirements(climber, drive);

    }

@Override
public void initialize() {
    timer.reset();
    timer.start();
    currentState = State.EXTENDING;

}

@Override
public void execute() {
    switch (currentState) {
        case EXTENDING:
            climber.moveClimber(Constants.SubsystemConstants.kMoveClimber);
            if (climber.getEncoderPosition() >= Constants.SubsystemConstants.kClimberMax) {
                currentState = State.DRIVING;
                timer.reset();
                timer.start();
                
            }
            
            break;
        case DRIVING:
            drive.drive(Constants.SubsystemConstants.kClimbDriveSpeed, 0.0, 0.0, false);
            if (timer.get() >= Constants.SubsystemConstants.kClimbDriveTime) {
                currentState = State.RETRACTING;
                timer.reset();
                timer.start();
                
            }

            break;
        case RETRACTING:
            climber.moveClimber(Constants.SubsystemConstants.kReverseClimber);
            if (climber.getEncoderPosition() <= Constants.SubsystemConstants.kClimberMin) {
                currentState = State.FINISHED;
                timer.reset();
                timer.start();
                
            }

            break;
        case FINISHED:
            climber.moveClimber(0);
            drive.drive(0, 0, 0, false);

            break;
    }


}

@Override 
public boolean isFinished() {
    return currentState == State.FINISHED;

}

@Override
public void end(boolean interrupted) {
    climber.moveClimber(0);
    drive.drive(0, 0, 0, false);

}

     
}