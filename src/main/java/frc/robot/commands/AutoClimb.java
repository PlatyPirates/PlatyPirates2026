//this is the command used to climb during autonomous. 
// PlatyPirates team 9181

package frc.robot.commands;

//imports
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Climber;

public class AutoClimb extends Command {

    //fields
    private final Climber climber;

    //constructor
    public AutoClimb(Climber climber) {
        this.climber = climber;
        addRequirements(climber);

    }

@Override
public void initialize() {

}

@Override
public void execute() {
    

}

@Override 
public boolean isFinished() {

}

@Override
public void end(boolean interrupted) {

}

     
}