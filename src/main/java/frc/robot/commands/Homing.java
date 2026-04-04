//this is the command that tells the robot what to do while homing. It sets all the limit switches
//PlatyPirates team 9181 
//TODO commented out during week5 comp because was moving weird maybe??
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Climber;

public class Homing extends Command {

    //fields
    private final Hood hood;
    private final Turret turret;
    private final Climber climber;
    

    //constructor
    public Homing(Hood hood, Turret turret, Climber climber) {
        this.hood = hood;
        this.turret = turret;
        this.climber = climber;
        addRequirements(hood, turret, climber);
    }

    //this is the part where we actually tell the motors what to do we call back to the subsystems here

    //execute runs every 20ms
    @Override
    public void execute() {
        if (hood.notAtLimit()) {
            hood.homeHood();
        } else {
            hood.stopHood();
        }

        if (turret.notAtLimit()) {
            turret.homeTurret();    
        } else {
            turret.stopTurret();
        }

        if (climber.notAtLimit()) {
            climber.homeClimber();
        } else {
            climber.stopMotor();
        }

    }
    
    //is finished is a boolean in this case it runs when they hit their limit switches
    @Override
    public boolean isFinished() {
        return hood.atLimit() && turret.atLimit() && climber.atLimit();

    }
    
    //ennds the whole thing and sets the encoders for the soft limits
    @Override 
    public void end(boolean interrupted) {
        hood.resetEncoder();
        turret.resetEncoder();
        climber.resetEncoder();

    }




}
    