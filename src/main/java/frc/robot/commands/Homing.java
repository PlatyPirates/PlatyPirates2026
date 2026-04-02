//this is the command that tells the robot what to do while homing. It sets all the limit switches
//PlatyPirates team 9181 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

public class Homing extends Command {

    //fields
    private final Hood hood;
    private final Turret turret;

    //constructor
    public Homing(Hood hood, Turret turret) {
        this.hood = hood;
        this.turret = turret;
        addRequirements(hood, turret);

    }
    
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

    }

    @Override
    public boolean isFinished() {
        return hood.atLimit() && turret.atLimit();

    }

    @Override 
    public void end(boolean interrupted) {
        hood.resetEncoder();
        turret.resetEncoder();

    }




}