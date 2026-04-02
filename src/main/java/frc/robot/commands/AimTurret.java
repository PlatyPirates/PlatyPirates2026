// this is the code needed to align the turret to an apirl tag in order to shoot
// PlatyPirates team 9181

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Turret;

public class AimTurret extends Command {

    //fields
    private final Turret turret;

    //constructor
    public AimTurret(Turret turret) {
        this.turret = turret;
        addRequirements(turret);

    }

//now we tell what the turret to do 
@Override 
public void execute() {
    if(LimelightHelpers.getTV("limelight")) {
        
        double tx = LimelightHelpers.getTX("limelight");
        double speed = tx * Constants.SubsystemConstants.kTurretP;
        turret.moveTurret(speed);

    } else {

        turret.stopTurret();
    }

}

@Override 
public boolean isFinished() {
    return false;

}

@Override 
public void end(boolean interrupted) {

    turret.stopTurret();

}
    
}
