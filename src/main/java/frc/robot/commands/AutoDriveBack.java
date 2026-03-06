// This is for the autonomous to drive backwards for a certain amount of time
// PlatyPirates team 9181 - written by Barbara

package frc.robot.commands;

// imports and such
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveSubsystem;

// class declaration
public class AutoDriveBack extends Command {

    // instance variables
    private final DriveSubsystem m_drive;
    private final Timer m_timer = new Timer();
    // sets the constant to the amount of time to drive backwards will apply to everything under the class
    private static final double DRIVE_TIME = 2.0;

    public AutoDriveBack(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(drive); 
    }

    // mehtods
     @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();

    }

    @Override
    public void execute() {
        m_drive.drive(-0.4, 0.0, 0.0, true);
    }

    @Override
    public boolean isFinished() {
        return true; 
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setX();  // this is what locks the wheels so the robot doesn't slide

    }

    
}
