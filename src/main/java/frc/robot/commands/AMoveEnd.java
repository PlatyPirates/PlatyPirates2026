
package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;

public class AMoveEnd extends Command {

    private DriveSubsystem _drive;
    private Intake _intake;
    private double startTime;

    public AMoveEnd(DriveSubsystem drive, Intake intake) {
        _drive = drive;
        _intake = intake;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize(){
        startTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        _drive.drive(0.5, 0.0, 0.0, true);
        _intake.setSpeed(1.0);
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _drive.setX();
        _intake.setSpeed(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp()-startTime>=0.9);
    }

}
