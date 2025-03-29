package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Shooter;

public class MotorFromLimelight extends Command {

    private Shooter _intake;

    public MotorFromLimelight(Shooter intake) {
        _intake = intake;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {    
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV("");
        if (hasTarget) {
            double new_speed = LimelightHelpers.getTX("") * 0.03;
            _intake.setSpeed(new_speed);
        }
        else {
            _intake.setSpeed(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _intake.setSpeed(0);
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}