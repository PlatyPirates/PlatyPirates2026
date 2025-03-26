package frc.robot.commands;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;

public class AMoveL4 extends Command {

    private DriveSubsystem _drive;
    private Shooter _intake;
    private LEDs _underglow;
    private double startTime;
    private double currentTime;
    private double stateStartTime;
    private double elapsedStateTime;
    private boolean end;

    enum State {
        DRIVE_FORWARD,
        SCORE,
        END
    }

    private static State state = State.DRIVE_FORWARD;


    public AMoveL4(DriveSubsystem drive, Shooter intake, LEDs underglow) {
        _drive = drive;
        _intake = intake;
        _underglow = underglow;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive, intake);
    }

    public void changeState(State inputState){
        stateStartTime = Timer.getFPGATimestamp();
        state = inputState;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize(){
        stateStartTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        currentTime = Timer.getFPGATimestamp();
        elapsedStateTime = currentTime - stateStartTime;

        switch(state){
            case DRIVE_FORWARD:
                _drive.drive(-0.5, 0.0, 0.05, true);

                if(elapsedStateTime >= 2.0){
                    changeState(State.SCORE);
                }
                break;
            case SCORE:
                new DriveRobotFromLimelight(_drive, _underglow);

                state = State.END;
                break;
            case END:
                end = true;;
                break;

        }    
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
        return end;
    }

}


