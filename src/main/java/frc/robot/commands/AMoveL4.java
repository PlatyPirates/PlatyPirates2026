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
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;

public class AMoveL4 extends Command {

    private DriveSubsystem _drive;
    private Shooter _shooter;
    private LEDs _underglow;
    private Elevator _elevator;
    private double startTime;
    private double currentTime;
    private double stateStartTime;
    private double elapsedStateTime;
    private boolean end;
    private DriveRobotFromLimelight driveRobotFromLimelight;

    enum State {
        DRIVE_FORWARD,
        ALIGN,
        SCORE,
        MOVE_BACK,
        END
    }

    private static State state = State.DRIVE_FORWARD;


    public AMoveL4(DriveSubsystem drive, Shooter shooter, LEDs underglow, Elevator elevator) {
        _drive = drive;
        _shooter = shooter;
        _underglow = underglow;
        _elevator = elevator;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive, shooter, underglow);
        driveRobotFromLimelight = new DriveRobotFromLimelight(_drive, _underglow);
    }

    public void changeState(State inputState){
        stateStartTime = Timer.getFPGATimestamp();
        state = inputState;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize(){
        DriveRobotFromLimelight.alignLeft();
        stateStartTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        currentTime = Timer.getFPGATimestamp();
        elapsedStateTime = currentTime - stateStartTime;

        switch(state){
            case DRIVE_FORWARD:
                _drive.drive(0.2, 0.0, 0, false);

                if(elapsedStateTime >= 1.0){
                    _drive.drive(0, 0, 0, false);
                    changeState(State.ALIGN);
                    driveRobotFromLimelight.schedule();
                }
                break;
            case ALIGN:
                _elevator.l4();

                if(driveRobotFromLimelight.done()){
                    driveRobotFromLimelight.end(true);
                    changeState(State.SCORE);
                }

                break;
            case SCORE:
                _shooter.setSpeed(-0.5);

                if(elapsedStateTime >= 1.0){
                    changeState(State.MOVE_BACK);
                }
                break;
            case MOVE_BACK:
                _elevator.l1();
                _drive.drive(0.2, 0.0, 0.0, false);
                
                if(elapsedStateTime >= 1.0){
                    changeState(State.END);
                }
                break;
            case END:
                end = true;
                break;

        }    
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _drive.setX();
        _shooter.setSpeed(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return end;
    }

}


