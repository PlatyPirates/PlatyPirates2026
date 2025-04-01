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
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;

public class AMoveL4 {

    private DriveSubsystem _drive;
    private Shooter _shooter;
    private LEDs _underglow;
    private Elevator _elevator;
    private DriveRobotFromLimelight driveRobotFromLimelight;

    public AMoveL4(DriveSubsystem drive, Shooter shooter, LEDs underglow, Elevator elevator) {
        _drive = drive;
        _shooter = shooter;
        _underglow = underglow;
        _elevator = elevator;
        // Use addRequirements() here to declare subsystem dependencies.
        driveRobotFromLimelight = new DriveRobotFromLimelight(_drive, _underglow);
    }

    public Command driveForward(){
        return new RunCommand(() -> {
            _drive.drive(0.2, 0.0, 0, false);
            DriveRobotFromLimelight.alignLeft();
        }, _drive);
    }

    public Command alignAndRaiseElevator(){
        return new RunCommand(() -> {_elevator.l4();}, _elevator).deadlineWith(driveRobotFromLimelight);
    }

    public Command score(){
        return new RunCommand(() -> {
            _shooter.setSpeed(-0.5);
        }, _shooter);
    }

    public Command moveBackAndLowerElevator(){
        return new RunCommand(() -> {
            _elevator.l1();
            _drive.drive(-0.2, 0.0, 0.0, false);
        }, _elevator, _drive);
    }

    public Command moveAndL4(){
        return driveForward().withTimeout(1.0)
                .andThen(alignAndRaiseElevator())
                .andThen(score()).withTimeout(1.0)
                .andThen(moveBackAndLowerElevator()).withTimeout(1.0);
    }
    
}


