package frc.robot.commands;
import java.util.function.BooleanSupplier;

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
        return (new RunCommand(() -> {
            _elevator.l4();
            DriveRobotFromLimelight.alignLeft();
        }, _elevator)).deadlineWith(driveRobotFromLimelight).repeatedly();
    }

    public Command score(){
        return new RunCommand(() -> {
            _shooter.setSpeed(-0.5);
        }, _shooter);
    }

    public Command moveBack(){
        return new RunCommand(() -> {
            _shooter.setSpeed(0.0);
            _drive.drive(-0.2, 0.0, 0.0, false);
        }, _drive);
    }

    public Command lowerElevator(){
        return new RunCommand(() -> {
            _drive.drive(0.0, 0.0, 0.0, false);
            _elevator.l1();
        }, _elevator).repeatedly();
    }


    public Command moveAndL4(){
        Command cmd = driveForward().withTimeout(1.0).andThen(
                alignAndRaiseElevator().withTimeout(5.0)).andThen(
                score().repeatedly().withTimeout(1.0)).andThen(
                moveBack().withTimeout(0.75)).andThen(
                lowerElevator())
                ;
        cmd.addRequirements(_elevator, _drive, _underglow, _shooter);
        return cmd;
    }
    
}


