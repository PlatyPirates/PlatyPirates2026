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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;

public class AMoveL4 {

    private DriveSubsystem _drive;
    private Shooter _shooter;
    private LEDs _underglow;
    private Elevator _elevator;
    private Intake _intake;
    private DriveRobotFromLimelight driveRobotFromLimelight;

    private static double rotationSpeed = 0.0;
    private static double maxRot = 0.1;


    public AMoveL4(DriveSubsystem drive, Shooter shooter, LEDs underglow, Elevator elevator, Intake intake) {
        _drive = drive;
        _shooter = shooter;
        _underglow = underglow;
        _elevator = elevator;
        _intake = intake;
        // Use addRequirements() here to declare subsystem dependencies.
        driveRobotFromLimelight = new DriveRobotFromLimelight(_drive, _underglow);
    }

    public Command driveForward(){
        return new RunCommand(() -> {
            _drive.drive(0.2, 0.0, rotationSpeed, false);
            DriveRobotFromLimelight.alignLeft();
            _intake.setSpeed(1.0);
        }, _drive, _intake);
    }

    public Command alignAndRaiseElevator(){
        return (new RunCommand(() -> {
            _elevator.l4();
            _intake.setSpeed(0.0);
            DriveRobotFromLimelight.alignLeft();
        }, _elevator, _intake)).deadlineWith(driveRobotFromLimelight).repeatedly();
    }

    public Command score(){
        return new RunCommand(() -> {
            _shooter.setSpeed(-0.5);
            _elevator.stop();
        }, _shooter);
    }

    public Command moveBack(){
        return new RunCommand(() -> {
            _shooter.setSpeed(0.0);
            _elevator.stop();
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
        cmd.addRequirements(_elevator, _drive, _underglow, _shooter, _intake);
        return cmd;
    }

    public static void leftTag(){
        rotationSpeed = -maxRot;
    }

    public static void rightTag(){
        rotationSpeed = maxRot;
    }

    public static void centerTag(){
        rotationSpeed = 0.0;
    }
    
}


