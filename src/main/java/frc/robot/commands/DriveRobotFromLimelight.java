package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MAXSwerveModule;
import frc.robot.subsystems.DriveSubsystem;

import java.io.IOException;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveRobotFromLimelight extends Command {

    private DriveSubsystem _DriveSubsystem;
    public static int aprilTagId = 0;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private Pose3d aprilTagPose;
    private double aprilTagAngle;
    private double xOffset = 0.4;
    private double yOffset = 0.0;

    enum State {
        ALIGN_ANGLE,
        ALIGN_TRANSLATION,
        FINISHED
    }

    private static State state = State.ALIGN_ANGLE;

    public DriveRobotFromLimelight(DriveSubsystem DriveSubsystem) {
        _DriveSubsystem = DriveSubsystem;
        addRequirements(DriveSubsystem);
    }

    public void alignWithAprilTag(){
        switch(state){
            case ALIGN_ANGLE:
                Robot.setLEDs(255, 0, 0);
                _DriveSubsystem.turnToHeading(aprilTagAngle);

                if(_DriveSubsystem.angleAligned(aprilTagAngle)){
                    if(_DriveSubsystem.translationAligned(aprilTagPose, xOffset, yOffset)){
                        state = State.FINISHED;
                    } else {
                        state = State.ALIGN_TRANSLATION;
                    }
                }
                break;
            case ALIGN_TRANSLATION:
                Robot.setLEDs(255, 0, 255);
                _DriveSubsystem.moveToCoordinates(aprilTagPose.getX() + xOffset, aprilTagPose.getY() + yOffset);

                if(_DriveSubsystem.translationAligned(aprilTagPose, xOffset, yOffset)){
                    state = State.ALIGN_ANGLE;
                }
                break;
            case FINISHED:
                Robot.setLEDs(0, 255, 0);
                break;
        }
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(LimelightHelpers.getTV("limelight")){
            aprilTagId = Math.toIntExact(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0));
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            aprilTagPose = aprilTagFieldLayout.getTagPose(aprilTagId).get();

            double w = Math.toDegrees(2*Math.asin(aprilTagPose.getRotation().getQuaternion().getW()));
            double z = Math.toDegrees(2*Math.asin(aprilTagPose.getRotation().getQuaternion().getZ()));
    
            if(DriverStation.getAlliance().toString().equals("Red")){
                aprilTagAngle = -z*(w/-w);
            } else {
                aprilTagAngle = -w;
            }
        }

        state = State.ALIGN_ANGLE;
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(aprilTagId > 0){
           alignWithAprilTag();
        } else {
            System.err.println("No AprilTag detected.");
        }
        // double kP = .035;
        // double targetingAngularVelocity = LimelightHelpers.getTX("")*kP;
        // targetingAngularVelocity *= -1.0*DriveConstants.kMaxAngularSpeed;

        // boolean hasTarget = LimelightHelpers.getTV("");
        // if (hasTarget && (LimelightHelpers.getTX("") != 0)) {
        //      _DriveSubsystem.drive(0.03*LimelightHelpers.getTX(""), 0, targetingAngularVelocity, true);
        // } else if(hasTarget && (LimelightHelpers.getTX("") == 0)){
        //     _DriveSubsystem.drive(0, 0, 0, true);
        // }
    }
       
    

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _DriveSubsystem.drive(0.0, 0.0, 0.0, true);
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return state == State.FINISHED;
    }
}