package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MAXSwerveModule;
import frc.robot.subsystems.DriveSubsystem;

import java.io.IOException;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveRobotFromLimelight extends Command {

    private DriveSubsystem _DriveSubsystem;
    public static int aprilTagId = 0;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private Pose3d aprilTagPose;

    public DriveRobotFromLimelight(DriveSubsystem DriveSubsystem) {
        _DriveSubsystem = DriveSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(LimelightHelpers.getTV("limelight")){
            aprilTagId = Math.toIntExact(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0));
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            aprilTagPose = aprilTagFieldLayout.getTagPose(aprilTagId).get();
        }
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(aprilTagId > 0){
           _DriveSubsystem.driveInSegments(aprilTagPose);
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
        System.out.println("FINISHED");
        return _DriveSubsystem.angleAligned();
    }
}