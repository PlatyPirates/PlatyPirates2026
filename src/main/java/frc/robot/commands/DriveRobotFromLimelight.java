package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.MAXSwerveModule;
import frc.robot.subsystems.DriveSubsystem;

import java.io.IOException;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveRobotFromLimelight extends Command {

    private DriveSubsystem _DriveSubsystem;
    private LEDs _LEDs;
    public static int aprilTagId = 0;
    private static Pose3d aprilTagPose;
    private static double aprilTagAngle;
    private static double xOffset = -0.58;
    private static double yOffset = 0.0; //0.46
    private static double xOffsetMod, yOffsetMod;
    public static AprilTagFieldLayout aprilTagFieldLayout;
    public static boolean pink = false;

    enum State {
        ALIGN_ANGLE,
        ALIGN_TRANSLATION,
        FINISHED
    }

    private static State state = State.ALIGN_ANGLE;

    public DriveRobotFromLimelight(DriveSubsystem DriveSubsystem, LEDs underglow) {
        _DriveSubsystem = DriveSubsystem;
        _LEDs = underglow;
        addRequirements(DriveSubsystem, underglow);
    }

    public void alignWithAprilTag(){
        boolean angleAligned = _DriveSubsystem.angleAligned(aprilTagAngle);
        boolean translationAligned = _DriveSubsystem.translationAligned(aprilTagPose, xOffsetMod, yOffsetMod);
        SmartDashboard.putBoolean("Aligned", angleAligned && translationAligned && pink);

        pink = !pink;

        switch(state){
            case ALIGN_ANGLE:
                _LEDs.red();
                _DriveSubsystem.turnToHeading(aprilTagAngle);

                if(angleAligned){
                    if(translationAligned){
                        state = State.FINISHED;
                    } else {
                        state = State.ALIGN_TRANSLATION;
                    }
                }
                break;
            case ALIGN_TRANSLATION:
                _LEDs.purple();
                _DriveSubsystem.moveToCoordinates(aprilTagPose.getX() + xOffsetMod, aprilTagPose.getY() + yOffsetMod);

                if(translationAligned){
                    state = State.ALIGN_ANGLE;
                }
                break;
            case FINISHED:
                _LEDs.green();
                //_DriveSubsystem.drive(0.075, 0, 0, false);
                break;
        }
    }

    public static void alignLeft(){
        double yTemp = yOffset + 0.21;
        xOffsetMod = xOffset*Math.cos(Math.toRadians(aprilTagAngle))-yTemp*Math.sin(Math.toRadians(aprilTagAngle));
        yOffsetMod = xOffset*Math.sin(Math.toRadians(aprilTagAngle))+yTemp*Math.cos(Math.toRadians(aprilTagAngle));

    }

    public static void alignMiddle(){
        double yTemp = yOffset + 0.01;
        xOffsetMod = xOffset*Math.cos(Math.toRadians(aprilTagAngle))-yTemp*Math.sin(Math.toRadians(aprilTagAngle));
        yOffsetMod = xOffset*Math.sin(Math.toRadians(aprilTagAngle))+yTemp*Math.cos(Math.toRadians(aprilTagAngle));
    }

    public static void alignRight(){
        double yTemp = yOffset - 0.13;
        xOffsetMod = xOffset*Math.cos(Math.toRadians(aprilTagAngle))-yTemp*Math.sin(Math.toRadians(aprilTagAngle));
        yOffsetMod = xOffset*Math.sin(Math.toRadians(aprilTagAngle))+yTemp*Math.cos(Math.toRadians(aprilTagAngle));
    }

    public boolean done(){
        return state == State.FINISHED;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(LimelightHelpers.getTV("limelight")){
            aprilTagId = Math.toIntExact(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0));
            // aprilTagPose = aprilTagFieldLayout.getTagPose(aprilTagId).get();

            // double w = Math.toDegrees(2*Math.asin(aprilTagPose.getRotation().getQuaternion().getW()));
            // double z = Math.toDegrees(2*Math.asin(aprilTagPose.getRotation().getQuaternion().getZ()));

            // if(DriverStation.getAlliance().toString().equals("Red")){
            //     aprilTagAngle = -z*(w/-w);
            // } else {
            //     aprilTagAngle = -w + 180;
            // }

            aprilTagAngle = getAprilTagAngle(aprilTagId);
            aprilTagPose = aprilTagFieldLayout.getTagPose(aprilTagId).get();

            System.out.println(aprilTagAngle + " " + aprilTagPose.getTranslation().getX() + " " + aprilTagPose.getTranslation().getY());
            xOffsetMod = xOffset*Math.cos(Math.toRadians(aprilTagAngle))-yOffset*Math.sin(Math.toRadians(aprilTagAngle));
            yOffsetMod = xOffset*Math.sin(Math.toRadians(aprilTagAngle))+yOffset*Math.cos(Math.toRadians(aprilTagAngle));
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
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _DriveSubsystem.drive(0.0, 0.0, 0.0, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public int getAprilTagAngle(int id){
        switch(id){
            case 1: return -54;
            case 2: return 54;
            case 3: return 90;
            case 4: return 180;
            case 5: return 180;
            case 6: return 120;
            case 7: return 180;
            case 8: return -120;
            case 9: return -60;
            case 10: return 0;
            case 11: return 60;

            case 12: return -126;
            case 13: return 126;
            case 14: return 0;
            case 15: return 0;
            case 16: return -90;
            case 17: return 60;
            case 18: return 0;
            case 19: return -60;
            case 20: return -120;
            case 21: return 180;
            case 22: return 120;
        }
        return -1;
    }
}