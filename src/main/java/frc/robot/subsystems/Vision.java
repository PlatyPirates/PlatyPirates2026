/*This is the vision code 
The goal is to track where the limelight is relative to the robot
If this is accurate, it can also help track the robot better on elastic
*/ 
package frc.robot.subsystems;
import edu.wpi.first.math.VecBuilder;
// imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.LimelightHelpers.RawFiducial;


public class Vision extends SubsystemBase {

    public Vision(){
        LimelightHelpers.setCameraPose_RobotSpace("limelight",
         .297, -0.289, 0.329, 0, 30, 0);
    }
    @Override
    public void periodic(){
// This gives the distance to a tag, which can be used to pick the hood angle.


// Get raw AprilTag/Fiducial data
RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight");
for (RawFiducial fiducial : fiducials) {
    int id = fiducial.id;                    // Tag ID
    double txnc = fiducial.txnc;             // X offset (no crosshair)
    double tync = fiducial.tync;             // Y offset (no crosshair)
    double ta = fiducial.ta;                 // Target area
    double distToCamera = fiducial.distToCamera;  // Distance to camera
    double distToRobot = fiducial.distToRobot;    // Distance to robot
    double ambiguity = fiducial.ambiguity;   // Tag pose ambiguity
}

// Get raw neural detector results
RawDetection[] detections = LimelightHelpers.getRawDetections("limelight");
for (RawDetection detection : detections) {
    int classID = detection.classId;
    double txnc = detection.txnc;
    double tync = detection.tync;
    double ta = detection.ta;
    // Access corner coordinates if needed
    double corner0X = detection.corner0_X;
    double corner0Y = detection.corner0_Y;
    // ... corners 1-3 available similarly
}

    }



}


    


    
