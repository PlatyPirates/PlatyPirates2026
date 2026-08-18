package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//This code only identifies April tags-It can't connect to other subsystem yet 
public class Orientation extends SubsystemBase {

    // This is your "camera": the Limelight network table
    // Imports not needed for camera??
    private final NetworkTable limelight;

    public Orientation() {
        // Connect to the Limelight using its table name ("limelight" by default)
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic() {
        // Read AprilTag ID (-1 means no tag detected)
        double id = limelight.getEntry("tid").getDouble(-1);

        // Horizontal offset from crosshair to target (degrees)
        double tx = limelight.getEntry("tx").getDouble(0);

        // Vertical offset from crosshair to target (degrees)
        double ty = limelight.getEntry("ty").getDouble(0);

        // Target area (how big the tag appears)
        double ta = limelight.getEntry("ta").getDouble(0);

        // Only log if a tag is actually detected
        if (id != -1) {
            System.out.println("=== AprilTag Detected ===");
            System.out.println("ID: " + id);
            System.out.println("TX: " + tx);
            System.out.println("TY: " + ty);
            System.out.println("TA: " + ta);
        }
    }
}