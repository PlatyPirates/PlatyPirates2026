// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import java.lang.reflect.Field;
import java.util.ArrayList;

import com.ctre.phoenix6.hardware.Pigeon2;

public class DriveSubsystem extends SubsystemBase {
  Field2d field = new Field2d();
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  //private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kGyroCanId);
  private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
    Constants.DriveConstants.kDriveKinematics, 
    Rotation2d.fromDegrees(0),
    getModulePositions(), 
    new Pose2d(),
    VecBuilder.fill(1,1,1), //Robot state std devs
    VecBuilder.fill(.5,.5,10)); //Vision std devs

  private double angleTolerance = 0.5;

  // Odometry class for tracking robot pose
  // SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
  //     DriveConstants.kDriveKinematics,
  //     //Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
  //     Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
  //     new SwerveModulePosition[] {
  //         m_frontLeft.getPosition(),
  //         m_frontRight.getPosition(),
  //         m_rearLeft.getPosition(),
  //         m_rearRight.getPosition()
  //     });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SmartDashboard.putData(field);

    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  public void poseInit(){
    if(DriverStation.getAlliance().get() == Alliance.Blue){
      m_poseEstimator.resetRotation(Rotation2d.fromDegrees(180.0));
      setHeading(180.0);
    } else {
      m_poseEstimator.resetRotation(Rotation2d.fromDegrees(0.0));
      setHeading(0.0);
    }
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    // m_odometry.update(
    //     //Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
    //     Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
    //     new SwerveModulePosition[] {
    //         m_frontLeft.getPosition(),
    //         m_frontRight.getPosition(),
    //         m_rearLeft.getPosition(),
    //         m_rearRight.getPosition()
    //     });
    updateOdometry();

    SmartDashboard.putNumber("Pose Estimator: X Translation", m_poseEstimator.getEstimatedPosition().getTranslation().getX());
    SmartDashboard.putNumber("Pose Estimator: Y Translation", m_poseEstimator.getEstimatedPosition().getTranslation().getY());
    SmartDashboard.putNumber("Pose Estimator: Robot Heading", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());

    SmartDashboard.putBoolean("Front Left Steering", !m_frontLeft.steeringError());
    SmartDashboard.putBoolean("Front Right Steering", !m_frontRight.steeringError());
    SmartDashboard.putBoolean("Back Left Steering", !m_rearLeft.steeringError());
    SmartDashboard.putBoolean("Back Right Steering", !m_rearRight.steeringError());

    SmartDashboard.putBoolean("Front Left Driving", !m_frontLeft.drivingError());
    SmartDashboard.putBoolean("Front Right Driving", !m_frontRight.drivingError());
    SmartDashboard.putBoolean("Back Left Driving", !m_rearLeft.drivingError());
    SmartDashboard.putBoolean("Back Right Driving", !m_rearRight.drivingError());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void updateOdometry() { //From Limelight example code

    boolean doRejectUpdate = false;

    LimelightHelpers.SetRobotOrientation("limelight", getEstimatedHeading(), 0, 0, 0, 0, 0);
    
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    try{
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        m_poseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
  }
    } catch (NullPointerException e){}

      m_poseEstimator.update(
            Rotation2d.fromDegrees(getHeading()),
            new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
            });
        field.setRobotPose(getPose());
    
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
        //Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                m_poseEstimator.getEstimatedPosition().getRotation())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public boolean angleAligned(double aprilTagAngle) {
    return(Math.abs(getEstimatedHeading() - aprilTagAngle) < angleTolerance);
  }

  public double getX(){
    return m_poseEstimator.getEstimatedPosition().getTranslation().getX();
  }

  public double getY(){
    return m_poseEstimator.getEstimatedPosition().getTranslation().getY();
  }

  public boolean translationAligned(Pose3d aprilTagPose, double xOffset, double yOffset) {
    return(Math.abs(aprilTagPose.getX() + xOffset - getX()) < 0.01) && (Math.abs(aprilTagPose.getY() + yOffset - getY()) < 0.01);
  }

  public void moveToCoordinates(double x, double y){
    double tolerance  = 0.01;
    double xSpeed = 0, ySpeed = 0;
    double xError = x - getX();
    double yError = y - getY();
    double ksPercent = 0.035;
    double kpPercent = 0.070;
    
    if(Math.abs(xError) > tolerance){
      xSpeed = Math.signum(xError) * ksPercent + kpPercent * xError;
    }

    if(Math.abs(yError) > tolerance){
      ySpeed = Math.signum(yError) * ksPercent + kpPercent * yError;
    }

    
    SmartDashboard.putNumber("xError", xSpeed);
    SmartDashboard.putNumber("yError", ySpeed);

    drive(xSpeed, ySpeed, 0, true);

  }

  public void resetHeading(){
    zeroHeading();
    m_poseEstimator.resetRotation(new Rotation2d(0.0));
  }

  public void turnToHeading(double heading){
    double turnSpeed;
    double error = heading - getEstimatedHeading();

    if(error > 180) {
      error -= 360;
    }
    else if(error < -180){
      error += 360;
    }
    
    if(Math.abs(error) > angleTolerance){
      turnSpeed = Math.signum(error) * Constants.DriveConstants.ksPercent + Constants.DriveConstants.kpPercent * error;
    }
    else{
      turnSpeed = 0;
    }

    drive(0, 0, turnSpeed, true);

  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public void setHeading(double heading){ //Heading is in degrees
    m_gyro.setYaw(heading);
  }


  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    //return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
    return m_gyro.getYaw().getValueAsDouble();
  }

  public double getEstimatedHeading(){
    return m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
  }

  public Rotation2d getHeadingRotation2d(){
    return Rotation2d.fromDegrees(getEstimatedHeading());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    //return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    //return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    return m_gyro.getAngularVelocityZWorld().getValueAsDouble();
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = m_frontLeft.getPosition();
    positions[1] = m_frontRight.getPosition();
    positions[2] = m_rearLeft.getPosition();
    positions[3] = m_rearRight.getPosition();
    return positions;
  }
}
