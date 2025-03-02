// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AMoveEnd;
import frc.robot.commands.AMoveLowCoral;
import frc.robot.commands.DriveRobotFromLimelight;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  Intake m_intake = new Intake();
  Climber m_climber = new Climber();

  double driveSpeedFactor = 1.0;
  boolean fieldRelative = true;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    autoChooser.setDefaultOption("Cross Auto Line Only", new AMoveEnd(m_robotDrive));
    autoChooser.addOption("Drive Robot From Limelight", new DriveRobotFromLimelight(m_robotDrive));
    autoChooser.addOption("Score L2 Coral", new AMoveLowCoral(m_robotDrive, m_intake));
    autoChooser.addOption("Do Nothing",

    new RunCommand(
      ()-> m_robotDrive.drive(0.0,0.0,0.0,true), m_robotDrive)
  );
  
    SmartDashboard.putData("Auto Choices", autoChooser);
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY()*driveSpeedFactor, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX()*driveSpeedFactor, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX()*driveSpeedFactor, OIConstants.kDriveDeadband),
                fieldRelative),
            m_robotDrive));

    m_climber.setDefaultCommand(
      new RunCommand(
        () -> m_climber.setChainSpeed(-m_operatorController.getRightY()*0.15),
        m_climber
        )
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    m_driverController
      .b()
      .whileTrue(new RunCommand(
        () -> m_robotDrive.setX(),
            m_robotDrive));

    m_driverController
      .leftBumper()
      .whileTrue(new RunCommand(
        () -> m_robotDrive.zeroHeading(),
        m_robotDrive));

     m_driverController
      .a()
      .whileTrue(new DriveRobotFromLimelight(m_robotDrive)
      ); 

    m_driverController
      .leftTrigger()
      .whileTrue(new RunCommand( () -> driveSpeedFactor = 0.3));

    m_driverController
      .leftTrigger()
      .whileFalse(new RunCommand(() -> driveSpeedFactor = 1.0));

    m_operatorController
      .leftBumper()
      .whileTrue(new RunCommand(
        () -> {
          m_climber.setMotorSpeed(1.0);
        }));

    m_operatorController
      .leftBumper()
      .whileFalse(new RunCommand(
        () -> {
          m_climber.setMotorSpeed(0.0);
        }));

    m_operatorController
      .start()
      .whileTrue(new RunCommand(
        () -> {
          m_climber.setMotorSpeed(-1.0);
        }));

    m_operatorController
      .start()
      .whileFalse(new RunCommand(
        () -> {
          m_climber.setMotorSpeed(0.0);
        }));

    m_driverController
      .rightTrigger()
      .onTrue(new RunCommand(
        () -> {
          fieldRelative = !fieldRelative;
        }));

    m_operatorController
      .povUp()
      .whileTrue(new RunCommand(
        () -> {
          m_intake.setSpeed(0.2);
        }));

    m_operatorController
      .povDown()
      .whileTrue(new RunCommand(
        () -> {
          m_intake.setSpeed(-0.5);
        }));

    m_operatorController
      .povUp()
      .onFalse(new RunCommand(
        () -> {
          m_intake.setSpeed(0.0);
        }));

    m_operatorController
      .povDown()
      .onFalse(new RunCommand(
        () -> {
          m_intake.setSpeed(0.0);
        }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // Create config for trajectory
    /*TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    */
  }

  public double getIntakeCurrent(){
    return m_intake.getOutputCurrent();
  }
}
