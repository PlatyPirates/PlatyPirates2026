// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.AprilTagAlign;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


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
  private SendableChooser<AprilTagAlign> l4Dropdown = new SendableChooser<AprilTagAlign>();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  Shooter m_shooter = new Shooter();
  Climber m_climber = new Climber();
  Elevator m_elevator = new Elevator();
  Intake m_intake = new Intake();

  LEDs m_underglow = new LEDs(173);


  double driveSpeedFactor = 1.0;
  public boolean fieldRelative = true;

  int invert = 1;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    l4Dropdown.setDefaultOption("Center Tag", AprilTagAlign.CENTER);
    l4Dropdown.addOption("Left Tag", AprilTagAlign.LEFT);
    l4Dropdown.addOption("Right Tag", AprilTagAlign.RIGHT);

    SmartDashboard.putData("L4 Options", l4Dropdown);

    autoChooser.addOption("Cross Auto Line Only", new AMoveEnd(m_robotDrive, m_intake));
    autoChooser.addOption("Drive Robot From Limelight", new DriveRobotFromLimelight(m_robotDrive, m_underglow));
    autoChooser.addOption("Score L2 Coral", new AMoveLowCoral(m_robotDrive, m_shooter));
    autoChooser.setDefaultOption("Score L4 Coral", new AMoveL4(m_robotDrive, m_shooter, m_underglow, m_elevator, m_intake).moveAndL4(l4Dropdown));
    autoChooser.addOption("Do Nothing", new RunCommand(
      ()-> {m_robotDrive.drive(0.0,0.0,0.0,true); m_intake.setSpeed(1.0);}, m_robotDrive, m_intake)
    );

    SmartDashboard.putData("Auto Choices", autoChooser);
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> {m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY()*driveSpeedFactor*invert, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX()*driveSpeedFactor*invert, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX()*driveSpeedFactor, OIConstants.kDriveDeadband),
                fieldRelative);
              },
            m_robotDrive));

    m_climber.setDefaultCommand(
      new RunCommand(
        () -> {m_climber.setChainSpeed(-m_operatorController.getRightY()*0.15);
          m_climber.setMotorSpeed(0.0);
        },
        m_climber
        )
    );
    m_underglow.setDefaultCommand(
      new RunCommand(() -> {
        if(m_climber.isLimited() && (Timer.getMatchTime() <= 30.0) && DriverStation.isTeleop()){
          m_underglow.scrollingRainbow();
        } else {
          m_underglow.maroon();
        }
      }, m_underglow)
    );

    m_elevator.setDefaultCommand(
        new RunCommand(
        () -> {m_elevator.setSpeed(-m_operatorController.getLeftY()*0.5);
        }
      , m_elevator));

    m_shooter.setDefaultCommand(new RunCommand(() -> {
      m_shooter.setSpeed(0.0);
    }, m_shooter));

    m_intake.setDefaultCommand(new RunCommand( () -> {
      m_intake.setSpeed(0.0);
    }, m_intake));
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
      .a()
      .whileTrue(new DriveRobotFromLimelight(m_robotDrive, m_underglow)
      );

    m_driverController
      .b()
      .whileTrue(new RunCommand(
        () -> m_robotDrive.setX(), m_robotDrive));

    m_driverController
      .leftBumper()
      .whileTrue(new RunCommand(
        () -> m_robotDrive.resetHeading(),
        m_robotDrive));

    m_driverController
      .leftTrigger()
      .whileTrue(new RunCommand( () -> {
        driveSpeedFactor = 0.3;
        m_underglow.blink(Color.kYellow);
      }, m_underglow));

    m_driverController
      .leftTrigger()
      .whileFalse(new RunCommand(() -> driveSpeedFactor = 1.0));

    m_driverController
      .rightTrigger()
      .whileTrue(
        new RunCommand(
            () -> {m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY()*driveSpeedFactor, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX()*driveSpeedFactor, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX()*driveSpeedFactor, OIConstants.kDriveDeadband),
                false);

                //m_underglow.blink(Color.kWhite);
              },
            m_robotDrive));

    m_driverController
      .povLeft()
      .whileTrue(new RunCommand(() -> {DriveRobotFromLimelight.alignLeft();}
      ));

    m_driverController
      .povRight()
      .whileTrue(new RunCommand(() -> {DriveRobotFromLimelight.alignRight();}
      ));

    m_driverController
      .povLeft().or(m_driverController.povRight())
      .whileFalse(new RunCommand(
        () -> {
          DriveRobotFromLimelight.alignMiddle();
        }));

    m_operatorController
      .a()
      .whileTrue(new RunCommand(
        () -> {
          m_shooter.setSpeed(0.5);
        }, m_shooter));

    m_operatorController
      .b()
      .whileTrue(new LoadCoral(m_shooter, m_underglow, m_intake, m_elevator));

    m_operatorController
      .y()
      .whileTrue(new RunCommand(
        () -> {
          m_shooter.setSpeed(-0.5);
        }, m_shooter));

    m_operatorController
      .start()
      .whileTrue(new RunCommand(
        () -> {
          m_climber.setMotorSpeed(-1.0);
        }));

    m_operatorController
      .leftBumper()
      .whileTrue(new RunCommand(
        () -> {
          m_climber.setMotorSpeed(1.0);
        }));

    m_operatorController
      .rightTrigger()
      .whileTrue(new RunCommand(
        () -> {
          m_elevator.barge();
        }
      ));

    m_operatorController
      .povLeft().and(m_operatorController.leftTrigger().negate())
      .whileTrue(new RunCommand(() -> {
        m_elevator.l2();
      }, m_elevator));

    m_operatorController
      .povLeft().and(m_operatorController.leftTrigger())
      .whileTrue(new RunCommand(() -> {
        m_elevator.algae1();
      }, m_elevator));

    m_operatorController
      .povUp()
      .whileTrue(new RunCommand(() -> {
        m_elevator.l4();
      }, m_elevator));

    m_operatorController
      .povRight().and(m_operatorController.leftTrigger().negate())
      .whileTrue(new RunCommand(() -> {
        m_elevator.l3();
      }, m_elevator));

    m_operatorController
      .povRight().and(m_operatorController.leftTrigger())
      .whileTrue(new RunCommand(() -> {
        m_elevator.algae2();
      }, m_elevator));

    m_operatorController
      .povDown()
      .whileTrue(new RunCommand(() -> {
        m_elevator.l1();
      }, m_elevator));

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

  public void updateInversion(){
    if(DriverStation.getAlliance().get() == Alliance.Red){
      invert = -1;
    }
  }

}
