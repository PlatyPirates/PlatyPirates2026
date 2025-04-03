// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.AMoveL4;
import frc.robot.commands.DriveRobotFromLimelight;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private SendableChooser<Command> ledTester = new SendableChooser<Command>();
  private SendableChooser<Command> l4Dropdown = new SendableChooser<Command>();


  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    m_robotContainer.m_underglow.maroon();

    m_robotContainer.m_robotDrive.zeroHeading();

    ledTester.addOption("Rainbow", new RunCommand(() -> {m_robotContainer.m_underglow.rainbow();}, m_robotContainer.m_underglow));
    ledTester.addOption("Rainbow Scroll", new RunCommand(() -> {m_robotContainer.m_underglow.scrollingRainbow();}, m_robotContainer.m_underglow));
    ledTester.addOption("Blink White", new RunCommand(() -> {m_robotContainer.m_underglow.blink(Color.kWhite);}, m_robotContainer.m_underglow));
    ledTester.addOption("Maroon and White Scrolling", new RunCommand(() -> {m_robotContainer.m_underglow.maroonAndWhiteScrolling();}, m_robotContainer.m_underglow));
    ledTester.addOption("Maroon and White", new RunCommand(() -> {m_robotContainer.m_underglow.maroonAndWhite();}, m_robotContainer.m_underglow));
    ledTester.addOption("Loading", new RunCommand(() -> {m_robotContainer.m_underglow.loading();}, m_robotContainer.m_underglow));
    ledTester.addOption("Error", new RunCommand(() -> {m_robotContainer.m_underglow.error();}, m_robotContainer.m_underglow));

    DriveRobotFromLimelight.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  
    l4Dropdown.addOption("Center Tag", new RunCommand(() -> {AMoveL4.centerTag();}));
    l4Dropdown.addOption("Left Tag", new RunCommand(() -> {AMoveL4.leftTag();}));
    l4Dropdown.addOption("Right Tag", new RunCommand(() -> {AMoveL4.rightTag();}));

    SmartDashboard.putData(l4Dropdown);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Robot Heading", m_robotContainer.m_robotDrive.getHeading());
    SmartDashboard.putNumber("Estimated Heading", m_robotContainer.m_robotDrive.getEstimatedHeading());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    // if(ledTester.getSelected() != null){
    //   ledTester.getSelected().execute();
    // }
    m_robotContainer.m_underglow.loading();

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      l4Dropdown.getSelected().andThen(m_autonomousCommand).schedule();
    }

    m_robotContainer.updateInversion();

    m_robotContainer.m_robotDrive.poseInit();


    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
