// This is our autonomous code 
// PlatyPirates team 9181 - written by Barbara and Victoria 

package frc.robot.commands;

// Imports and such
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.DriveSubsystem;

public class AutoShoot extends Command {

    private final Shooter m_shooter;
    private final Carousel m_carousel;
    private final DriveSubsystem m_drive;
    private final Timer m_timer = new Timer();

    // constants - numbers to refer back to later
    private static final double DRIVE_TIME = 2.0;
    private static final double TOTAL_AUTO_TIME = 6.0;
    private static final double SPINUP_TIME = 3.5;

    public AutoShoot(Shooter shooter, Carousel carousel, DriveSubsystem drive) {
        m_shooter = shooter;
        m_carousel = carousel;
        m_drive = drive;
        addRequirements(shooter, carousel, drive);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        // if the code has been running less then 2 seconds move backward
        if (m_timer.get() < DRIVE_TIME) {
            m_drive.drive(-0.4, 0.0, 0.0, true);

        } else if (m_timer.get() < SPINUP_TIME) { // if above is false stop and start running the shooter
            m_drive.drive(0.0, 0.0, 0.0, true);
            m_shooter.shoot();
            
        } else { // if everything above is false feed the balls into the shooter and shoot
            m_shooter.shoot();
            m_carousel.moveCarousel();
            m_shooter.feedBall();

        }
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() >= TOTAL_AUTO_TIME;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setX();  // this is what locks the wheels so the robot doesn't slide
        m_shooter.stopFlywheels();
        m_shooter.stopFeed();
        m_carousel.stopCarousel();

    }
}