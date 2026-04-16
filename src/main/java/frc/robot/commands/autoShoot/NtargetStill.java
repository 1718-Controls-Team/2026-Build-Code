package frc.robot.commands.autoShoot;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.turretHood;


import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class NtargetStill extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_Drivetrain;
  
    public boolean m_autoTarget = true;
    private double m_turretDegrees;
    private boolean m_red;
    private double m_turretOffset;
    private double m_turretOffsetOne;
    private double m_turretOffsetTwo;
    private boolean m_isFinished = false;
    private Pose2d m_robotPose;
    private Optional<Alliance> m_alliance;
    private double m_deltaY;
    private double m_deltaX;
    private double m_hubX;
    private double m_hubY;
    private final turretHood m_turretSubsystem;
    private double deltaYTurretOne;
    private double deltaXTurretOne;
    private double deltaYTurretTwo;
    private double deltaXTurretTwo;
    private double turretXOne;
    private double turretYOne;
    private double turretXTwo;
    private double turretYTwo;
    private double turretOneHeading;
    private double turretTwoHeading;
    private double launchHeadingTurretOne;
    private double launchHeadingTurretTwo;

  
    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public NtargetStill(CommandSwerveDrivetrain drive, turretHood turret) {
      m_Drivetrain = drive;
      m_turretSubsystem = turret;
     
      // Use addRequirements() here to declare subsystem dependencies.
          
    }
   
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
     m_alliance = DriverStation.getAlliance();
    if (m_alliance.get() == Alliance.Red) {
      m_hubX = Constants.kRedHubCoord[0];
      m_hubY = Constants.kRedHubCoord[1];
      m_red = true;
    } else {
      m_hubX = Constants.kBlueHubCoord[0];
      m_hubY = Constants.kBlueHubCoord[1];
      m_red = false;
    } 
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   m_robotPose = m_Drivetrain.getState().Pose;

      m_deltaX =  m_hubX - m_robotPose.getX();
      m_deltaY =  m_hubY - m_robotPose.getY();

    SmartDashboard.putNumber("deltaY", m_deltaY);
    SmartDashboard.putNumber("deltaX", m_deltaX);   

    turretXOne = m_robotPose.getX() + ((-7 * Math.cos(m_robotPose.getRotation().getDegrees())) + (8 * Math.sin(m_robotPose.getRotation().getDegrees())));
    turretYOne = m_robotPose.getY() + ((-7 * Math.sin(m_robotPose.getRotation().getDegrees())) + (8 * Math.cos(m_robotPose.getRotation().getDegrees())));

    turretXTwo = m_robotPose.getX() + ((-7 * Math.cos(m_robotPose.getRotation().getDegrees())) + (-8 * Math.sin(m_robotPose.getRotation().getDegrees())));
    turretYTwo = m_robotPose.getY() + ((-7 * Math.sin(m_robotPose.getRotation().getDegrees())) + (-8 * Math.cos(m_robotPose.getRotation().getDegrees())));

   deltaXTurretOne = m_hubX - turretXOne;
   deltaYTurretOne = m_hubY - turretYOne;
   deltaXTurretTwo = m_hubX - turretXTwo;
   deltaYTurretTwo = m_hubY - turretYTwo;

   launchHeadingTurretOne = Math.atan2(deltaYTurretOne, deltaXTurretOne);
   launchHeadingTurretTwo = Math.atan2(deltaYTurretTwo, deltaXTurretTwo);
   turretOneHeading = launchHeadingTurretOne;
   turretTwoHeading = launchHeadingTurretTwo;

    if (m_red == true) {
      m_turretDegrees = ((((Math.atan2(m_deltaY, m_deltaX))/ Math.PI )*180) - (m_robotPose.getRotation().getDegrees()) + 180);
    } else {
      m_turretDegrees = ((((Math.atan2(m_deltaY, m_deltaX))/ Math.PI )*180) - (m_robotPose.getRotation().getDegrees()) + 180);
    }

    turretOneHeading = ((turretOneHeading + m_turretDegrees));
    turretTwoHeading = ((turretTwoHeading + m_turretDegrees));

    if (turretOneHeading > 180) {
      turretOneHeading = turretOneHeading - 360;
      turretTwoHeading = turretTwoHeading - 360;
    }
    if (turretOneHeading < -180) {
      turretOneHeading = turretOneHeading + 360;
      turretTwoHeading = turretTwoHeading + 360;
    }
      
    if (m_turretDegrees < -180) {
      m_turretDegrees = m_turretDegrees + 360;
    }
    if (m_turretDegrees > 180) {
      m_turretDegrees = m_turretDegrees - 360;
    }

    m_turretOffsetOne = (turretOneHeading / 90);
    m_turretOffsetTwo = (turretTwoHeading / 90);
    m_turretOffset = (m_turretDegrees / 90);

    SmartDashboard.putNumber("turret deg", m_turretDegrees);
    SmartDashboard.putNumber("turret off", m_turretOffset);

     //m_turretSubsystem.setTurretMotorPos(m_turretOffset);
     m_turretSubsystem.setTurretMotor1(m_turretOffsetTwo);
     m_turretSubsystem.setTurretMotor2(m_turretOffsetOne);
    
}
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
}
  // Returns true when the command should end. p
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}

