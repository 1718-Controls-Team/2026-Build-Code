package frc.robot.commands.autoShoot;


import frc.robot.subsystems.turretHood;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class NtargetMove extends Command {
  private final CommandSwerveDrivetrain m_Drivetrain;
  
    public boolean m_autoTarget = true;
    private double m_turretDegrees;
    private boolean m_isFinished = false;
    private Pose2d m_robotPose;
    private Optional<Alliance> m_alliance;
    private double dist;
    private double currentTime;
    private double prevTime;
    private double accelerationX;
    private double accelerationY;
    private ChassisSpeeds velocity;
    private ChassisSpeeds previousLoopVelocity;
    private boolean m_red;
    private double m_deltaX;
    private double m_deltaY;
    private double m_turretOffsetOne;
    private double m_turretOffsetTwo;
    private double deltaYTurretOne;
    private double deltaXTurretOne;
    private double deltaYTurretTwo;
    private double deltaXTurretTwo;
    private double turretXOne;
    private double turretYOne;
    private double turretXTwo;
    private double turretYTwo;
    private double m_hubX;
    private double m_hubY;
    private double turretOneHeading;
    private double turretTwoHeading;
    private double launchHeadingTurretOne;
    private double launchHeadingTurretTwo;
    private final turretHood m_turretSubsystem;

    Timer loopTimer = new Timer();

    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public NtargetMove(CommandSwerveDrivetrain drive, turretHood turret) {
      m_Drivetrain = drive;
      m_turretSubsystem = turret;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(m_turretSubsystem);
          
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
    loopTimer.reset();
    loopTimer.start();
    
    velocity = m_Drivetrain.getState().Speeds;
    currentTime = loopTimer.get();

   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    previousLoopVelocity = velocity;
    velocity = m_Drivetrain.getState().Speeds;

    prevTime = currentTime;
    currentTime = loopTimer.get();

    accelerationX = (velocity.vxMetersPerSecond - previousLoopVelocity.vxMetersPerSecond) / (currentTime - prevTime);
    accelerationY = (velocity.vyMetersPerSecond - previousLoopVelocity.vyMetersPerSecond) / (currentTime - prevTime); 

    m_robotPose = m_Drivetrain.getState().Pose;

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

    
    dist = Math.sqrt(Math.pow(m_deltaX, 2) + Math.pow(m_deltaY, 2));

    m_deltaX = (m_deltaX - Constants.kShotTimeTable.get(dist)*((accelerationX*Constants.kAccelCompFactor) + velocity.vxMetersPerSecond));
    m_deltaY = (m_deltaY - Constants.kShotTimeTable.get(dist)*((accelerationY*Constants.kAccelCompFactor) + velocity.vyMetersPerSecond));
   
    dist = Math.sqrt(Math.pow(m_deltaX, 2) + Math.pow(m_deltaY, 2));
    
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

    m_turretOffsetOne = (turretOneHeading / 90);
    m_turretOffsetTwo = (turretTwoHeading / 90);

    SmartDashboard.putNumber("distance", dist);
    SmartDashboard.putNumber("turret deg", m_turretDegrees);

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

