package frc.robot.commands.autoShoot;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.turretHood;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class NtargetStill extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_Drivetrain;
  
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public boolean m_autoTarget = true;
    private double m_turretDegrees;
    private double m_robotDegrees;
    private double m_turretOffset;
    private double m_turretOffsetOne;
    private double m_turretOffsetTwo;
    private double m_turretRadians;
    private double dist;
    private boolean m_isFinished = false;
    private Pose2d m_robotPose;
    private Optional<Alliance> m_alliance;
    private double deltaY;
    private double deltaX;
    private double hubX;
    private double hubY;
    private final CommandXboxController m_driverController;
    private final turretHood m_turretSubsystem;

  
    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public NtargetStill(CommandSwerveDrivetrain drive, CommandXboxController driver, turretHood turret) {
      m_Drivetrain = drive;
      m_driverController = driver;
      m_turretSubsystem = turret;



     /* if ((m_autoTarget) && (m_kUseLimelight)) {
        m_turretOffset = Math.atan2(legTwo, legOne);
        SmartDashboard.putNumber("turretOffset", m_turretOffset);
        // robot pose 2d and rotation 2d compared to april tag coordinate
      } */
     
      // Use addRequirements() here to declare subsystem dependencies.
      
          
    }
   
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   m_robotPose = m_Drivetrain.getState().Pose;
    m_alliance = DriverStation.getAlliance();
    if (m_alliance.get() == Alliance.Red) {
      deltaX = m_robotPose.getX() - Constants.kRedHubCoord[0];
      deltaY = m_robotPose.getY() - Constants.kRedHubCoord[1];
      hubX = Constants.kRedHubCoord[0];
      hubY = Constants.kRedHubCoord[1];
    } else {
      deltaX = Constants.kBlueHubCoord[0] - m_robotPose.getX();
      deltaY = Constants.kBlueHubCoord[1] - m_robotPose.getY();
      hubX = Constants.kBlueHubCoord[0];
      hubY = Constants.kBlueHubCoord[1];
    }  
  
    m_turretRadians = Math.atan2(deltaY, deltaX);
    m_robotDegrees = (( m_turretRadians / Math.PI )*180);
    SmartDashboard.putNumber("deltaY", deltaY);
    SmartDashboard.putNumber("deltaX", deltaX);   
    SmartDashboard.putNumber("robot off", ((m_robotPose.getRotation().getDegrees())));

   double turretXOne = m_robotPose.getX() + ((-7 * Math.cos(m_robotPose.getRotation().getDegrees())) + (8 * Math.sin(m_robotPose.getRotation().getDegrees())));
   double turretYOne = m_robotPose.getY() + ((-7 * Math.sin(m_robotPose.getRotation().getDegrees())) + (8 * Math.cos(m_robotPose.getRotation().getDegrees())));

   double turretXTwo = m_robotPose.getX() + ((-7 * Math.cos(m_robotPose.getRotation().getDegrees())) + (-8 * Math.sin(m_robotPose.getRotation().getDegrees())));
   double turretYTwo = m_robotPose.getY() + ((-7 * Math.sin(m_robotPose.getRotation().getDegrees())) + (-8 * Math.cos(m_robotPose.getRotation().getDegrees())));

   double deltaXTurretOne = hubX - turretXOne;
   double deltaYTurretOne = hubY - turretYOne;
   double deltaXTurretTwo = hubX - turretXTwo;
   double deltaYTurretTwo = hubY - turretYTwo;

   double launchHeadingTurretOne = Math.atan2(deltaYTurretOne, deltaXTurretOne);
   double launchHeadingTurretTwo = Math.atan2(deltaYTurretTwo, deltaXTurretTwo);
   double turretOneHeading = launchHeadingTurretOne - m_robotPose.getRotation().getDegrees();
   double turretTwoHeading = launchHeadingTurretTwo - m_robotPose.getRotation().getDegrees();
  /*  m_Drivetrain.setControl(autoAlign.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) 
        .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
        .withTargetDirection(new Rotation2d(((m_robotDegrees + 180)/180)*Math.PI)));
    */

    dist = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    m_turretDegrees = (((Math.atan2(deltaY, deltaX))/ Math.PI )*180) - m_robotPose.getRotation().getDegrees();
    turretOneHeading = ((turretOneHeading + m_turretDegrees));
    turretTwoHeading = ((turretTwoHeading + m_turretDegrees));

    if (turretOneHeading > 180) {
      turretOneHeading = turretOneHeading - 360;
      turretTwoHeading = turretTwoHeading - 360;
    }
    if (turretOneHeading < 180) {
      turretOneHeading = turretOneHeading + 360;
      turretTwoHeading = turretTwoHeading + 360;
    }

    m_turretOffsetOne = (turretOneHeading / 90);
    m_turretOffsetTwo = (turretTwoHeading / 90);
    m_turretOffset = (m_turretDegrees / 90);

    SmartDashboard.putNumber("distance", dist);
    SmartDashboard.putNumber("turret deg", m_turretDegrees);
    SmartDashboard.putNumber("turret off", ((m_turretOffset)));

     m_turretSubsystem.setTurretMotorPos(m_turretOffset);
     //m_turretSubsystem.setTurretMotor2(m_turretOffsetTwo);
    
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

