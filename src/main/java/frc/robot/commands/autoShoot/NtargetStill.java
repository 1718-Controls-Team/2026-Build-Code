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
    private double m_turretRadians;
    private double dist;
    private boolean m_isFinished = false;
    private Pose2d m_robotPose;
    private Optional<Alliance> m_alliance;
    private double legOne;
    private double legTwo;
    private PoseEstimate cry;
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
      legTwo = m_robotPose.getX() - Constants.kRedHubCoord[0];
      legOne = m_robotPose.getY() - Constants.kRedHubCoord[1];
    } else {
      legTwo = m_robotPose.getX() - Constants.kBlueHubCoord[0];
      legOne = m_robotPose.getY() - Constants.kBlueHubCoord[1];
    }  
  
    m_turretRadians = Math.atan2(legOne, legTwo);
    m_robotDegrees = (( m_turretRadians / Math.PI )*180);
    SmartDashboard.putNumber("legone", legOne);
    SmartDashboard.putNumber("legtwo", legTwo);   
    SmartDashboard.putNumber("robot off", ((m_robotPose.getRotation().getDegrees())));

  /*  m_Drivetrain.setControl(autoAlign.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) 
        .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
        .withTargetDirection(new Rotation2d(((m_robotDegrees + 180)/180)*Math.PI)));
    */

    dist = Math.sqrt(Math.pow(legTwo, 2) + Math.pow(legOne, 2));
    SmartDashboard.putNumber("distance", dist);

    m_turretDegrees = (((Math.atan2(legOne, legTwo))/ Math.PI )*180) - m_robotPose.getRotation().getDegrees();
    SmartDashboard.putNumber("turret deg", m_turretDegrees);
    m_turretDegrees = ((m_turretDegrees + 180) / 90);
    SmartDashboard.putNumber("turret off", ((m_turretDegrees)));

     m_turretSubsystem.setTurretMotorPos(m_turretDegrees);
    
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

