package frc.robot.commands.autoShoot;


import frc.robot.subsystems.shooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class hubTargeting extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final shooterSubsystem m_shooterSubsystem;
  private final CommandSwerveDrivetrain m_Drivetrain;
  
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private boolean m_isFinished = false;
    public boolean m_autoTarget = true;
    private PoseEstimate m_robotPose;
    private Optional<Alliance> m_alliance;
    private double m_turretDegrees;
    private double m_turretRadians;
    private double legOne;
    private double legTwo;
    private final CommandXboxController m_driverController;

    private final SwerveRequest.FieldCentricFacingAngle autoAlign = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed*0.05).withHeadingPID(8, 0, 0.01)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
 
  
    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public hubTargeting(shooterSubsystem turretSubsystem, CommandSwerveDrivetrain drive, CommandXboxController driver) {
      m_shooterSubsystem = turretSubsystem;
      m_Drivetrain = drive;
      m_driverController = driver;



     /* if ((m_autoTarget) && (m_kUseLimelight)) {
        m_turretOffset = Math.atan2(legTwo, legOne);
        SmartDashboard.putNumber("turretOffset", m_turretOffset);
        // robot pose 2d and rotation 2d compared to april tag coordinate
      } */
     
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(turretSubsystem);
      
          
    }
   
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_alliance = DriverStation.getAlliance();
    if (m_alliance.get() == Alliance.Red) {
      m_robotPose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
      legTwo = (m_robotPose.pose.getX() - Constants.kRedHubCoord[0]);
      legOne = (m_robotPose.pose.getY() - Constants.kRedHubCoord[1]);
    } else {
      m_robotPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      legTwo = (m_robotPose.pose.getX() - Constants.kBlueHubCoord[0]);
      legOne = (m_robotPose.pose.getY() - Constants.kBlueHubCoord[1]);
    }
    
    m_turretRadians = Math.atan2(legOne, legTwo);
    m_turretDegrees = (( m_turretRadians / Math.PI )*180);
    SmartDashboard.putNumber("turretOffset", m_turretDegrees);
    SmartDashboard.putNumber("legone", legOne);
    SmartDashboard.putNumber("robot Y", m_robotPose.pose.getY());
    SmartDashboard.putNumber("robot X", m_robotPose.pose.getX());
    SmartDashboard.putNumber("legtwo", legTwo);   
    SmartDashboard.putNumber("robot off", ((m_turretDegrees + 180)));
    m_Drivetrain.setControl(autoAlign.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) 
        .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
        .withTargetDirection(new Rotation2d(((m_turretDegrees + 180)/180)*Math.PI)));

    //m_shooterSubsystem.setTurretMotorPos(m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees() + m_turretDegrees);
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

