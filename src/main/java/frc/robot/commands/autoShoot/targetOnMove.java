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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class targetOnMove extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final shooterSubsystem m_shooterSubsystem;
  private final CommandSwerveDrivetrain m_Drivetrain;
  
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public boolean m_autoTarget = true;
    private double m_turretDegrees;
    private double m_turretRadians;
    private boolean m_isFinished = false;
    private int shootFlag = 0;
    private PoseEstimate m_robotPose;
    private Optional<Alliance> m_alliance;
    private double legOne;
    private double legTwo;
    private double dist;
    private double currentTime;
    private double prevTime;
    private double[] acceleration;
    private ChassisSpeeds velocity;
    private ChassisSpeeds previousLoopVelocity;
    private final CommandXboxController m_driverController;

    Timer loopTimer = new Timer();

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
    public targetOnMove(shooterSubsystem turretSubsystem, CommandSwerveDrivetrain drive, CommandXboxController driver) {
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
    previousLoopVelocity = velocity;
    velocity = m_Drivetrain.getState().Speeds;

    prevTime = currentTime;
    currentTime = loopTimer.get();

    acceleration[0] = (velocity.vxMetersPerSecond - previousLoopVelocity.vxMetersPerSecond) / (currentTime - prevTime);
    acceleration[1] = (velocity.vyMetersPerSecond - previousLoopVelocity.vyMetersPerSecond) / (currentTime - prevTime); 

    m_alliance = DriverStation.getAlliance();
    if (m_alliance.get() == Alliance.Red) {
      m_robotPose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
      legTwo = (Constants.kRedHubCoord[0] - m_robotPose.pose.getX());
      legOne = (Constants.kRedHubCoord[1] - m_robotPose.pose.getY());
    } 
    else {
      m_robotPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      legTwo = (Constants.kBlueHubCoord[0] - m_robotPose.pose.getX());
      legOne = (Constants.kBlueHubCoord[1] - m_robotPose.pose.getY());
    }

    dist = Math.sqrt(Math.pow(legTwo, 2) + Math.pow(legOne, 2));

    legTwo = (legTwo - Constants.kShotTimeTable.get(dist)*((acceleration[0]*Constants.kAccelCompFactor) + velocity.vxMetersPerSecond));
    legOne = (legOne - Constants.kShotTimeTable.get(dist)*((acceleration[1]*Constants.kAccelCompFactor) + velocity.vyMetersPerSecond));
   
    dist = Math.sqrt(Math.pow(legTwo, 2) + Math.pow(legOne, 2));

    
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

