package frc.robot.commands.autoShoot;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intakeFuel;
import frc.robot.subsystems.shooterIndexer;
import frc.robot.subsystems.spiralRoller;
import frc.robot.subsystems.turretHood;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;


/** An example command that uses an example subsystem. */
public class shootSelf extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final shooterIndexer m_shooterSubsystem;
  private final spiralRoller m_spiralRollerSubsystem;
  private final CommandSwerveDrivetrain m_Drivetrain;
  private final intakeFuel m_intakeSubsystem;
  private final turretHood m_turretSubsystem;

    private double dist;
    private int shootFlag = 0;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public boolean m_autoTarget = true;
    private double m_turretDegrees;
    private double m_turretRadians;
    private boolean m_isFinished = false;
    private PoseEstimate m_robotPose;
    private Optional<Alliance> m_alliance;
    private double legOne;
    private double legTwo;
    private final CommandXboxController m_driverController;

    private final SwerveRequest.FieldCentricFacingAngle autoAlign = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed*0.05).withHeadingPID(8, 0, 0.01)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  
    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public shootSelf(shooterIndexer shooter, spiralRoller spirals,  CommandXboxController driver, CommandSwerveDrivetrain drivetrain, intakeFuel intake, turretHood turret) {
        m_shooterSubsystem = shooter;
        m_spiralRollerSubsystem = spirals;
        m_driverController = driver;
        m_Drivetrain = drivetrain;
        m_intakeSubsystem = intake;
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
    shootFlag = 1;
    

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


    switch (shootFlag) {
        case 1:
            m_spiralRollerSubsystem.setSpiralRollerSpinSpeed(Constants.kRollerMainSpeed);
            m_shooterSubsystem.setShooterSpinSpeed(Constants.kSpeedTable.get(dist) + 4);
            shootFlag = 2;
          break;
        case 2:
          if (m_shooterSubsystem.getShooterSpeed() > (Constants.kSpeedTable.get(dist) - 5)) {
            m_shooterSubsystem.setIndexerSpinSpeed(Constants.kIndexerMainSpeed);
          }
          break; 

      }
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

