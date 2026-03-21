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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


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
    private boolean m_isFinished = false;
    private Pose2d m_robotPose;
    private Optional<Alliance> m_alliance;
    private double legOne;
    private double legTwo;

    private final SwerveRequest.FieldCentricFacingAngle autoAlign = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed*0.05).withHeadingPID(8, 0, 0.01)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  
    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public shootSelf(shooterIndexer shooter, spiralRoller spirals, CommandSwerveDrivetrain drivetrain, intakeFuel intake, turretHood turret) {
        m_shooterSubsystem = shooter;
        m_spiralRollerSubsystem = spirals;
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
    m_robotPose = m_Drivetrain.getState().Pose;
    if (m_robotPose != null) {
       m_alliance = DriverStation.getAlliance();
    if (m_alliance.get() == Alliance.Red) {
      legTwo = Math.abs((m_robotPose.getX() - Constants.kRedHubCoord[0]));
      legOne = Math.abs((m_robotPose.getY() - Constants.kRedHubCoord[1]));
    } else {
      legTwo = Math.abs((m_robotPose.getX() - Constants.kBlueHubCoord[0]));
      legOne = Math.abs((m_robotPose.getY() - Constants.kBlueHubCoord[1]));
    }
    
    dist = Math.sqrt(Math.pow(legTwo, 2) + Math.pow(legOne, 2));


    switch (shootFlag) {
        case 1:
            m_spiralRollerSubsystem.setSpiralRollerSpinSpeed(Constants.kRollerMainSpeed);
            m_shooterSubsystem.setShooterSpinSpeed(Constants.kSpeedTable.get(dist));
            shootFlag = 2;
          break;
        case 2:
          if (m_shooterSubsystem.getShooterSpeed() > (Constants.kSpeedTable.get(dist) - 9)) {
            m_shooterSubsystem.setIndexerSpinSpeed(Constants.kIndexerMainSpeed);
          }
          break; 

      }
    }
      
    //m_shooterSubsystem.setTurretMotorPos(m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees() + m_turretDegrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  m_shooterSubsystem.setShooterOff(0);
  m_spiralRollerSubsystem.setSpiralRollerOff(0);
  m_intakeSubsystem.setIntakeOutput(0);
}
  // Returns true when the command should end. p
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}

