package frc.robot.commands.autoShoot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.hoodServo;
import frc.robot.subsystems.intakeFuel;
import frc.robot.subsystems.shooterIndexer;
import frc.robot.subsystems.spiralRoller;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class smartPass extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_Drivetrain;
  private final shooterIndexer m_shooterSubsystem;
  private final spiralRoller m_spiralRollerSubsystem;
  private final intakeFuel m_intakeSubsystem;
  private final hoodServo m_hoodServo;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private boolean m_isFinished = false;
    private int shootFlag = 0;
    Timer spiralTimer = new Timer();
    public boolean m_autoTarget = true;
    private PoseEstimate m_robotPose;
    private double m_passDegrees;
    private double m_passRadians;
    private double passX;
    private double passYLeft;
    private double passYRight;
    
    
    private final CommandXboxController m_driverController;

    private final SwerveRequest.FieldCentricFacingAngle autoAlign = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed*0.05).withHeadingPID(8, 0, 0.01)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  
    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public smartPass(CommandSwerveDrivetrain drive, CommandXboxController driver, shooterIndexer shooter, spiralRoller spirals, intakeFuel intake, hoodServo hood) {
      m_shooterSubsystem = shooter;
      m_spiralRollerSubsystem = spirals;
      m_intakeSubsystem = intake;
      m_hoodServo = hood;
      m_Drivetrain = drive;
      m_driverController = driver;


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
    m_robotPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    passX = (m_robotPose.pose.getX() - Constants.kBluePassCoords[0]);
    passYLeft = (m_robotPose.pose.getY() - Constants.kBluePassCoords[1]);
    passYRight = (m_robotPose.pose.getY() - Constants.kBluePassCoords[2]);
    if ((Math.sqrt(Math.pow(passX, 2) + Math.pow(passYLeft, 2))) > (Math.sqrt(Math.pow(passX, 2) + Math.pow(passYRight, 2)))) {
      m_passRadians = Math.atan2(passYRight, passX);
    } else {
      m_passRadians = Math.atan2(passYLeft, passX);
    }

    m_passDegrees = (( m_passRadians / Math.PI )*180);
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (shootFlag) {
        case 1:
            m_hoodServo.setPos1(0.35);
            m_spiralRollerSubsystem.setSpiralRollerSpinSpeed(Constants.kRollerMainSpeed);
            m_shooterSubsystem.setShooterSpinSpeed(90);
            shootFlag = 2;
          break;
        case 2:
          if (m_hoodServo.getPos() == (0.35 + 0.05) || m_hoodServo.getPos() == (0.35 - 0.05)) {
            m_shooterSubsystem.setIndexerSpinSpeed(Constants.kIndexerMainSpeed);
            spiralTimer.reset();
            spiralTimer.start();
            shootFlag = 3;
          }
          break; 
        case 3:
          if (spiralTimer.get() >= 2) {
            m_intakeSubsystem.setIntakeSpinSpeed(Constants.kIntakeNoSpeed);
            //if (m_intakeSubsystem.getIntakeElectricSlidePos() != (Constants.kIntakeSlideInPos +- .5)) {
            //  m_intakeSubsystem.setIntakeElectricSlidePos(Constants.kIntakeSlideOutPos + 0.5);
            //}
          }
          break;
      }
  
    SmartDashboard.putNumber("pass off", ((m_passDegrees)));
    //m_shooterSubsystem.setTurretMotorPos(m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees() + m_turretDegrees);
    m_Drivetrain.setControl(autoAlign.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) 
        .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
        .withTargetDirection(new Rotation2d(((m_passDegrees + 180)/180)*Math.PI)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}

