package frc.robot.commands.autoShoot;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class smartPass extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_Drivetrain;
  
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private boolean m_isFinished = false;
    public boolean m_autoTarget = true;
    private PoseEstimate m_robotPose;
    private double m_passDegrees;
    private double m_passRadians;
    private double passX;
    private double passYLeft;
    private double passYRight;

    private final SwerveRequest.FieldCentricFacingAngle autoAlign = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed*0.05).withHeadingPID(8, 0, 0.01)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  
    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public smartPass(CommandSwerveDrivetrain drive) {
      m_Drivetrain = drive;



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
    SmartDashboard.putNumber("pass off", ((m_passDegrees)));
    //m_shooterSubsystem.setTurretMotorPos(m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees() + m_turretDegrees);
    m_Drivetrain.setControl(autoAlign.withTargetDirection(new Rotation2d(((m_passDegrees + 180)/180)*Math.PI)));
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

