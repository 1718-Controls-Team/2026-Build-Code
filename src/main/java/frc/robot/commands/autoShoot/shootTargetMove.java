// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoShoot;

import frc.robot.subsystems.shooterIndexer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.spiralRoller;
import frc.robot.subsystems.turretHood;
import frc.robot.subsystems.hoodServo;
import frc.robot.subsystems.intakeFuel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
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
public class shootTargetMove extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final shooterIndexer m_shooterSubsystem;
  private final spiralRoller m_spiralRollerSubsystem;
  private final CommandSwerveDrivetrain m_Drivetrain;
  private final intakeFuel m_intakeSubsystem;
  private final turretHood m_turretSubsystem;
  private final hoodServo m_hoodSubsystem;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double legOne;
    private double legTwo;
    private double dist;
    private double currentTime;
    public boolean m_autoTarget = true;
    private double m_turretDegrees;
    private double m_robotDegrees;
    private double m_turretRadians;
    private boolean m_isFinished = false;
    private int shootFlag = 0;
    private Pose2d m_robotPose;
    private Optional<Alliance> m_alliance;
    private double prevTime;
    private double accelerationX;
    private double accelerationY;
    private final CommandXboxController m_driverController;
    private ChassisSpeeds velocity;
    private ChassisSpeeds previousLoopVelocity;
    private Timer spiralTimer;
    private double headingDeg;


    private final SwerveRequest.FieldCentricFacingAngle autoAlign = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed*0.05).withHeadingPID(8, 0, 0.01)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    Timer loopTimer = new Timer();


      /**
       * Creates a new set-PowerCommand.
       *
       * @param subsystem The subsystem used by this command.
       */
      public shootTargetMove(shooterIndexer shooter, spiralRoller spirals, hoodServo hood, CommandXboxController driver, CommandSwerveDrivetrain drivetrain, intakeFuel intake, turretHood turret) {
        m_shooterSubsystem = shooter;
        m_spiralRollerSubsystem = spirals;
        m_driverController = driver;
        m_hoodSubsystem = hood;
        m_Drivetrain = drivetrain;
        m_intakeSubsystem = intake;
        m_turretSubsystem = turret;

      addRequirements(shooter);
      addRequirements(spirals);
      addRequirements(intake);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      headingDeg = m_Drivetrain.getState().Pose.getRotation().getDegrees();
      shootFlag = 1;

      loopTimer.reset();
      loopTimer.start();
      currentTime = loopTimer.get();
      velocity = m_Drivetrain.getState().Speeds;
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

    legTwo = (legTwo - Constants.kShotTimeTable.get(dist)*((accelerationX*Constants.kAccelCompFactor) + velocity.vxMetersPerSecond));
    legOne = (legOne - Constants.kShotTimeTable.get(dist)*((accelerationY*Constants.kAccelCompFactor) + velocity.vyMetersPerSecond));
   
    dist = Math.sqrt(Math.pow(legTwo, 2) + Math.pow(legOne, 2));

    m_robotDegrees = Math.atan2(legOne, legTwo);

    m_Drivetrain.setControl(autoAlign.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) 
        .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
        .withTargetDirection(new Rotation2d((m_robotDegrees + 180))));

    m_turretDegrees = (Math.acos(legTwo / dist)) - m_robotPose.getRotation().getDegrees();
    SmartDashboard.putNumber("turret deg", m_turretDegrees);
    m_turretDegrees = (-m_turretDegrees / 108);
    SmartDashboard.putNumber("turret off", ((m_turretDegrees)));

    if (m_turretDegrees <= Constants.kTurretMax && m_turretDegrees >= Constants.kTurretMin) {
      m_turretSubsystem.setTurretMotorPos(m_turretDegrees);
    }
    
      switch (shootFlag) {
        case 1:
            m_hoodSubsystem.setPos1(0.35);
            shootFlag = 2;
          break;
        case 2:
          if (m_hoodSubsystem.getPos() == 0.35) {
            m_spiralRollerSubsystem.setSpiralRollerSpinSpeed(Constants.kRollerMainSpeed);
            m_shooterSubsystem.setShooterSpinSpeed(Constants.kSpeedTable.get(dist));
            shootFlag = 3;
          }
          break;
        case 3:
          if (m_shooterSubsystem.getShooterSpeed() > (Constants.kSpeedTable.get(dist) - 5)) {
            m_shooterSubsystem.setIndexerSpinSpeed(Constants.kIndexerMainSpeed);
            spiralTimer.reset();
            spiralTimer.start();
            shootFlag = 4;
          }
          break; 
        case 4:
          if (spiralTimer.get() >= 2) {
            m_intakeSubsystem.setIntakeSpinSpeed(Constants.kIntakeNoSpeed);
          }
          break;
      }
  } }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  m_shooterSubsystem.setShooterSpinSpeed(0);
  m_spiralRollerSubsystem.setSpiralRollerSpinSpeed(0);
  m_shooterSubsystem.setIndexerSpinSpeed(0);
  m_hoodSubsystem.setPos1(0.2);
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
