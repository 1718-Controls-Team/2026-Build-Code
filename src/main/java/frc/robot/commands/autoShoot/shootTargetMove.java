// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoShoot;

import frc.robot.subsystems.shooterIndexer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.spiralRoller;
import frc.robot.subsystems.turretHood;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;



/** An example command that uses an example subsystem. */
public class shootTargetMove extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final shooterIndexer m_shooterSubsystem;
  private final spiralRoller m_spiralRollerSubsystem;
  private final CommandSwerveDrivetrain m_Drivetrain;
  private final turretHood m_turretSubsystem;

    private double legOne;
    private double deltaX;
    private double deltaY;
    private double legTwo;
    private double dist;
    private double currentTime;
    public boolean m_autoTarget = true;
    private double m_turretDegrees;
    private boolean m_isFinished = false;
    private int shootFlag = 0;
    private double m_hubX;
    private double m_hubY;
    private double m_deltaX;
    private double m_deltaY;
    private boolean m_red;
    private Pose2d m_robotPose;
    private Optional<Alliance> m_alliance;
    private double prevTime;
    private double accelerationX;
    private double accelerationY;
    private ChassisSpeeds velocity;
    private ChassisSpeeds previousLoopVelocity;
    private double m_turretOffsetOne;
    private double m_turretOffsetTwo;


    Timer loopTimer = new Timer();


      /**
       * Creates a new set-PowerCommand.
       *
       * @param subsystem The subsystem used by this command.
       */
      public shootTargetMove(shooterIndexer shooter, spiralRoller spirals, CommandSwerveDrivetrain drivetrain, turretHood turret) {
        m_shooterSubsystem = shooter;
        m_spiralRollerSubsystem = spirals;
        m_Drivetrain = drivetrain;
        m_turretSubsystem = turret;

      addRequirements(shooter);
      addRequirements(spirals);
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

    m_deltaX =  m_hubX - m_robotPose.getX();
    m_deltaY =  m_hubY - m_robotPose.getY();

    SmartDashboard.putNumber("deltaY", m_deltaY);
    SmartDashboard.putNumber("deltaX", m_deltaX);   

   double turretXOne = m_robotPose.getX() + ((-7 * Math.cos(m_robotPose.getRotation().getDegrees())) + (8 * Math.sin(m_robotPose.getRotation().getDegrees())));
   double turretYOne = m_robotPose.getY() + ((-7 * Math.sin(m_robotPose.getRotation().getDegrees())) + (8 * Math.cos(m_robotPose.getRotation().getDegrees())));

   double turretXTwo = m_robotPose.getX() + ((-7 * Math.cos(m_robotPose.getRotation().getDegrees())) + (-8 * Math.sin(m_robotPose.getRotation().getDegrees())));
   double turretYTwo = m_robotPose.getY() + ((-7 * Math.sin(m_robotPose.getRotation().getDegrees())) + (-8 * Math.cos(m_robotPose.getRotation().getDegrees())));

  double deltaXTurretOne = m_hubX - turretXOne;
   double deltaYTurretOne = m_hubY - turretYOne;
   double deltaXTurretTwo = m_hubX - turretXTwo;
   double deltaYTurretTwo = m_hubY - turretYTwo;

   double launchHeadingTurretOne = Math.atan2(deltaYTurretOne, deltaXTurretOne);
   double launchHeadingTurretTwo = Math.atan2(deltaYTurretTwo, deltaXTurretTwo);
   double turretOneHeading;
   double turretTwoHeading;


    dist = Math.sqrt(Math.pow(m_deltaX, 2) + Math.pow(m_deltaY, 2));
    if (m_red == true) {
      m_turretDegrees = ((((Math.atan2(m_deltaY, m_deltaX))/ Math.PI )*180) - (m_robotPose.getRotation().getDegrees()) + 180);
    } else {
      m_turretDegrees = ((((Math.atan2(m_deltaY, m_deltaX))/ Math.PI )*180) - (m_robotPose.getRotation().getDegrees()) + 180);
    }

    turretOneHeading = ((launchHeadingTurretOne + m_turretDegrees));
    turretTwoHeading = ((launchHeadingTurretTwo + m_turretDegrees));

    if (turretOneHeading > 180) {
      turretOneHeading = turretOneHeading - 360;
      turretTwoHeading = turretTwoHeading - 360;
    }
    if (turretOneHeading < -180) {
      turretOneHeading = turretOneHeading + 360;
      turretTwoHeading = turretTwoHeading + 360;
    }
      
    if (m_turretDegrees < -180) {
      m_turretDegrees = m_turretDegrees + 360;
    }
    if (m_turretDegrees > 180) {
      m_turretDegrees = m_turretDegrees - 360;
    }

    m_turretOffsetOne = (turretOneHeading / 90);
    m_turretOffsetTwo = (turretTwoHeading / 90);

    SmartDashboard.putNumber("distance", dist);
    SmartDashboard.putNumber("turret deg", m_turretDegrees);
    SmartDashboard.putNumber("turret off", m_turretOffsetOne);

     m_turretSubsystem.setTurretMotor1(m_turretOffsetTwo);
     m_turretSubsystem.setTurretMotor2(m_turretOffsetOne);
    

    dist = Math.sqrt(Math.pow(legTwo, 2) + Math.pow(legOne, 2));

    deltaX = (legTwo - Constants.kShotTimeTable.get(dist)*((accelerationX*Constants.kAccelCompFactor) + velocity.vxMetersPerSecond));
    deltaY = (legOne - Constants.kShotTimeTable.get(dist)*((accelerationY*Constants.kAccelCompFactor) + velocity.vyMetersPerSecond));
   
    dist = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

    if (Constants.kSpeedTable.get(dist) >= 92) {
      m_shooterSubsystem.setIndexerSpinTorq(85);
    } else {
      m_shooterSubsystem.setShooterSpinSpeed(Constants.kSpeedTable.get(dist));
    }

      if (m_shooterSubsystem.getShooterSpeedR() > (Constants.kSpeedTable.get(dist) - 9)) {
        m_shooterSubsystem.setIndexerSpinSpeed(Constants.kIndexerMainSpeed);
        m_spiralRollerSubsystem.setSpiralRollerOff(Constants.kRollerMainSpeed);
        }
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
