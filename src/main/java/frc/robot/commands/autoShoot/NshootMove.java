// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoShoot;

import frc.robot.subsystems.shooterIndexer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.spiralRoller;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class NshootMove extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final shooterIndexer m_shooterSubsystem;
  private final spiralRoller m_spiralRollerSubsystem;
  private final CommandSwerveDrivetrain m_Drivetrain;
  
  
    private boolean m_isFinished = false;
    private Pose2d m_robotPose;
    private double dist;
    private double currentTime;
    private double prevTime;
    private ChassisSpeeds velocity;
    private ChassisSpeeds previousLoopVelocity;
    private double deltaX;
    private double deltaY;
    public boolean m_autoTarget = true;
    private double m_hubX;
    private double m_hubY;
    private double m_deltaX;
    private double m_deltaY;
    private double accelerationX;
    private double accelerationY;

    Timer loopTimer = new Timer();
  
    
      /**
       * Creates a new set-PowerCommand.
       *
       * @param subsystem The subsystem used by this command.
       */
      public NshootMove(shooterIndexer shooter, spiralRoller spirals, CommandSwerveDrivetrain drivetrain) {
        m_shooterSubsystem = shooter;
        m_spiralRollerSubsystem = spirals;
        m_Drivetrain = drivetrain;
    
      addRequirements(shooter);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
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

    dist = Math.sqrt(Math.pow(m_deltaX, 2) + Math.pow(m_deltaY, 2));

    deltaX = (m_deltaX - Constants.kShotTimeTable.get(dist)*((accelerationX*Constants.kAccelCompFactor) + velocity.vxMetersPerSecond));
    deltaY = (m_deltaY - Constants.kShotTimeTable.get(dist)*((accelerationY*Constants.kAccelCompFactor) + velocity.vyMetersPerSecond));
   
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
  m_shooterSubsystem.setShooterSpinSpeed(0);
  m_spiralRollerSubsystem.setSpiralRollerSpinSpeed(0);
  m_shooterSubsystem.setIndexerSpinSpeed(0);
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
