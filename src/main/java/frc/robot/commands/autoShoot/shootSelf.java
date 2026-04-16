package frc.robot.commands.autoShoot;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intakeFuel;
import frc.robot.subsystems.shooterIndexer;
import frc.robot.subsystems.spiralRoller;
import frc.robot.subsystems.turretHood;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class shootSelf extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final shooterIndexer m_shooterSubsystem;
  private final spiralRoller m_spiralRollerSubsystem;
  private final CommandSwerveDrivetrain m_Drivetrain;
  private final intakeFuel m_intakeSubsystem;
  private double dist;
    public boolean m_autoTarget = true;
    private boolean m_isFinished = false;
    private Pose2d m_robotPose;
    private Optional<Alliance> m_alliance;
    private double legOne;
    private double legTwo;


  
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
        legTwo = Math.abs((m_robotPose.getX() - Constants.kRedHubCoord[0]));
        legOne = Math.abs((m_robotPose.getY() - Constants.kRedHubCoord[1]));
     } else {
        legTwo = Math.abs((m_robotPose.getX() - Constants.kBlueHubCoord[0]));
        legOne = Math.abs((m_robotPose.getY() - Constants.kBlueHubCoord[1]));
      }
         dist = Math.sqrt(Math.pow(legTwo, 2) + Math.pow(legOne, 2));
            m_shooterSubsystem.setShooterSpinSpeed(Constants.kSpeedTable.get(dist));
             if (m_shooterSubsystem.getShooterSpeedR() > (Constants.kSpeedTable.get(dist) - 9)) {
            m_shooterSubsystem.setIndexerSpinTorq(Constants.kIndexerMainSpeed);
            m_spiralRollerSubsystem.setSpiralRollerOff(Constants.kRollerMainSpeed);
          }
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

