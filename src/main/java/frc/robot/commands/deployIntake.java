package frc.robot.commands;



import frc.robot.subsystems.intakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class deployIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final intakeSubsystem m_intakeSubsystem;
  
    private boolean m_isFinished = false;
    private int m_stateMachine = 1;
  
  
    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public deployIntake(intakeSubsystem intakeSubsystem) {
      m_intakeSubsystem = intakeSubsystem;
  
     
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(intakeSubsystem);
      
          
    }
   
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_stateMachine = 1;
        m_isFinished = false;
    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   switch (m_stateMachine) {
    case 1:
            m_intakeSubsystem.setIntakeElectricSlidePos(Constants.kintakeSlidePos);
            m_stateMachine = 2;
        break;
    case 2:
        if (m_intakeSubsystem.getIntakeSlideInPos()) {
            m_intakeSubsystem.setIntakeSpinSpeed(11);
            m_stateMachine = 0;
        }
        break; 
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