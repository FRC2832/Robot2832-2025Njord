package org.livoniawarriors.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Arrays;
import java.util.function.BooleanSupplier;

/**
 * A command which switches between the behaviors based on the conditions. <br>
 * The conditions are prioritized in the order they are specified in the arrays. The commands array
 * must be 1 longer than the conditions array. The last command will be the default command to run
 * if no condition is met. Ending the SwitchCommand ends all commands passed to it. Finishes when
 * the currently running command finishes. If you want to continue the command, use a {@code
 * RepeatCommand}<br>
 * <br>
 * SwitchCommandBuilder provides an easier way to construct this.
 */
public class SwitchCommand extends Command {
  private Command[] commands;
  private BooleanSupplier[] conditions;
  private int lastCommandDex = -1;
  private boolean[] initialized;

  SwitchCommand(BooleanSupplier[] conditions, Command[] commands) {
    assert conditions.length == commands.length - 1;
    this.commands = commands;
    this.conditions = conditions;
    initialized = new boolean[commands.length];
    Arrays.fill(initialized, false);
    for (Command c : commands) {
      addRequirements(c.getRequirements());
    }
  }

  private int chooseCommand() {
    for (int i = 0; i < conditions.length; i++) { // for each case
      if (conditions[i].getAsBoolean()) {
        return i; // return first condition met; highest priority
      }
    }
    return commands.length - 1; // default command
  }

  public void initialize() {
    // pick the command to start
    int comDex = chooseCommand();
    // set lastCommandDex so we know if we changed selection later
    lastCommandDex = comDex;
    // start that command
    commands[comDex].initialize();
  }

  public void execute() {
    int comDex = chooseCommand();
    if (lastCommandDex != comDex) { // change command
      commands[lastCommandDex].cancel();
      commands[comDex].initialize();
    }
    lastCommandDex = comDex;
    commands[comDex].execute();
  }

  public void end(boolean interrupted) {
    for (Command com : commands) {
      com.end(interrupted);
    }
  }

  public boolean isFinished() {
    return commands[lastCommandDex].isFinished();
  }

  public boolean runsWhenDisabled() {
    for (BooleanSupplier condition : conditions) {
      if (!condition.getAsBoolean()) {
        return false;
      }
    }
    return true;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    for (Command c : commands) {
      if (c.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
        return InterruptionBehavior.kCancelSelf;
      }
    }
    return InterruptionBehavior.kCancelIncoming;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    for (int i = 0; i < commands.length; i++) {
      builder.addStringProperty("command" + i, commands[i]::getName, null);
    }
    builder.addStringProperty(
        "selected",
        () -> {
          if (lastCommandDex == -1) {
            return "null";
          } else {
            return commands[lastCommandDex].getName();
          }
        },
        null);
  }
}
