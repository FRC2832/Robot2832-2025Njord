package org.livoniawarriors.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.LinkedList;
import java.util.function.BooleanSupplier;

/**
 * A class to be used to build a {@link SwitchCommand}, a command based on the concept of if-else
 * chains. <br>
 * <br>
 * To create one, construct a new {@code SwitchCommandBuilder}, then call withCase(), passing the
 * command and a BooleanSupplier for when to run the command. Cases are prioritized in order of
 * first to last registered, so the conditions can be set up on the assumption that earlier cases
 * had false conditions. {@code withCase()} supports method chaining. Finally, to complete the
 * {@code SwitchCommand}, call {@code withDefault()} with the command to run when all conditions are
 * false. <br>
 * <br>
 * When the command to run changes, either because a higher priority condition becomes true, or the
 * condition for the current command becomes false, the current command is cancelled, and the new
 * one initialized.
 *
 * <p>As an example, consider this simple autonomous example:
 *
 * <pre>
 * {@code SwitchCommand auton = new SwitchCommandBuilder()
 * .withCase(() -> hasPiece, new ScorePieceCommand())
 * .withCase(() -> autonTimeRemaining > 3, new CollectPieceCommand())
 * .withDefault(new ParkCommand())
 * }</pre>
 */
public class SwitchCommandBuilder {
  LinkedList<BooleanSupplier> conditions;
  LinkedList<Command> commands;

  public SwitchCommandBuilder() {
    conditions = new LinkedList<>();
    commands = new LinkedList<>();
  }

  public SwitchCommandBuilder withCase(BooleanSupplier condition, Command command) {
    conditions.add(condition);
    commands.add(command);
    return this;
  }

  public Command withDefault(Command command) {
    commands.add(command);
    BooleanSupplier[] conds = new BooleanSupplier[conditions.size()];
    Command[] comms = new Command[commands.size()];
    conds = conditions.toArray(conds);
    comms = commands.toArray(comms);
    return new SwitchCommand(conds, comms);
  }
}
