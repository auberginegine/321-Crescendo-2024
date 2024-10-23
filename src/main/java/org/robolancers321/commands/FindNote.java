/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.subsystems.drivetrain.Drivetrain;

public class FindNote extends SequentialCommandGroup {
  private Drivetrain drivetrain;

  public FindNote() {
    this.drivetrain = Drivetrain.getInstance();

    addCommands(
        this.drivetrain.driveCommand(0, 0, 0.2, false).until(this.drivetrain::seesNote),
        new AutoPickupNote());
  }
}
