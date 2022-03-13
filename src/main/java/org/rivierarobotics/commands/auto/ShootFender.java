package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.shoot.Shoot;
import org.rivierarobotics.subsystems.vision.Floppas;

public class ShootFender extends SequentialCommandGroup {
    public ShootFender() {
        addCommands(
                new Shoot(Floppas.ShooterLocations.FENDER),
                new Sideways(3)
        );
    }
}
