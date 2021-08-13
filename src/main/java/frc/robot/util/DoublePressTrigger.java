// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Trigger that activates only when a button is pressed twice quickly.
 */
public class DoublePressTrigger extends Trigger {
  private static final double maxLengthSecs = 0.7; // How long after the first press does the second need to occur?

  private final Trigger trigger;
  private final Timer resetTimer = new Timer();
  private DoublePressState state = DoublePressState.IDLE;

  public DoublePressTrigger(Trigger baseTrigger) {
    trigger = baseTrigger;
  }

  @Override
  public boolean get() {
    boolean pressed = trigger.get();
    switch (state) {
      case IDLE:
        if (pressed) {
          state = DoublePressState.FIRST_PRESS;
          resetTimer.reset();
          resetTimer.start();
        }
        break;
      case FIRST_PRESS:
        if (!pressed) {
          if (resetTimer.hasElapsed(maxLengthSecs)) {
            reset();
          } else {
            state = DoublePressState.FIRST_RELEASE;
          }
        }
        break;
      case FIRST_RELEASE:
        if (pressed) {
          state = DoublePressState.SECOND_PRESS;
        } else if (resetTimer.hasElapsed(maxLengthSecs)) {
          reset();
        }
        break;
      case SECOND_PRESS:
        if (!pressed) {
          reset();
        }
    }
    return state == DoublePressState.SECOND_PRESS;
  }

  private void reset() {
    state = DoublePressState.IDLE;
    resetTimer.stop();
  }

  private enum DoublePressState {
    IDLE, FIRST_PRESS, FIRST_RELEASE, SECOND_PRESS
  }
}
