// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>

/**
 * Utility class to detect collisions/stalls based on motor current spikes.
 * A collision is detected when:
 * - Current exceeds threshold
 * - Robot is commanding motion (commanded velocity > minimum)
 * - Actual velocity has dropped significantly (< velocityDropRatio of commanded)
 * - Condition persists for confirmationTime
 */
class CollisionDetector {
 public:
  /**
   * Constructs a CollisionDetector.
   * @param currentThreshold Current threshold in amps to trigger detection
   * @param confirmationTime Time in seconds the condition must persist
   * @param velocityDropRatio Ratio of actual/commanded velocity below which collision is suspected
   */
  CollisionDetector(double currentThreshold, double confirmationTime,
                    double velocityDropRatio = 0.3);

  /**
   * Updates the collision detection state. Call this every periodic cycle.
   * @param currentAmps Current motor current reading in amps
   * @param commandedVelocity The velocity being commanded (m/s)
   * @param actualVelocity The measured actual velocity (m/s)
   * @return true if collision is confirmed
   */
  bool Update(double currentAmps, double commandedVelocity, double actualVelocity);

  /**
   * Resets the collision detector state.
   */
  void Reset();

  /**
   * Returns true if a collision has been confirmed.
   */
  bool IsCollisionDetected() const { return m_collisionDetected; }

  /**
   * Returns the time spent in the potential collision state (for debugging).
   */
  double GetCollisionStateTime() const;

 private:
  double m_currentThreshold;
  double m_confirmationTime;
  double m_velocityDropRatio;

  frc::Timer m_collisionTimer;
  bool m_inCollisionState = false;
  bool m_collisionDetected = false;
};
