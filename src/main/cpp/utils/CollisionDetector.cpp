// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/CollisionDetector.h"
#include <cmath>

CollisionDetector::CollisionDetector(double currentThreshold,
                                     double confirmationTime,
                                     double velocityDropRatio)
    : m_currentThreshold(currentThreshold),
      m_confirmationTime(confirmationTime),
      m_velocityDropRatio(velocityDropRatio) {}

bool CollisionDetector::Update(double currentAmps, double commandedVelocity,
                               double actualVelocity) {
  // Minimum commanded velocity to consider (avoid false positives when stopped)
  constexpr double kMinCommandedVelocity = 0.1;  // m/s

  bool potentialCollision = false;

  // Check if commanding significant motion
  if (std::abs(commandedVelocity) > kMinCommandedVelocity) {
    // Check if current exceeds threshold
    if (currentAmps > m_currentThreshold) {
      // Check if velocity has dropped significantly
      double velocityRatio = std::abs(actualVelocity) / std::abs(commandedVelocity);
      if (velocityRatio < m_velocityDropRatio) {
        potentialCollision = true;
      }
    }
  }

  // State machine for confirmation
  if (potentialCollision) {
    if (!m_inCollisionState) {
      // Just entered potential collision state
      m_inCollisionState = true;
      m_collisionTimer.Reset();
      m_collisionTimer.Start();
    } else {
      // Already in collision state, check if confirmed
      if (m_collisionTimer.Get().value() >= m_confirmationTime) {
        m_collisionDetected = true;
      }
    }
  } else {
    // Not in collision state
    m_inCollisionState = false;
    m_collisionTimer.Stop();
    m_collisionTimer.Reset();
  }

  return m_collisionDetected;
}

void CollisionDetector::Reset() {
  m_inCollisionState = false;
  m_collisionDetected = false;
  m_collisionTimer.Stop();
  m_collisionTimer.Reset();
}

double CollisionDetector::GetCollisionStateTime() const {
  if (m_inCollisionState) {
    return m_collisionTimer.Get().value();
  }
  return 0.0;
}
