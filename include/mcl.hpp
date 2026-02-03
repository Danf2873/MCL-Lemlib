#pragma once

#include "lemlib/pose.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace mcl {

/**
 * @brief Struct representing a single particle in the filter
 */
struct Particle {
  lemlib::Pose pose;
  double weight;
};

/**
 * @brief Monte Carlo Localization class
 *
 * @tparam N The number of particles to use
 */
template <size_t N> class MonteCarlo {
public:
  /**
   * @brief Construct a new Monte Carlo object
   *
   * @param initialPose The starting pose of the robot
   * @param driveNoise The standard deviation of the drivetrain movement noise
   * (inches)
   * @param turnNoise The standard deviation of the turning noise (radians)
   */
  MonteCarlo(lemlib::Pose initialPose, double driveNoise, double turnNoise)
      : driveNoise(driveNoise), turnNoise(turnNoise), wSlow(0), wFast(0) {
    std::random_device rd;
    gen = std::mt19937(rd());
    setPose(initialPose);
  }

  /**
   * @brief Set the current pose of the robot and reset particles around it
   *
   * @param pose
   */
  void setPose(lemlib::Pose pose) {
    particles.clear();
    std::normal_distribution<double> distX(pose.x,
                                           2.0); // 2 inch initial spread
    std::normal_distribution<double> distY(pose.y, 2.0);
    std::normal_distribution<double> distTheta(pose.theta,
                                               0.05); // ~3 degrees spread

    for (size_t i = 0; i < N; ++i) {
      particles.push_back(
          {lemlib::Pose(distX(gen), distY(gen), distTheta(gen)), 1.0 / N});
    }
  }

  /**
   * @brief Update the particles based on robot movement (Prediction Step)
   *
   * @param deltaX Movement in local X direction
   * @param deltaY Movement in local Y direction
   * @param deltaTheta Change in heading (radians)
   */
  void predict(double deltaX, double deltaY, double deltaTheta) {
    std::normal_distribution<double> distMove(0, driveNoise);
    std::normal_distribution<double> distTurn(0, turnNoise);

    for (auto &p : particles) {
      double noisyDeltaX = deltaX + distMove(gen);
      double noisyDeltaY = deltaY + distMove(gen);
      double noisyDeltaTheta = deltaTheta + distTurn(gen);

      p.pose.x += noisyDeltaX;
      p.pose.y += noisyDeltaY;
      p.pose.theta += noisyDeltaTheta;
    }
  }

  /**
   * @brief Correct particle weights based on sensor readings (Update Step)
   *
   * @param sensorReadings Vector of actual distance sensor readings
   * @param sensorOffsets Vector of poses representing sensor
   * positions/orientations relative to robot center
   * @param sensorStandardDeviation Expected noise of the distance sensor
   */
  void update(const std::vector<double> &sensorReadings,
              const std::vector<lemlib::Pose> &sensorOffsets,
              double sensorStandardDeviation) {
    double totalWeight = 0;

    for (auto &p : particles) {
      double likelihood = 1.0;
      for (size_t i = 0; i < sensorReadings.size(); ++i) {
        double expected = getExpectedDistance(p.pose, sensorOffsets[i]);
        double actual = sensorReadings[i];

        // Gaussian likelihood with clipping to prevent weight explosion
        double diff = actual - expected;
        double exponent = -(diff * diff) / (2 * sensorStandardDeviation *
                                           sensorStandardDeviation);
        // Clamp exponent to prevent overflow: exp(-x) where x > 100 ≈ 0
        exponent = std::max(-100.0, exponent);
        likelihood *= std::exp(exponent);
      }
      p.weight *= likelihood;
      totalWeight += p.weight;
    }

    // Bayesian confidence tracking (Augmented MCL)
    // We track a slow and fast moving average of the total weight
    double avgWeight = totalWeight / N;
    if (wSlow == 0) {
      wSlow = avgWeight;
      wFast = avgWeight; // Initialize wFast along with wSlow
    } else {
      wSlow += alphaSlow * (avgWeight - wSlow);
      wFast += alphaFast * (avgWeight - wFast);
    }

    // Normalize weights
    if (totalWeight > 1e-9) {
      for (auto &p : particles) {
        p.weight /= totalWeight;
      }
    } else {
      // If all weights are zero, reset to uniform (or handle kidnap)
      for (auto &p : particles) {
        p.weight = 1.0 / N;
      }
    }
  }

  /**
   * @brief Resample particles based on their weights (Importance Sampling)
   * Also injects random particles if the Bayes filter detects the robot is lost
   */
  void resample() {
    std::vector<Particle> newParticles;
    std::vector<double> weights;
    for (const auto &p : particles)
      weights.push_back(p.weight);

    std::discrete_distribution<size_t> dist(weights.begin(), weights.end());

    // Calculate probability of adding a random particle
    // Based on the ratio of fast and slow moving weight averages
    double pRandom = std::max(0.0, 1.0 - wFast / wSlow);

    std::uniform_real_distribution<double> randProb(0.0, 1.0);
    std::uniform_real_distribution<double> fieldX(-72.0, 72.0);
    std::uniform_real_distribution<double> fieldY(-72.0, 72.0);
    std::uniform_real_distribution<double> fieldTheta(0, 360);

    for (size_t i = 0; i < N; ++i) {
      if (randProb(gen) < pRandom) {
        // Inject a random particle (Kidnapped robot recovery)
        newParticles.push_back(
            {lemlib::Pose(fieldX(gen), fieldY(gen), fieldTheta(gen)), 1.0 / N});
      } else {
        newParticles.push_back(particles[dist(gen)]);
      }
      newParticles.back().weight = 1.0 / N;
    }

    particles = newParticles;
  }

  /**
   * @brief Get the estimated pose of the robot
   *
   * @return lemlib::Pose
   */
  lemlib::Pose getPose() const {
    double avgX = 0, avgY = 0, avgThetaX = 0, avgThetaY = 0;

    for (const auto &p : particles) {
      avgX += p.pose.x * p.weight;
      avgY += p.pose.y * p.weight;
      // Average angles using vector components to handle wrap-around
      double rad = p.pose.theta * M_PI / 180.0;
      avgThetaX += cos(rad) * p.weight;
      avgThetaY += sin(rad) * p.weight;
    }

    // Normalize angle average and convert back to degrees
    double finalTheta = atan2(avgThetaY, avgThetaX) * 180.0 / M_PI;
    // Ensure angle is in [0, 360) range for consistency
    while (finalTheta < 0)
      finalTheta += 360.0;
    while (finalTheta >= 360.0)
      finalTheta -= 360.0;

    return lemlib::Pose(avgX, avgY, finalTheta);
  }

  /**
   * @brief Get the localization confidence (0.0 to 1.0)
   * This is calculated using the Bayesian weight averages.
   *
   * @return double
   */
  double getConfidence() const {
    if (wSlow == 0 || wFast == 0)
      return 1.0; // Both uninitialized, max confidence
    return std::min(1.0, std::max(0.0, wFast / wSlow));
  }

private:
  std::vector<Particle> particles;
  double driveNoise;
  double turnNoise;

  // Augmented MCL Bayesian tracking variables
  double wSlow;
  double wFast;
  const double alphaSlow = 0.05;
  const double alphaFast = 0.2;

  // Internal random number generator
  mutable std::mt19937 gen;

  /**
   * @brief Calculate the expected distance from a pose and sensor offset to the
   * field wall
   *
   * @param robotPose
   * @param sensorOffset
   * @return double
   */
  double getExpectedDistance(lemlib::Pose robotPose,
                             lemlib::Pose sensorOffset) {
    // LemLib uses: 0° = Forward (Y+), Clockwise = Positive
    // Convert to standard math coordinates for calculation
    double robotAngleRad = (90.0 - robotPose.theta) * M_PI / 180.0;
    double sensorAngleRad =
        (90.0 - (robotPose.theta + sensorOffset.theta)) * M_PI / 180.0;

    // Transform sensor offset from robot-local to global coordinates
    double sensorGlobalX = robotPose.x + sensorOffset.x * cos(robotAngleRad) -
                           sensorOffset.y * sin(robotAngleRad);
    double sensorGlobalY = robotPose.y + sensorOffset.x * sin(robotAngleRad) +
                           sensorOffset.y * cos(robotAngleRad);

    // Raycasting Algorithm: Find closest wall intersection
    // Ray equation: P(t) = Origin + t * Direction, where t > 0
    double dirX = cos(sensorAngleRad);
    double dirY = sin(sensorAngleRad);

    double minDistance = 1e9;

    // Field boundaries (VEX 144" x 144" field)
    const double FIELD_MIN = -72.0;
    const double FIELD_MAX = 72.0;

    // Helper lambda for ray-line segment intersection
    auto rayIntersect = [&](double wallX1, double wallY1, double wallX2,
                            double wallY2) {
      // Wall direction vector
      double wallDX = wallX2 - wallX1;
      double wallDY = wallY2 - wallY1;

      // Solve: Origin + t*Dir = Wall1 + s*WallDir
      // Using Cramer's rule for 2x2 system
      double denom = dirX * wallDY - dirY * wallDX;

      if (std::abs(denom) < 1e-9) {
        // Ray parallel to wall - no intersection or infinite intersections
        return;
      }

      double t = ((wallX1 - sensorGlobalX) * wallDY -
                  (wallY1 - sensorGlobalY) * wallDX) /
                 denom;
      double s =
          ((wallX1 - sensorGlobalX) * dirY - (wallY1 - sensorGlobalY) * dirX) /
          denom;

      // Check if intersection is valid (t > 0 means forward, 0 <= s <= 1 means
      // on wall segment)
      if (t > 1e-6 && s >= 0 && s <= 1.0) {
        minDistance = std::min(minDistance, t);
      }
    };

    // Cast ray against all four walls
    // Bottom wall (Y = -72), from left to right
    rayIntersect(FIELD_MIN, FIELD_MIN, FIELD_MAX, FIELD_MIN);
    // Right wall (X = 72), from bottom to top
    rayIntersect(FIELD_MAX, FIELD_MIN, FIELD_MAX, FIELD_MAX);
    // Top wall (Y = 72), from right to left
    rayIntersect(FIELD_MAX, FIELD_MAX, FIELD_MIN, FIELD_MAX);
    // Left wall (X = -72), from top to bottom
    rayIntersect(FIELD_MIN, FIELD_MAX, FIELD_MIN, FIELD_MIN);

    // If minDistance is still 1e9, the sensor is outside field boundaries
    return (minDistance < 1e8) ? minDistance : 72.0;
  }
};

/**
 * @brief Standalone Discrete Bayes Filter for state estimation
 * Helpful for tracking categorical states like "Am I stuck?" or "Which object
 * is this?"
 */
class DiscreteBayesFilter {
public:
  DiscreteBayesFilter(size_t numStates) : beliefs(numStates, 1.0 / numStates) {}

  /**
   * @brief Update belief with a measurement
   *
   * @param likelihoods Vector of P(z | state) for each state
   */
  void update(const std::vector<double> &likelihoods) {
    double total = 0;
    for (size_t i = 0; i < beliefs.size(); ++i) {
      beliefs[i] *= (i < likelihoods.size()) ? likelihoods[i] : 1e-6;
      total += beliefs[i];
    }
    // Normalize
    for (double &b : beliefs)
      b /= total;
  }

  /**
   * @brief Apply a transition (Prediction Step)
   *
   * @param transitionMatrix Matrix where [i][j] is P(state_j | state_i)
   */
  void predict(const std::vector<std::vector<double>> &transitionMatrix) {
    std::vector<double> nextBeliefs(beliefs.size(), 0.0);
    for (size_t j = 0; j < beliefs.size(); ++j) {
      for (size_t i = 0; i < beliefs.size(); ++i) {
        nextBeliefs[j] += beliefs[i] * transitionMatrix[i][j];
      }
    }
    beliefs = nextBeliefs;
  }

  double getBelief(size_t state) const { return beliefs[state]; }

private:
  std::vector<double> beliefs;
};

} // namespace mcl
