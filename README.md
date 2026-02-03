# MCL-Lemlib

This project integrates **Monte Carlo Localization (MCL)** with **LemLib** for high-precision robot positioning on VEX V5 hardware. It uses a particle filter to fuse odometry data with distance sensor readings, correcting for drift and ensuring the robot knows its absolute position on the field.

## What is MCL?

Monte Carlo Localization (MCL), also known as a Particle Filter, is a probabilistic algorithm used to estimate the position and orientation of a robot. 

Instead of tracking a single x, y, theta coordinate (like standard odometry), MCL maintains a cloud of thousands of "particles". Each particle represents a *possible* location of the robot. This allows the system to:
1.  **Handle Noise:** Account for wheel slip and sensor inaccuracies.
2.  **Correct Drift:** Use distance sensors to "snap" the position estimate back to reality by measuring the distance to field walls.
3.  **Recover from Errors:** If the robot gets pushed or "kidnapped", the filter can recover its true position over time.

## Key Features

*   **Seamless LemLib Integration:** Works directly with `lemlib::Pose` and standard VEX PROS peripherals.
*   **Augmented MCL:** Includes "kidnapped robot" recovery by injecting random particles when the filter detects low confidence.
*   **Confidence Tracking:** precise `getConfidence()` metric to know when the robot is lost or localized.
*   **Discrete Bayes Filter:** Includes a separate utility for tracking categorical states (e.g., "Am I stuck?", "Which game object is this?").

## API Documentation

The core functionality is provided by the `mcl::MonteCarlo` class in `include/mcl.hpp`.

### `mcl::MonteCarlo<size_t N>`

The main class for the particle filter. `N` is the number of particles (e.g., `10000`).

#### Constructor
```cpp
MonteCarlo(lemlib::Pose initialPose, double driveNoise, double turnNoise)
```
*   `initialPose`: The starting position of the robot.
*   `driveNoise`: Standard deviation of drivetrain noise (in inches).
*   `turnNoise`: Standard deviation of turning noise (in radians).

#### `void setPose(lemlib::Pose pose)`
Resets the filter to a specific pose with a small Gaussian spread. Use this at the start of a match or when the position is known.

#### `void predict(double deltaX, double deltaY, double deltaTheta)`
The **Prediction Step**. Updates all particles based on the robot's movement (odometry).
*   `deltaX`, `deltaY`: Change in position (local coordinates).
*   `deltaTheta`: Change in heading (radians).

#### `void update(const std::vector<double> &readings, const std::vector<lemlib::Pose> &offsets, double sensorStdDev)`
The **Update Step**. Re-weighs particles based on how well they match sensor data.
*   `readings`: Vector of distances measured by sensors (inches).
*   `offsets`: Vector of poses defining where sensors are relative to the robot center.
*   `sensorStdDev`: Expected noise of the distance sensors (e.g., `1.5` inches).

#### `void resample()`
The **Resampling Step**. Removes low-weight particles and duplicates high-weight ones. Automatically injects random particles if confidence is low.

#### `lemlib::Pose getPose() const`
Returns the weighted average of all particles, representing the robot's best estimated position.

#### `double getConfidence() const`
Returns a value between `0.0` and `1.0` representing how certain the filter is of its location.

---

### `mcl::DiscreteBayesFilter`

A general-purpose filter for tracking discrete states.

#### `void update(const std::vector<double> &likelihoods)`
Updates beliefs based on observation likelihoods.

#### `void predict(const std::vector<std::vector<double>> &transitionMatrix)`
Updates beliefs based on a state transition probability matrix.

#### `double getBelief(size_t state) const`
Returns the probability (0.0 - 1.0) of being in a specific state.

## Example Usage

See `src/main.cpp` for a full implementation.

1.  **Define and Initialize:**
    ```cpp
    // 10,000 particles
    mcl::MonteCarlo<10000> mclFilter(lemlib::Pose(0, 0, 0), 0.05, 0.01);
    ```

2.  **Run a Background Task:**
    Create a detailed loop (like `mclTask` in `src/main.cpp`) that:
    *   Calculates odometry changes.
    *   Calls `mclFilter.predict()`.
    *   Periodically reads distance sensors.
    *   Calls `mclFilter.update()` and `mclFilter.resample()`.
    *   Updates the Chassis pose with the new estimate.

3.  **Wait for Confidence in Auton:**
    ```cpp
    // Block until 95% confident
    waitForMCLConfidence(0.95, 3000); 
    chassis.moveToPoint(...);
    ```
