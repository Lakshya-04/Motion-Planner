/**
 * @file force_limiter.hpp
 * @brief A hard contact-force bound as a control barrier on the velocity along the wall normal.
 * @details Pressing on a wall with stiffness k raises the force at the rate F' = k v, where v is
 *          the tool's velocity towards the wall. Requiring F' <= alpha (F_max - F) keeps the set
 *          {F <= F_max} forward invariant: the force approaches the limit exponentially and cannot
 *          cross it, and above the limit the velocity is forced to retreat. The barrier condition
 *          gives a velocity bound
 *
 *              v_max = alpha (F_max - F) / k
 *
 *          and the limiter returns min(v_cmd, v_max) once the tool is in contact (before contact it
 *          caps the approach speed at `free_space_speed_mps`). The wall stiffness is estimated online
 *          (recursive least squares on dF = k dx with forgetting) while the tool is in contact and
 *          moving; until an estimate exists it assumes the stiffest wall (k_max), which is the
 *          conservative choice (a stiffer wall allows a smaller velocity). Idea after the control
 *          barrier approach of arXiv 2405.04859 ("Guarding Force"), reduced to the 1-D normal axis;
 *          no code from that work is used.
 *
 *          Limits: one axis (the wall normal), a linear spring contact model, and no energy tank.
 *          It bounds the force the tool is told to build; it does not replace the tool's abort
 *          threshold or an e-stop. Plain C++, no ROS, no clock.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

namespace force_core
{

struct ForceLimiterParams
{
  double f_max_n{15.0};            ///< Hard limit on the normal force, N, > 0.
  double alpha_per_s{20.0};        ///< Barrier gain: how fast the force may close in on the limit, 1/s.
  double k_min_n_per_m{500.0};     ///< Stiffness estimate is kept within [k_min, k_max].
  double k_max_n_per_m{50000.0};   ///< Also the value used before any estimate exists.
  double forgetting{0.98};         ///< RLS forgetting factor in (0, 1].
  double contact_force_n{0.5};     ///< In contact when the force is at least this.
  double min_step_m{2e-5};         ///< Ignore position steps smaller than this (no excitation).
  /// Approach speed cap before contact is felt (force below contact_force_n). The first contact
  /// builds about k * speed * dt of force per control step, so pick it from the loop rate and the
  /// stiffest wall expected (50 kN/m, 1 kHz, 0.02 m/s gives 1 N a step).
  double free_space_speed_mps{0.02};
};

class ForceLimiter
{
public:
  /// @throws std::invalid_argument if a parameter is out of range.
  explicit ForceLimiter(const ForceLimiterParams & params = {});

  /// Feed one sample: tool position along the wall normal pointing INTO the wall (m, increases as
  /// the tool presses deeper) and the measured normal force (N, positive pressing).
  void update(double normal_pos_m, double force_n);

  /// Largest allowed velocity into the wall (m/s) at this force: `free_space_speed_mps` while no
  /// contact is felt, the barrier bound `alpha (F_max - F) / k` in contact (negative above the limit).
  double maxApproachVelocity(double force_n) const;

  /// min(commanded, maxApproachVelocity): `v_cmd` is positive into the wall.
  double limit(double v_cmd, double force_n) const;

  double stiffness() const {return k_;}
  bool hasEstimate() const {return has_estimate_;}
  void reset();

private:
  ForceLimiterParams p_;
  double k_;
  double cov_;
  bool has_estimate_{false};
  bool have_prev_{false};
  double prev_pos_{0.0};
  double prev_force_{0.0};
};

/// Parameters of the position-level stage that wraps ForceLimiter for a controller that commands a position.
struct NormalAxisLimiterParams
{
  ForceLimiterParams limiter{};
  /// Largest compliance offset (commanded minus reference) into the wall, m. A tight clamp: the
  /// admittance offset cannot wind up past it, so a retreat releases within this distance.
  double max_offset_m{0.05};
  /// A reference moving back out of the wall faster than this (m/s) while the offset is into the wall
  /// is a retreat: the offset cap drops to zero and `resetRequested()` asks the controller to clear
  /// its admittance state, so the release is immediate instead of the integrator's slow relaxation.
  double retreat_speed_mps{0.005};
  /// Before contact is felt, a planned reference advance up to this speed (m/s) is passed through; only
  /// the compliance offset's own advance is held to `limiter.free_space_speed_mps`. Throttling the
  /// planned approach itself makes the command lag the trajectory by metres (found in the MuJoCo run).
  /// Above this speed the approach is capped to it.
  double reference_speed_cap_mps{0.1};
  /// Back-calculation gain, N per m of offset beyond `max_offset_m`: the controller subtracts
  /// `windupForce()` from the pressing force it feeds the admittance rule so its own offset state
  /// stops growing while the output is clamped.
  double windup_gain_n_per_m{1000.0};
};

/// Anti-windup and force barrier on the tool's position along the wall normal, for a controller that
/// outputs `desired = reference + compliance offset`. Per cycle:
///   target = min(desired, reference + max_offset)               (offset clamp, 0 on a retreat)
///   cmd    = min(target, previous cmd + v_max(F) dt)            (force barrier on the velocity;
///            before contact v_max is the larger of the offset speed cap and the planned reference advance)
/// The command follows a retreat at once (nothing is integrated below the target) and a detected
/// retreat also asks for the admittance state to be reset; the barrier only ever lags the approach. Plain C++, no ROS, no clock.
class NormalAxisLimiter
{
public:
  /// @throws std::invalid_argument if a parameter is out of range.
  explicit NormalAxisLimiter(const NormalAxisLimiterParams & params = {});

  /// Positions are along the wall normal pointing INTO the wall (m); `force_n` is the measured
  /// pressing force (N, positive pressing); `dt` > 0 in s. Returns the position to command.
  double step(double dt, double p_desired, double p_reference, double p_measured, double force_n);

  /// Force (N, >= 0) to take away from the pressing force fed to the admittance rule this cycle.
  double windupForce() const {return windup_force_n_;}
  /// True on a cycle that detected a retreat: the caller should reset its admittance offset state.
  bool resetRequested() const {return reset_requested_;}
  const ForceLimiter & limiter() const {return limiter_;}
  void reset();

private:
  NormalAxisLimiterParams p_;
  ForceLimiter limiter_;
  bool have_prev_{false};
  double prev_cmd_{0.0};
  double windup_force_n_{0.0};
  bool reset_requested_{false};
  bool have_prev_ref_{false};
  double prev_ref_{0.0};
};

}  // namespace force_core
