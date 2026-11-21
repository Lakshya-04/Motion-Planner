/**
 * @file contact_free_gate.hpp
 * @brief Decides when the tool is verified free of contact, for the F/T bias tracker.
 * @details The bias tracker only learns while the caller states `free_of_contact`. This gate produces
 *          that flag from the force FSM state and the conditioned force magnitude:
 *            - free states are only kIdle and kApproach (contact not yet confirmed). Every contact
 *              state, kRetractSettle/kRetractFollow (the tool may still be touching or just leaving) and
 *              kAborted are never free;
 *            - after entering a free state from a non-free one (for example after a retract completes),
 *              the gate stays not-free for `min_settle_s` so the wrench can ring down;
 *            - hysteresis on the force: becoming free needs |F| < `free_below_n` for `debounce_ticks`
 *              consecutive updates, leaving free is immediate once |F| > `not_free_above_n`;
 *            - a NaN or infinite force, a non-monotonic time, or (optionally) commanded motion is not free.
 *          Plain C++, no clock, no ROS: the caller supplies time. The ft_conditioner node can call
 *          `update` once per wrench sample and publish the result; the node is not wired here.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include "force_core/force_stage_fsm.hpp"

namespace force_core
{

struct ContactFreeGateConfig
{
  double free_below_n{0.5};      ///< Force magnitude under which the tool may become free [N].
  double not_free_above_n{1.0};  ///< Force magnitude above which free is lost at once [N]; >= free_below_n.
  int debounce_ticks{5};         ///< Consecutive low-force updates needed to become free.
  double min_settle_s{1.0};      ///< Time to wait after entering a free state from a non-free one [s].
  bool require_no_motion{false}; ///< If true, commanded motion makes the gate not free.
};

class ContactFreeGate
{
public:
  explicit ContactFreeGate(const ContactFreeGateConfig & config = ContactFreeGateConfig{});

  /// True for the two FSM states in which contact-free can ever hold.
  static bool isFreeState(ForceFsmState state);

  /**
   * @brief One update. @param force_n conditioned force magnitude [N]; @param now_s caller time [s].
   * @return the current contact_free flag.
   */
  bool update(ForceFsmState state, double force_n, double now_s, bool commanded_motion = false);

  bool contactFree() const {return free_;}
  void reset();

private:
  ContactFreeGateConfig cfg_;
  bool free_{false};
  bool in_free_state_{false};
  bool have_time_{false};
  double settle_start_s_{0.0};
  double last_s_{0.0};
  int low_count_{0};
};

}  // namespace force_core
