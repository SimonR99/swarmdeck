// Render scheduling only: never change ray geometry, range, or moving cadence.
#pragma once
#include <cstdint>

class SwarmDeckLidarSchedule {
public:
   bool IsDue(std::uint32_t tick, std::uint32_t phase,
              std::uint32_t active_divider, std::uint32_t parked_divider,
              std::uint32_t settle_ticks, bool moved) {
      if(!m_initialized || tick < m_previous_tick || moved) {
         m_last_motion_tick = tick;
      }
      m_initialized = true;
      m_previous_tick = tick;
      const bool parked = parked_divider > active_divider &&
                          tick - m_last_motion_tick >= settle_ticks;
      const auto divider = parked ? parked_divider : active_divider;
      return (tick + phase) % divider == 0;
   }

private:
   bool m_initialized = false;
   std::uint32_t m_previous_tick = 0;
   std::uint32_t m_last_motion_tick = 0;
};
