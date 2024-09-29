#pragma once

#include "mpc/LeggedMpc.h"
#include "utils/MovingWindowFilter.hpp"

namespace legged {
    using namespace altro;
    class ConvexMpc : public LeggedMpc {
    public:
        ConvexMpc() {};
        ConvexMpc(LeggedState &state);
        bool update(LeggedState &state) override;
        bool goal_update(LeggedState &state) override;
        bool grf_update(LeggedState &state) override;
        bool foot_update(LeggedState &state) override;
        bool terrain_update(LeggedState &state) override;
    private:
        MovingWindowFilter terrain_angle_filter;
    };
}  // namespace legged
