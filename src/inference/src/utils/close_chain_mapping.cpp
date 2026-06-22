// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2025-2026 Luo1imasi

#include "close_chain_mapping.hpp"
#include "decouple_rpo.hpp"

std::shared_ptr<Decouple> Decouple::create(const std::string &type)
{
    if (type == "rpo")
    {
        return std::make_shared<DecoupleRPO>();
    }
    else if (type == "mevius2")
    {
        // mevius2 is a 12-DoF quadruped with no parallel (close-chain) ankle
        // linkage, so no decoupling is required. Returning nullptr is safe: every
        // close-chain code path in robot_interface is guarded by
        // `!close_chain_joint_idx_.empty() && ankle_decouple_`, and mevius2's
        // close_chain_motor_idx is empty.
        return nullptr;
    }
    else
    {
        throw std::runtime_error("Unknown close_chain type: " + type +
                                 ". Supported types: rpo, mevius2");
    }
}
