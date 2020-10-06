// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include <msode/core/factory.h>
#include <msode/core/file_parser.h>
#include <msode/rl/environment.h>

#include <smarties.h>

#include <memory>

void setActionDims  (const msode::rl::MSodeEnvironment *env, smarties::Communicator *const comm);
void setActionBounds(const msode::rl::MSodeEnvironment *env, smarties::Communicator *const comm);
void setStateBounds (const msode::rl::MSodeEnvironment *env, smarties::Communicator *const comm);
