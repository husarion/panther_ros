// Copyright 2024 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PANTHER_MANAGER_TYPES_HPP_
#define PANTHER_MANAGER_TYPES_HPP_

namespace panther_manager
{

enum DockingCmd : unsigned {
  DOCKING_CMD_NONE = 0U,
  DOCKING_CMD_DOCK = 1U,
  DOCKING_CMD_UNDOCK = 2U,
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_TYPES_HPP_
