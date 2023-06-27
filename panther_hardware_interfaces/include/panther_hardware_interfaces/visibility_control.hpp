
// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef PANTHER_HARDWARE_INTERFACES__VISIBILITY_CONTROL_H_
#define PANTHER_HARDWARE_INTERFACES__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PANTHER_HARDWARE_INTERFACES_EXPORT __attribute__((dllexport))
#define PANTHER_HARDWARE_INTERFACES_IMPORT __attribute__((dllimport))
#else
#define PANTHER_HARDWARE_INTERFACES_EXPORT __declspec(dllexport)
#define PANTHER_HARDWARE_INTERFACES_IMPORT __declspec(dllimport)
#endif
#ifdef PANTHER_HARDWARE_INTERFACES_BUILDING_DLL
#define PANTHER_HARDWARE_INTERFACES_PUBLIC PANTHER_HARDWARE_INTERFACES_EXPORT
#else
#define PANTHER_HARDWARE_INTERFACES_PUBLIC PANTHER_HARDWARE_INTERFACES_IMPORT
#endif
#define PANTHER_HARDWARE_INTERFACES_PUBLIC_TYPE PANTHER_HARDWARE_INTERFACES_PUBLIC
#define PANTHER_HARDWARE_INTERFACES_LOCAL
#else
#define PANTHER_HARDWARE_INTERFACES_EXPORT __attribute__((visibility("default")))
#define PANTHER_HARDWARE_INTERFACES_IMPORT
#if __GNUC__ >= 4
#define PANTHER_HARDWARE_INTERFACES_PUBLIC __attribute__((visibility("default")))
#define PANTHER_HARDWARE_INTERFACES_LOCAL __attribute__((visibility("hidden")))
#else
#define PANTHER_HARDWARE_INTERFACES_PUBLIC
#define PANTHER_HARDWARE_INTERFACES_LOCAL
#endif
#define PANTHER_HARDWARE_INTERFACES_PUBLIC_TYPE
#endif

#endif  // PANTHER_HARDWARE_INTERFACES__VISIBILITY_CONTROL_H_
