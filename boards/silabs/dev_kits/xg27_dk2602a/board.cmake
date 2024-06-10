# Copyright (c) 2024 Silicon Laboratories Inc.
# SPDX-License-Identifier: Apache-2.0

board_runner_args(silabs_commander "--device=EFR32BG27C140F768IM40")
include(${ZEPHYR_BASE}/boards/common/silabs_commander.board.cmake)
