#!/usr/bin/env bash
# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Loads the per-unit-type ASCII art banners from include/menus/art/*.sh
# (one print_unit_art_<key>() function per file, keyed the same way as
# include/units/registry.sh's UNIT_KEY) and dispatches on installation_type.
#
# To add art for a new unit type, drop a include/menus/art/<key>.sh file
# defining print_unit_art_<key>(), where <key> matches the UNIT_KEY entry
# for that type in include/units/registry.sh. No changes needed here.

for _art_file in "$(dirname "${BASH_SOURCE[0]}")/art/"*.sh; do
  # shellcheck source=/dev/null
  source "$_art_file"
done
unset _art_file

print_unit_art() {
  # Called as a bare statement under `set -e` by the menu loops, so a
  # missing art file (unknown type, or a new type that skipped step 2 in
  # registry.sh's "how to add a new unit type") must return 0 rather than
  # the failure status of a missing `declare -F`, or the whole script exits.
  local key fn
  key="$(unit_key_for_type "$1")"
  [ -n "$key" ] || return 0
  fn="print_unit_art_${key}"
  if declare -F "$fn" >/dev/null; then
    "$fn"
  fi
}
