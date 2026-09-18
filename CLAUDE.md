# CLAUDE.md

Guidance for Claude Code when working in this repository.

## Language policy

- **Everything that goes into the repository or onto GitHub must be written
  in English**: code, comments, commit messages, file/branch names, PR
  titles and descriptions, PR review comments, issue comments, and
  documentation files (e.g. under `documentation/`, `README.md`,
  `release_creation.md`).
- This applies **regardless of the language used in the conversation** with
  the person requesting the change. Conversation in this chat/session often
  happens in German — that's fine and does not change the rule above.
- If asked to write a commit message, PR, doc, or comment based on a German
  request or discussion, translate the substance into English rather than
  quoting the German text verbatim.

## Comment style

- Prefer short comments. A comment should capture the non-obvious WHY (a
  hidden constraint, a hardware quirk, a subtle invariant) in a line or
  two, not a multi-paragraph essay. If a fact belongs anywhere, prefer
  `documentation/wlan_link.md` (or another doc file) and a short pointer
  comment over inlining the full story in the source.
- Don't restate what the code already makes obvious from names/structure.

## Related repositories

- [`larus-breeze/sw_tools`](https://github.com/larus-breeze/sw_tools) — sibling
  repository with the generic firmware-update tooling used by this project's
  SD-card update mechanism (`sw_stm32/scripts/pack.py`,
  `sw_stm32/Communication/uSD_helpers.cpp::read_software_update()`):
  - [`sw_update/`](https://github.com/larus-breeze/sw_tools/tree/master/sw_update) —
    source of the "copy routine" that this repo only vendors as a prebuilt
    `.elf` (`sw_stm32/scripts/copy_stm32f407_1m-806.elf`), plus the generic
    packer script and the meta-data header format it and `pack.py` share.
    See `documentation/wlan_link.md` for details on how this fits
    together.
- [`larus-breeze/sw_algorithms_lib`](https://github.com/larus-breeze/sw_algorithms_lib) —
  the `sw_stm32/lib` git submodule (see `.gitmodules`). Not checked out by
  default in a shallow/partial clone of this repo, but readable directly at
  its own URL — clone it separately when a question needs its source (flight
  algorithms, AHRS, navigation, output formatting; e.g. `state_vector_t`,
  `flight_state_t`/`ON_GROUND`, `airborne_detector_t` all live here, see
  `documentation/wlan_link.md`'s flight-state-gating section for
  an example of what was only answerable by reading it directly).
