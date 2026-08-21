- get_encoder_value() truncates get_data() instead of rounding it, so every position reads up to
  1% low. Use lroundf() if that precision is ever needed.

- UI: let the user skip to the next step while a program is running, instead of waiting out
  the current one. Needs a new action/ topic plus a ProgramManager handler that jumps state
  straight to STEP_COMPLETED for the current step.
