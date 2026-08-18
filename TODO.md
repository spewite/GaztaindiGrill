- Reject a program when the encoder reading is invalid, instead of anchoring on a stale one.
    * execute_program() captures positionAnchor once and cannot retry, so it reads it through
      get_last_known_position() to be sure it never gets ENCODER_ERROR.
    * That fails silently: if the encoder is dead, it keeps returning an old position and the
      caller has no way to tell.
    * Better: read get_encoder_value() and refuse to start the program on ENCODER_ERROR.
      get_last_known_position() can then be removed.

- lastEncoderValue starts at 0, which is now a real position (bottom of travel) instead of an
  impossible one. Only ambiguous between boot and the first homing, but the relative-mode anchor
  reads it.

- get_encoder_value() truncates get_data() instead of rounding it, so every position reads up to
  1% low. Use lroundf() if that precision is ever needed.
