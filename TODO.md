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

- Request/response channel between the clients and the ESP32 (branch: request-response).
  Today every action/ topic is fire-and-forget: the ESP32 receives eight different commands and has
  no way to answer any of them. That is a structural gap rather than a missing error message, which
  is what makes it worth doing on its own. Correlate by hand: Response Topic / Correlation Data are
  MQTT 5 only and PubSubClient speaks 3.1.1.

  -- Wire format --

    * Every command the client sends is wrapped in a uniform envelope, added centrally by a
      sendCommand() helper so no call site has to remember it:
          { "value": <the old payload>, "requestId": "<uuid>" }

    * One result topic per grill: TOPIC_STATE_RESULT = "status/result", so
      grill/{id}/status/result. QoS 1.

    * PUBLISH IT WITH retain = false (the publish_message default). Every other status/ topic is
      retained because it carries state; this one carries an answer. Retaining it would replay a
      stale error toast at the next client that connects.

    * Result payload: { "requestId": "...", "command": "action/movement/rotation",
                        "ok": false, "error": "no_rotor" }

    * "command" is redundant when the reply is correlated, since the client knows what it sent, but
      it is the only thing that makes the EVERYONE case usable: a broadcast has no pending entry to
      look up. It also makes the topic readable under mosquitto_sub.

    * requestId may be the sentinel "EVERYONE" instead of a uuid, and the ESP32 chooses which.
      Failures of a specific command answer the requester; things that happen to the grill
      (emergency stop, a program cancelled by someone else, mode changed) go to everyone. A toast
      matching no action the user took is confusing, so targeted is the default.

    * "error" carries a CODE, never display text. The firmware is in English and the UI in Spanish,
      and text in the payload would mean reflashing the grill to reword a message. The firmware says
      what happened, the client owns how it reads.

    * Reply on success too, not only on failure. That is what makes silence mean exactly one thing
      (no communication) instead of leaving the client to guess by timeout. grill/connection (LWT)
      already covers the offline case.

  -- Firmware shape --

    * struct GrillRequest { String id; String value; bool replied = false; }, parsed once by the
      dispatcher in GaztaindiGrill.cpp and passed by reference into handle_mqtt_message():
          GrillRequest req = GrillMQTT::parse_request(message);
          grill->handle_mqtt_message(actionStr.c_str(), req);
          if (!req.replied) mqtt->reply_ok(req);   // success = whatever did not reject

    * Handlers therefore only ever spell out their failures: mqtt->reply_error(req, "no_rotor").
      reply_error() and defer() both set req.replied.

    * DO NOT hold the current requestId in a GrillMQTT member. That was the first design and it is
      broken: client.loop() can deliver two messages in one iteration, so a second begin_request()
      would overwrite the first id and a deferred reply would reach the wrong client. Keeping the
      flag inside the per-request struct makes that impossible.

    * Deferred outcomes must STORE the id with the pending operation instead of replying later from
      ambient state. The rule: if the outcome is not decided before the handler returns, call
      defer(req) and answer later with reply_to(storedId, ...).

    * The one deferred case today is the mode change, and it is also the best use case for the whole
      channel, because it is routine rather than a hardware fault. handle_mqtt_message() only sets
      modeManager->requestedMode and returns; the rejection ("Program in progress. Mode change
      denied.") happens later in GrillSystem::update() -> set_system_mode(). So:
          ModeManager::requestMode(Mode newMode, const String& requestId)  <- the only new signature
      stores requestedByRequestId, and set_system_mode() answers with it.

    * Everything else needs NO signature change. Guards that decide synchronously live in
      handle_mqtt_message(), where the id is already at hand, and execute_program() reads the
      requestId straight out of the JSON it already parses.

    * While doing this, move the rotor range check out of MovementManager::go_to_rotor() and up into
      the handler. Validating input at the boundary is better anyway, and it keeps MovementManager
      free of any reply concern.

    * Later, if movement completion or MOVEMENT_TIMEOUT should report, MovementManager already keeps
      targetPosition/targetDegrees/targetTemperature as members: add a parallel targetRequestId.
      Same pattern, id stored with the pending target. Not needed for the first pass.

  -- Client shape --

    * sendCommand(topic, value, opts) builds the envelope, generates the uuid and records it in a
      Map<requestId, {command, opts}> of pending requests.

    * ONE subscription to grill/+/status/result in the provider, not one per request. On a message:
      delete the id from the pending map; if it was not ours and requestId !== "EVERYONE", ignore it
      silently.

    * Error codes map to Spanish strings in the client (no_rotor, encoder_not_answering,
      mode_change_denied, invalid_json, no_steps), with a generic fallback for unknown codes so a
      newer firmware never produces a blank toast.

    * Toast suppression belongs HERE, not in the firmware: it is presentation policy, and putting it
      on the ESP32 would mean reflashing to change a UI detail. Do it per call site via an opts flag
      ({ silent: true }) stored in the pending map, rather than a global list of commands to keep in
      sync. silent should only ever mute success; a failure the user caused should always surface.
      EVERYONE messages have no pending entry, so they always show, which is right since they are
      rare and significant by definition.

  -- Migration --

    * The envelope is a breaking change: handlers compare payload == PAYLOAD_UP today and must
      become req.value == PAYLOAD_UP. Mechanical, one place per command, and parse_request() is the
      single point that unwraps it.

    * It breaks GaztaindiGrill-Shadow and any mosquitto_pub habits, so update the simulator in the
      same branch.

    * Check first whether any action/ topic is published retained: an old-format retained message
      would arrive after the migration and fail to parse.

    * Keep GaztaindiGrill-NextJS/src/constants/mqtt.ts in sync with the new topic.

  -- First consumers, in order of how much they are actually worth --

    * mode_change_denied: routine, happens in normal use, and today the mode selector just snaps
      back with no explanation at all.
    * no_rotor: see the rotation bug below, which has to be fixed regardless.
    * encoder_not_answering: see the first entry. Least valuable of the three, since it only fires
      when the hardware is genuinely broken.

- UI: let the user skip to the next step while a program is running, instead of waiting out
  the current one. Needs a new action/ topic plus a ProgramManager handler that jumps state
  straight to STEP_COMPLETED for the current step.

- BUG: rotation commands sent to grill 1 dereference an uninitialised pointer.
    * HardwareManager only allocates rotor/rotorEncoder/thermocouple inside `if (grillIndex == 0)`,
      and its constructor initialises neither, so on grill 1 they hold garbage rather than nullptr.
    * MovementManager::rotate_clockwise/rotate_counter_clockwise/stop_rotor and go_to_rotor call
      hardware->rotor->... with no guard, so `grill/1/action/movement/rotation` is undefined
      behaviour, most likely a crash and reboot. Same for a program step with a rotation.
    * Fix in the firmware, not the client: GaztaindiGrill-NextJS/TODO.md plans to block this in the
      UI, but that protects neither a second client nor mosquitto_pub.
    * Two parts: initialise the pointers to nullptr in the HardwareManager constructor, and reject
      rotor commands when there is no rotor. Good first consumer of the command-result channel,
      since the user should be told why the command was refused.
