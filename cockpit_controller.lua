-- cockpit_controller.lua
-- Zentrale Helicopter-Steuerung (Cockpit) fuer 3-Computer-Setup.
--
-- Kurzstart:
-- 1) IDs + Protokolle unten in CFG.net passend setzen.
-- 2) Auf ALLEN drei Rechnern jeweilige Skripte starten.
-- 3) Startreihenfolge: Rotor-Nodes zuerst, dann Cockpit.
-- 4) Im Cockpit: [A] Arm, [D] Disarm, [M] Mode (Hover/Cruise), [Q] Quit.
-- 5) Test: Disarmed starten -> beide Nodes ONLINE -> Arm -> kleine Korrekturwerte pruefen.

local CFG = {
  modemSide = "back",
  monitorSide = nil, -- z.B. "right" oder nil fuer Terminal
  imuSide = "left",
  altitudeSide = nil,

  controlHz = 20,
  statusRxHz = 25,
  heartbeatHz = 2,
  nodeTimeoutSec = 2.5,
  startupGraceSec = 4.0,

  -- Netzwerk/Adressierung (konfigurierbar)
  net = {
    cockpitId = os.getComputerID(),
    mainRotorId = 11,
    tailRotorId = 12,
    protocolCommand = "heli.ctrl.cmd.v1",
    protocolStatus = "heli.ctrl.status.v1",
    protocolHeartbeat = "heli.ctrl.heartbeat.v1",
  },

  modes = {
    hover = {
      name = "HOVER",
      targetRollDeg = 0.0,
      targetPitchDeg = 0.0,
      altitudeHold = true,
      yawHold = true,
      baseCollectiveDeg = 6.0,
    },
    cruise = {
      name = "CRUISE",
      targetRollDeg = 0.0,
      targetPitchDeg = -4.0,
      altitudeHold = false,
      yawHold = true,
      baseCollectiveDeg = 5.0,
    },
  },

  limits = {
    rollCmd = { min = -1.0, max = 1.0 },
    pitchCmd = { min = -1.0, max = 1.0 },
    yawCmd = { min = -1.0, max = 1.0 },
    collectiveDeg = { min = -2.0, max = 16.0 },
  },

  pid = {
    roll = { kp = 0.08, ki = 0.010, kd = 0.045, outMin = -1.0, outMax = 1.0, iMin = -0.4, iMax = 0.4 },
    pitch = { kp = 0.08, ki = 0.010, kd = 0.045, outMin = -1.0, outMax = 1.0, iMin = -0.4, iMax = 0.4 },
    yaw = { kp = 0.06, ki = 0.008, kd = 0.025, outMin = -0.8, outMax = 0.8, iMin = -0.4, iMax = 0.4 },
    altitude = { kp = 0.90, ki = 0.08, kd = 0.35, outMin = -4.0, outMax = 4.0, iMin = -3.0, iMax = 3.0 },
  },
}

local function clamp(x, mn, mx)
  if x < mn then return mn end
  if x > mx then return mx end
  return x
end

local function nowSec()
  return os.epoch("utc") / 1000
end

local function normAngleDeg(a)
  local x = (a + 180) % 360
  if x < 0 then x = x + 360 end
  return x - 180
end

local function wrapOptional(side)
  if side and peripheral.isPresent(side) then
    return peripheral.wrap(side)
  end
  return nil
end

local function openRednet(modemSide)
  if not modemSide then return false, "Kein modemSide konfiguriert." end
  if not peripheral.isPresent(modemSide) then
    return false, "Kein Modem auf Seite '" .. modemSide .. "'."
  end
  local p = peripheral.wrap(modemSide)
  if not p or peripheral.getType(modemSide) ~= "modem" then
    return false, "Peripherie auf '" .. modemSide .. "' ist kein Modem."
  end
  if p.isWireless and not p.isWireless() then
    return false, "Modem ist nicht wireless."
  end
  if not rednet.isOpen(modemSide) then
    rednet.open(modemSide)
  end
  return true
end

local function newPID(cfg)
  return {
    kp = cfg.kp, ki = cfg.ki, kd = cfg.kd,
    outMin = cfg.outMin, outMax = cfg.outMax,
    iMin = cfg.iMin, iMax = cfg.iMax,
    i = 0.0, prevErr = 0.0, first = true,
  }
end

local function pidReset(pid)
  pid.i = 0.0
  pid.prevErr = 0.0
  pid.first = true
end

local function pidStep(pid, err, dt)
  dt = math.max(dt, 1e-4)
  pid.i = clamp(pid.i + err * dt, pid.iMin, pid.iMax)

  local d = 0.0
  if pid.first then
    pid.first = false
  else
    d = (err - pid.prevErr) / dt
  end
  pid.prevErr = err

  local out = pid.kp * err + pid.ki * pid.i + pid.kd * d
  local clipped = clamp(out, pid.outMin, pid.outMax)
  if out ~= clipped then
    pid.i = clamp(pid.i - err * dt * 0.3, pid.iMin, pid.iMax)
  end
  return clipped
end

local HW = {
  imu = wrapOptional(CFG.imuSide),
  alt = wrapOptional(CFG.altitudeSide),
  monitor = wrapOptional(CFG.monitorSide),
}

local function readAttitude()
  local rollDeg, pitchDeg, yawDeg, yawRateDegS = 0, 0, 0, 0
  local ok = false
  local imu = HW.imu
  if not imu then return rollDeg, pitchDeg, yawDeg, yawRateDegS, false end

  if imu.getRoll then rollDeg = imu.getRoll() ok = true end
  if imu.getPitch then pitchDeg = imu.getPitch() ok = true end
  if imu.getYaw then yawDeg = imu.getYaw() ok = true end
  if imu.getYawRate then yawRateDegS = imu.getYawRate() ok = true end
  if imu.getAttitude then
    local a = imu.getAttitude()
    if type(a) == "table" then
      rollDeg = a.roll or a.rollDeg or rollDeg
      pitchDeg = a.pitch or a.pitchDeg or pitchDeg
      yawDeg = a.yaw or a.heading or yawDeg
      yawRateDegS = a.yawRate or a.yawRateDegS or yawRateDegS
      ok = true
    end
  end
  return rollDeg, pitchDeg, yawDeg, yawRateDegS, ok
end

local function readAltitude()
  local alt = HW.alt
  if not alt then return nil end
  if alt.getAltitude then return alt.getAltitude() end
  if alt.getY then return alt.getY() end
  return nil
end

local state = {
  running = true,
  armed = false,
  modeKey = "hover",
  seq = 0,
  heartbeatSeq = 0,
  startedAt = nowSec(),
  targetHeading = 0.0,
  targetAltitude = nil,
  telemetry = {
    roll = 0, pitch = 0, yaw = 0, yawRate = 0,
    altitude = nil,
    rollCmd = 0, pitchCmd = 0, yawCmd = 0,
    collectiveDeg = 0,
    sensorsOk = false,
    altitudeHoldActive = false,
  },
  nodes = {
    main = { id = CFG.net.mainRotorId, online = false, lastSeen = 0, lastStatus = "n/a", lastSeq = 0 },
    tail = { id = CFG.net.tailRotorId, online = false, lastSeen = 0, lastStatus = "n/a", lastSeq = 0 },
  },
}

local pids = {
  roll = newPID(CFG.pid.roll),
  pitch = newPID(CFG.pid.pitch),
  yaw = newPID(CFG.pid.yaw),
  altitude = newPID(CFG.pid.altitude),
}

local function resetPIDs()
  pidReset(pids.roll)
  pidReset(pids.pitch)
  pidReset(pids.yaw)
  pidReset(pids.altitude)
end

local function validateStatusPacket(msg)
  if type(msg) ~= "table" then return false, "status not table" end
  if msg.kind ~= "status" then return false, "kind" end
  if type(msg.node) ~= "string" then return false, "node" end
  if msg.node ~= "main" and msg.node ~= "tail" then return false, "node value" end
  if type(msg.seq) ~= "number" then return false, "seq" end
  if type(msg.ok) ~= "boolean" then return false, "ok" end
  if msg.mode ~= nil and type(msg.mode) ~= "string" then return false, "mode" end
  if msg.armed ~= nil and type(msg.armed) ~= "boolean" then return false, "armed" end
  if msg.lastCmdAgeSec ~= nil and type(msg.lastCmdAgeSec) ~= "number" then return false, "lastCmdAgeSec" end
  return true
end

local function nodeAge(nodeState)
  if nodeState.lastSeen <= 0 then return math.huge end
  return nowSec() - nodeState.lastSeen
end

local function updateNodeOnlineFlags()
  local timedOutMain = nodeAge(state.nodes.main) > CFG.nodeTimeoutSec
  local timedOutTail = nodeAge(state.nodes.tail) > CFG.nodeTimeoutSec

  state.nodes.main.online = not timedOutMain
  state.nodes.tail.online = not timedOutTail

  local graceDone = (nowSec() - state.startedAt) > CFG.startupGraceSec
  if state.armed and graceDone and (timedOutMain or timedOutTail) then
    state.armed = false
    resetPIDs()
  end
end

local function writeLine(dev, x, y, txt, color)
  if color then dev.setTextColor(color) end
  dev.setCursorPos(x, y)
  dev.write(txt)
end

local function drawUI()
  local dev = HW.monitor or term
  local w, h = dev.getSize()

  dev.setBackgroundColor(colors.black)
  dev.setTextColor(colors.white)
  dev.clear()

  writeLine(dev, 1, 1, "Cockpit Controller (3 Nodes)")
  writeLine(dev, 1, 2, string.rep("-", math.min(w, 34)))
  writeLine(dev, 1, 4, "Armed:")
  writeLine(dev, 8, 4, state.armed and "JA" or "NEIN", state.armed and colors.lime or colors.red)
  writeLine(dev, 1, 5, "Mode : " .. CFG.modes[state.modeKey].name, colors.cyan)
  writeLine(dev, 1, 6, "Sensor: " .. (state.telemetry.sensorsOk and "OK" or "FEHLT"), state.telemetry.sensorsOk and colors.lime or colors.orange)

  writeLine(dev, 1, 8, string.format("Roll/Pitch: %+6.2f / %+6.2f", state.telemetry.roll, state.telemetry.pitch))
  writeLine(dev, 1, 9, string.format("Yaw/Rate : %+6.2f / %+6.2f", state.telemetry.yaw, state.telemetry.yawRate))
  if state.telemetry.altitude ~= nil then
    writeLine(dev, 1, 10, string.format("Altitude : %+7.2f", state.telemetry.altitude))
  else
    writeLine(dev, 1, 10, "Altitude : N/A", colors.lightGray)
  end

  writeLine(dev, 1, 12, string.format("Cmd R/P/Y: %+5.2f %+5.2f %+5.2f", state.telemetry.rollCmd, state.telemetry.pitchCmd, state.telemetry.yawCmd))
  writeLine(dev, 1, 13, string.format("Collective: %+5.2f deg", state.telemetry.collectiveDeg))
  writeLine(dev, 1, 14, string.format("Alt Hold  : %s", state.telemetry.altitudeHoldActive and "AN" or "AUS"), state.telemetry.altitudeHoldActive and colors.lime or colors.yellow)

  local mAge = nodeAge(state.nodes.main)
  local tAge = nodeAge(state.nodes.tail)
  writeLine(dev, 1, 16, string.format("Main Node: %s (%.1fs)", state.nodes.main.online and "ONLINE" or "OFFLINE", mAge), state.nodes.main.online and colors.lime or colors.red)
  writeLine(dev, 1, 17, string.format("Tail Node: %s (%.1fs)", state.nodes.tail.online and "ONLINE" or "OFFLINE", tAge), state.nodes.tail.online and colors.lime or colors.red)
  writeLine(dev, 1, 18, string.format("MainStatus: %s", state.nodes.main.lastStatus))
  writeLine(dev, 1, 19, string.format("TailStatus: %s", state.nodes.tail.lastStatus))

  writeLine(dev, 1, h, "[A] Arm [D] Disarm [M] Mode [Q] Quit", colors.lightGray)
end

local function setArmed(v)
  state.armed = v
  if not v then
    state.targetAltitude = nil
    resetPIDs()
  else
    local _, _, yaw = readAttitude()
    state.targetHeading = yaw or 0
    state.targetAltitude = readAltitude()
  end
end

local function toggleMode()
  if state.modeKey == "hover" then
    state.modeKey = "cruise"
    state.targetAltitude = nil
  else
    state.modeKey = "hover"
    state.targetAltitude = readAltitude()
  end
  resetPIDs()
end

local function safeMainCommand()
  return {
    neutral = true,
    collectiveDeg = 0.0,
    rollCmd = 0.0,
    pitchCmd = 0.0,
  }
end

local function safeTailCommand()
  return {
    neutral = true,
    yawCmd = 0.0,
    throttle = 0.0,
  }
end

local function buildCommands(dt)
  local roll, pitch, yaw, yawRate, sensorsOk = readAttitude()
  local altitude = readAltitude()
  local mode = CFG.modes[state.modeKey]

  state.telemetry.roll = roll
  state.telemetry.pitch = pitch
  state.telemetry.yaw = yaw
  state.telemetry.yawRate = yawRate
  state.telemetry.altitude = altitude
  state.telemetry.sensorsOk = sensorsOk

  local rollCmd, pitchCmd, yawCmd = 0.0, 0.0, 0.0
  local collectiveDeg = 0.0
  local altHoldActive = false

  if state.armed and sensorsOk then
    local rollErr = mode.targetRollDeg - roll
    local pitchErr = mode.targetPitchDeg - pitch
    rollCmd = pidStep(pids.roll, rollErr, dt)
    pitchCmd = pidStep(pids.pitch, pitchErr, dt)

    collectiveDeg = mode.baseCollectiveDeg
    if mode.altitudeHold and altitude ~= nil then
      if state.targetAltitude == nil then
        state.targetAltitude = altitude
      end
      local altErr = state.targetAltitude - altitude
      collectiveDeg = collectiveDeg + pidStep(pids.altitude, altErr, dt)
      altHoldActive = true
    end

    if mode.yawHold then
      local yawErr = normAngleDeg(state.targetHeading - yaw)
      yawCmd = pidStep(pids.yaw, yawErr, dt)
    else
      yawCmd = pidStep(pids.yaw, -yawRate, dt)
    end
  else
    resetPIDs()
  end

  rollCmd = clamp(rollCmd, CFG.limits.rollCmd.min, CFG.limits.rollCmd.max)
  pitchCmd = clamp(pitchCmd, CFG.limits.pitchCmd.min, CFG.limits.pitchCmd.max)
  yawCmd = clamp(yawCmd, CFG.limits.yawCmd.min, CFG.limits.yawCmd.max)
  collectiveDeg = clamp(collectiveDeg, CFG.limits.collectiveDeg.min, CFG.limits.collectiveDeg.max)

  state.telemetry.rollCmd = rollCmd
  state.telemetry.pitchCmd = pitchCmd
  state.telemetry.yawCmd = yawCmd
  state.telemetry.collectiveDeg = collectiveDeg
  state.telemetry.altitudeHoldActive = altHoldActive

  if not state.armed then
    return safeMainCommand(), safeTailCommand()
  end

  return {
    neutral = false,
    collectiveDeg = collectiveDeg,
    rollCmd = rollCmd,
    pitchCmd = pitchCmd,
  }, {
    neutral = false,
    yawCmd = yawCmd,
    throttle = math.abs(yawCmd),
  }
end

local function sendCommandPacket(mainCmd, tailCmd)
  state.seq = state.seq + 1
  local payload = {
    kind = "command",
    seq = state.seq,
    timestampMs = os.epoch("utc"),
    sender = "cockpit",
    cockpitId = CFG.net.cockpitId,
    armed = state.armed,
    mode = state.modeKey,
    main = mainCmd,
    tail = tailCmd,
  }

  rednet.send(CFG.net.mainRotorId, payload, CFG.net.protocolCommand)
  rednet.send(CFG.net.tailRotorId, payload, CFG.net.protocolCommand)
end

local function sendHeartbeat()
  state.heartbeatSeq = state.heartbeatSeq + 1
  local hb = {
    kind = "heartbeat",
    seq = state.heartbeatSeq,
    cockpitId = CFG.net.cockpitId,
    timestampMs = os.epoch("utc"),
  }
  rednet.send(CFG.net.mainRotorId, hb, CFG.net.protocolHeartbeat)
  rednet.send(CFG.net.tailRotorId, hb, CFG.net.protocolHeartbeat)
end

local function rxLoop()
  while state.running do
    local sender, msg = rednet.receive(CFG.net.protocolStatus, 1.0 / CFG.statusRxHz)
    if sender then
      local ok, reason = validateStatusPacket(msg)
      if ok then
        local n = state.nodes[msg.node]
        if n and sender == n.id then
          n.lastSeen = nowSec()
          n.lastSeq = msg.seq
          n.lastStatus = msg.statusText or (msg.ok and "ok" or "error")
        end
      else
        if sender == CFG.net.mainRotorId then
          state.nodes.main.lastStatus = "bad status: " .. reason
        elseif sender == CFG.net.tailRotorId then
          state.nodes.tail.lastStatus = "bad status: " .. reason
        end
      end
    end
    updateNodeOnlineFlags()
  end
end

local function controlLoop()
  local dt = 1 / CFG.controlHz
  while state.running do
    local t0 = os.clock()

    updateNodeOnlineFlags()
    local mainCmd, tailCmd = buildCommands(dt)

    if not (state.nodes.main.online and state.nodes.tail.online) then
      state.armed = false
      mainCmd = safeMainCommand()
      tailCmd = safeTailCommand()
    end

    sendCommandPacket(mainCmd, tailCmd)
    drawUI()

    local elapsed = os.clock() - t0
    local sleepTime = dt - elapsed
    if sleepTime > 0 then sleep(sleepTime) else sleep(0) end
  end

  sendCommandPacket(safeMainCommand(), safeTailCommand())
end

local function heartbeatLoop()
  local dt = 1 / CFG.heartbeatHz
  while state.running do
    sendHeartbeat()
    sleep(dt)
  end
end

local function uiLoop()
  while state.running do
    local ev, key = os.pullEvent("key")
    if key == keys.a then
      setArmed(true)
    elseif key == keys.d then
      setArmed(false)
    elseif key == keys.m then
      toggleMode()
    elseif key == keys.q then
      state.running = false
      setArmed(false)
    end
    drawUI()
  end
end

local ok, err = openRednet(CFG.modemSide)
if not ok then
  term.setTextColor(colors.red)
  print("Netzwerkfehler: " .. err)
  return
end

drawUI()
parallel.waitForAny(controlLoop, rxLoop, heartbeatLoop, uiLoop)
setArmed(false)
sendCommandPacket(safeMainCommand(), safeTailCommand())
term.setBackgroundColor(colors.black)
term.setTextColor(colors.white)
term.clear()
term.setCursorPos(1, 1)
print("Cockpit Controller beendet.")
