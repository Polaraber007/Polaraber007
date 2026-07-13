-- main_rotor_node.lua
-- Main-Rotor-Node: steuert einzelne Blattwinkel via setTargetAngle().
--
-- Kurzstart:
-- 1) CFG.net IDs/Protokolle mit Cockpit abgleichen.
-- 2) CFG.blades konfigurieren (Seiten der Blatt-Peripherien).
-- 3) Node starten, dann Cockpit starten.
-- 4) Bei Kommunikationsverlust gehen Blaetter automatisch auf Neutral.

local CFG = {
  modemSide = "back",
  controlHz = 30,
  statusHz = 4,
  commandTimeoutSec = 1.2,

  net = {
    cockpitId = 10,
    thisNodeId = os.getComputerID(),
    protocolCommand = "heli.ctrl.cmd.v1",
    protocolStatus = "heli.ctrl.status.v1",
    protocolHeartbeat = "heli.ctrl.heartbeat.v1",
  },

  rotor = {
    maxBladePitchDeg = 20.0,
    collectiveMinDeg = -2.0,
    collectiveMaxDeg = 16.0,
    cyclicGainDegPerCmd = 8.0,
    neutralAngleDeg = 0.0,
    maxSlewDegPerSec = 120.0, -- optional rate limit
    smoothingAlpha = 0.35, -- 0..1, hoeher = direkter
  },

  -- Blatt-Peripheriezuordnung: jede Seite sollte ein Peripheral mit setTargetAngle(deg) sein.
  -- Alternativ kann ein zentrales Rotor-Peripheral genutzt werden (siehe applyBladeAngles()).
  blades = {
    "top",
    "left",
    "right",
    "front",
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

local function openRednet(modemSide)
  if not modemSide then return false, "Kein modemSide konfiguriert." end
  if not peripheral.isPresent(modemSide) then
    return false, "Kein Modem auf '" .. modemSide .. "'."
  end
  local p = peripheral.wrap(modemSide)
  if not p or peripheral.getType(modemSide) ~= "modem" then
    return false, "Peripherie auf '" .. modemSide .. "' ist kein Modem."
  end
  if p.isWireless and not p.isWireless() then
    return false, "Modem ist nicht wireless."
  end
  if not rednet.isOpen(modemSide) then rednet.open(modemSide) end
  return true
end

local function wrapOptional(side)
  if side and peripheral.isPresent(side) then
    return peripheral.wrap(side)
  end
  return nil
end

local function validateCommandPacket(msg)
  if type(msg) ~= "table" then return false, "msg not table" end
  if msg.kind ~= "command" then return false, "kind" end
  if type(msg.seq) ~= "number" then return false, "seq" end
  if type(msg.armed) ~= "boolean" then return false, "armed" end
  if type(msg.main) ~= "table" then return false, "main" end

  local main = msg.main
  if type(main.neutral) ~= "boolean" then return false, "main.neutral" end
  if type(main.collectiveDeg) ~= "number" then return false, "main.collectiveDeg" end
  if type(main.rollCmd) ~= "number" then return false, "main.rollCmd" end
  if type(main.pitchCmd) ~= "number" then return false, "main.pitchCmd" end

  if main.collectiveDeg < -50 or main.collectiveDeg > 50 then return false, "main.collectiveDeg range" end
  if main.rollCmd < -2 or main.rollCmd > 2 then return false, "main.rollCmd range" end
  if main.pitchCmd < -2 or main.pitchCmd > 2 then return false, "main.pitchCmd range" end

  return true
end

local function allocateBladeAngles(collectiveDeg, rollCmd, pitchCmd, bladeCount)
  local out = {}
  local collective = clamp(collectiveDeg, CFG.rotor.collectiveMinDeg, CFG.rotor.collectiveMaxDeg)
  local cyclicGain = CFG.rotor.cyclicGainDegPerCmd

  for i = 1, bladeCount do
    local az = 2 * math.pi * (i - 1) / bladeCount
    local cyclic = cyclicGain * (pitchCmd * math.cos(az) + rollCmd * math.sin(az))
    out[i] = clamp(collective + cyclic, -CFG.rotor.maxBladePitchDeg, CFG.rotor.maxBladePitchDeg)
  end
  return out
end

local bladePeripherals = {}
for i = 1, #CFG.blades do
  bladePeripherals[i] = wrapOptional(CFG.blades[i])
end

local rotorPeripheral = nil
if peripheral.isPresent("bottom") then
  rotorPeripheral = peripheral.wrap("bottom")
end

local state = {
  running = true,
  seq = 0,
  rxSeq = 0,
  lastCmdAt = 0,
  lastHbAt = 0,
  armed = false,
  mode = "n/a",
  lastStatusText = "boot",
  command = {
    neutral = true,
    collectiveDeg = 0.0,
    rollCmd = 0.0,
    pitchCmd = 0.0,
  },
  currentAngles = {},
}

for i = 1, #CFG.blades do
  state.currentAngles[i] = CFG.rotor.neutralAngleDeg
end

local function applyBladeAngles(targetAngles, dt)
  for i = 1, #targetAngles do
    local prev = state.currentAngles[i] or CFG.rotor.neutralAngleDeg
    local target = targetAngles[i]

    local filtered = prev + (target - prev) * CFG.rotor.smoothingAlpha
    local maxStep = CFG.rotor.maxSlewDegPerSec * dt
    local stepped = clamp(filtered, prev - maxStep, prev + maxStep)
    local final = clamp(stepped, -CFG.rotor.maxBladePitchDeg, CFG.rotor.maxBladePitchDeg)

    state.currentAngles[i] = final
  end

  -- Prioritaet: direkte Blatt-Peripherien mit setTargetAngle()
  for i = 1, #state.currentAngles do
    local p = bladePeripherals[i]
    if p and p.setTargetAngle then
      p.setTargetAngle(state.currentAngles[i])
    end
  end

  -- Optionaler Fallback fuer zentrales Rotor-Peripheral
  if rotorPeripheral then
    if rotorPeripheral.setBladeAngles then
      rotorPeripheral.setBladeAngles(state.currentAngles)
    elseif rotorPeripheral.setTargetAngle then
      for i = 1, #state.currentAngles do
        rotorPeripheral.setTargetAngle(i, state.currentAngles[i])
      end
    end
  end
end

local function moveNeutral(dt)
  local neutral = {}
  for i = 1, #CFG.blades do
    neutral[i] = CFG.rotor.neutralAngleDeg
  end
  applyBladeAngles(neutral, dt)
end

local function sendStatus(ok)
  state.seq = state.seq + 1
  local age = nowSec() - state.lastCmdAt
  local payload = {
    kind = "status",
    node = "main",
    seq = state.seq,
    ok = ok,
    armed = state.armed,
    mode = state.mode,
    statusText = state.lastStatusText,
    rxSeq = state.rxSeq,
    bladeCount = #CFG.blades,
    lastCmdAgeSec = age,
    timestampMs = os.epoch("utc"),
  }
  rednet.send(CFG.net.cockpitId, payload, CFG.net.protocolStatus)
end

local function rxLoop()
  while state.running do
    local sender, msg, protocol = rednet.receive(nil, 0.1)
    if sender and sender == CFG.net.cockpitId then
      if protocol == CFG.net.protocolCommand then
        local ok, reason = validateCommandPacket(msg)
        if ok then
          state.command = msg.main
          state.armed = msg.armed
          state.mode = msg.mode or "unknown"
          state.rxSeq = msg.seq
          state.lastCmdAt = nowSec()
          state.lastStatusText = "cmd ok"
        else
          state.lastStatusText = "cmd invalid: " .. reason
        end
      elseif protocol == CFG.net.protocolHeartbeat then
        if type(msg) == "table" and msg.kind == "heartbeat" and type(msg.seq) == "number" then
          state.lastHbAt = nowSec()
        end
      end
    end
  end
end

local function controlLoop()
  local dt = 1 / CFG.controlHz
  while state.running do
    local t0 = os.clock()
    local timedOut = (nowSec() - state.lastCmdAt) > CFG.commandTimeoutSec
    if timedOut or not state.armed or state.command.neutral then
      moveNeutral(dt)
      if timedOut then
        state.lastStatusText = "timeout -> neutral"
      else
        state.lastStatusText = "neutral"
      end
    else
      local rollCmd = clamp(state.command.rollCmd, -1.0, 1.0)
      local pitchCmd = clamp(state.command.pitchCmd, -1.0, 1.0)
      local collectiveDeg = clamp(state.command.collectiveDeg, CFG.rotor.collectiveMinDeg, CFG.rotor.collectiveMaxDeg)
      local target = allocateBladeAngles(collectiveDeg, rollCmd, pitchCmd, #CFG.blades)
      applyBladeAngles(target, dt)
      state.lastStatusText = "tracking"
    end

    local elapsed = os.clock() - t0
    local st = dt - elapsed
    if st > 0 then sleep(st) else sleep(0) end
  end
  moveNeutral(dt)
end

local function statusLoop()
  local dt = 1 / CFG.statusHz
  while state.running do
    local timedOut = (nowSec() - state.lastCmdAt) > CFG.commandTimeoutSec
    sendStatus(not timedOut)
    sleep(dt)
  end
end

local ok, err = openRednet(CFG.modemSide)
if not ok then
  term.setTextColor(colors.red)
  print("Netzwerkfehler: " .. err)
  return
end

term.setTextColor(colors.white)
term.setBackgroundColor(colors.black)
term.clear()
term.setCursorPos(1, 1)
print("Main Rotor Node online. ID=" .. os.getComputerID())
print("Warte auf Cockpit-Kommandos...")

parallel.waitForAny(rxLoop, controlLoop, statusLoop)
moveNeutral(1 / CFG.controlHz)
