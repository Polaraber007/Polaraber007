-- tail_rotor_node.lua
-- Tail-Rotor-Node: steuert Drehrichtung + Drehzahl des Heckrotors.
--
-- Kurzstart:
-- 1) CFG.net IDs/Protokolle mit Cockpit abgleichen.
-- 2) Hardware-Wrapper in applyTailOutput() auf reale Methoden mappen.
-- 3) Node starten, dann Cockpit.
-- 4) Bei Timeout: neutral/stop wird automatisch gesetzt.

local CFG = {
  modemSide = "back",
  tailSide = "left",
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

  limits = {
    yawCmdMin = -1.0,
    yawCmdMax = 1.0,
    throttleMin = 0.0,
    throttleMax = 1.0,
    neutralYawCmd = 0.0,
    neutralThrottle = 0.0,
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

local function validateCommandPacket(msg)
  if type(msg) ~= "table" then return false, "msg not table" end
  if msg.kind ~= "command" then return false, "kind" end
  if type(msg.seq) ~= "number" then return false, "seq" end
  if type(msg.armed) ~= "boolean" then return false, "armed" end
  if type(msg.tail) ~= "table" then return false, "tail" end

  local tail = msg.tail
  if type(tail.neutral) ~= "boolean" then return false, "tail.neutral" end
  if type(tail.yawCmd) ~= "number" then return false, "tail.yawCmd" end
  if type(tail.throttle) ~= "number" then return false, "tail.throttle" end
  if tail.yawCmd < -2 or tail.yawCmd > 2 then return false, "yawCmd range" end
  if tail.throttle < -1 or tail.throttle > 2 then return false, "throttle range" end
  return true
end

local tail = nil
if CFG.tailSide and peripheral.isPresent(CFG.tailSide) then
  tail = peripheral.wrap(CFG.tailSide)
end

-- Hardware-Wrapper: hier bei Bedarf auf echte Peripheral-Methoden anpassen.
local function applyTailOutput(yawCmd, throttle)
  if not tail then return end

  local direction = 0
  if yawCmd > 0.001 then direction = 1 elseif yawCmd < -0.001 then direction = -1 end
  local speed = throttle

  if tail.setDirection then
    tail.setDirection(direction)
  elseif tail.setReversed then
    tail.setReversed(direction < 0)
  end

  if tail.setSpeed then
    tail.setSpeed(speed)
  elseif tail.setThrottle then
    tail.setThrottle(speed)
  elseif tail.setTargetSpeed then
    tail.setTargetSpeed(speed)
  end

  -- Falls nur eine kombinierte Methode existiert:
  if tail.setCommand then
    tail.setCommand({ direction = direction, speed = speed, yawCmd = yawCmd })
  end
end

local state = {
  running = true,
  seq = 0,
  rxSeq = 0,
  lastCmdAt = 0,
  armed = false,
  mode = "n/a",
  statusText = "boot",
  command = {
    neutral = true,
    yawCmd = 0.0,
    throttle = 0.0,
  },
}

local function sendStatus(ok)
  state.seq = state.seq + 1
  local payload = {
    kind = "status",
    node = "tail",
    seq = state.seq,
    ok = ok,
    armed = state.armed,
    mode = state.mode,
    statusText = state.statusText,
    rxSeq = state.rxSeq,
    lastCmdAgeSec = nowSec() - state.lastCmdAt,
    timestampMs = os.epoch("utc"),
  }
  rednet.send(CFG.net.cockpitId, payload, CFG.net.protocolStatus)
end

local function applyNeutral()
  applyTailOutput(CFG.limits.neutralYawCmd, CFG.limits.neutralThrottle)
end

local function rxLoop()
  while state.running do
    local sender, msg, protocol = rednet.receive(nil, 0.1)
    if sender and sender == CFG.net.cockpitId then
      if protocol == CFG.net.protocolCommand then
        local ok, reason = validateCommandPacket(msg)
        if ok then
          state.command = msg.tail
          state.armed = msg.armed
          state.mode = msg.mode or "unknown"
          state.rxSeq = msg.seq
          state.lastCmdAt = nowSec()
          state.statusText = "cmd ok"
        else
          state.statusText = "cmd invalid: " .. reason
        end
      elseif protocol == CFG.net.protocolHeartbeat then
        if type(msg) == "table" and msg.kind == "heartbeat" then
          -- Heartbeat wird implizit ueber statusLoop bestaetigt.
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
      applyNeutral()
      if timedOut then
        state.statusText = "timeout -> stop"
      else
        state.statusText = "neutral"
      end
    else
      local yawCmd = clamp(state.command.yawCmd, CFG.limits.yawCmdMin, CFG.limits.yawCmdMax)
      local throttle = clamp(state.command.throttle, CFG.limits.throttleMin, CFG.limits.throttleMax)
      applyTailOutput(yawCmd, throttle)
      state.statusText = "tracking"
    end

    local elapsed = os.clock() - t0
    local st = dt - elapsed
    if st > 0 then sleep(st) else sleep(0) end
  end
  applyNeutral()
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
print("Tail Rotor Node online. ID=" .. os.getComputerID())
print("Warte auf Cockpit-Kommandos...")

parallel.waitForAny(rxLoop, controlLoop, statusLoop)
applyNeutral()
