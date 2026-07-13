-- helicopter_autopilot.lua
-- ComputerCraft helicopter flight-control script
--
-- Features:
--  * Arm/disarm stabilization
--  * Hover / Cruise modes
--  * Roll, pitch, yaw stabilization (PID)
--  * Optional altitude hold in Hover mode (graceful fallback)
--  * Main rotor collective + per-blade cyclic pitch generation
--  * Tail rotor anti-torque + yaw control
--
-- IMPORTANT:
-- Map hardware methods in HW adapter functions below to your actual peripherals.

-- =========================
-- Configuration
-- =========================
local CFG = {
  updateHz = 25,                     -- Control loop frequency
  monitorSide = "right",             -- Optional monitor side
  imuSide = "back",                  -- IMU/attitude sensor peripheral side
  mainRotorSide = "top",             -- Main rotor actuator peripheral side
  tailRotorSide = "left",            -- Tail rotor actuator peripheral side
  altitudeSide = "bottom",           -- Optional altitude sensor side

  -- Main rotor geometry
  rotor = {
    bladeCount = 4,                  -- Number of individually controllable main blades
    maxBladePitchDeg = 20,           -- Blade pitch clamp for individual blades
    collectiveMinDeg = -2,           -- Collective clamp
    collectiveMaxDeg = 15,
    cyclicGainDegPerCmd = 8,         -- Cyclic authority scale (deg per normalized command)
  },

  -- Tail rotor
  tail = {
    minCmd = -1.0,
    maxCmd = 1.0,
    antiTorqueFeedForward = 0.06,    -- Base anti-torque per degree collective
  },

  -- Hover/Cruise defaults
  modes = {
    hover = {
      name = "HOVER",
      targetRollDeg = 0,
      targetPitchDeg = 0,
      yawHold = true,
      altitudeHold = true,
      -- Base collective used if altitude hold is unavailable
      baseCollectiveDeg = 5.0,
    },
    cruise = {
      name = "CRUISE",
      targetRollDeg = 0,
      targetPitchDeg = -4,           -- Slight forward attitude for normal forward flight
      yawHold = true,
      altitudeHold = false,
      baseCollectiveDeg = 4.0,
    },
  },

  -- PID gains (start conservative and tune incrementally)
  -- Tuning hints:
  -- * Increase kp for stronger correction
  -- * Increase kd to damp oscillation
  -- * Increase ki only if steady-state offset persists
  -- * Keep output clamps tight to avoid aggressive commands
  pid = {
    roll =  { kp = 0.08, ki = 0.010, kd = 0.045, outMin = -1.0, outMax = 1.0, iMin = -0.4, iMax = 0.4 },
    pitch = { kp = 0.08, ki = 0.010, kd = 0.045, outMin = -1.0, outMax = 1.0, iMin = -0.4, iMax = 0.4 },

    -- Yaw controller output is normalized tail command contribution
    yaw =   { kp = 0.06, ki = 0.008, kd = 0.025, outMin = -0.7, outMax = 0.7, iMin = -0.4, iMax = 0.4 },

    -- Altitude output is collective adjustment in degrees
    altitude = { kp = 0.90, ki = 0.08, kd = 0.35, outMin = -4.0, outMax = 4.0, iMin = -3.0, iMax = 3.0 },
  },

  -- Safety
  safe = {
    neutralCollectiveDeg = 0,
    neutralTailCmd = 0,
    neutralBladePitchDeg = 0,
  },

  -- UI
  ui = {
    textScale = 0.5,
  }
}

-- =========================
-- Utility
-- =========================
local function clamp(x, mn, mx)
  if x < mn then return mn end
  if x > mx then return mx end
  return x
end

local function wrapOptional(side)
  if side and peripheral.isPresent(side) then
    return peripheral.wrap(side)
  end
  return nil
end

local function normAngleDeg(a)
  local x = (a + 180) % 360
  if x < 0 then x = x + 360 end
  return x - 180
end

-- =========================
-- PID Controller
-- =========================
local function newPID(cfg)
  return {
    kp = cfg.kp, ki = cfg.ki, kd = cfg.kd,
    outMin = cfg.outMin, outMax = cfg.outMax,
    iMin = cfg.iMin, iMax = cfg.iMax,
    i = 0,
    prevErr = 0,
    first = true,
  }
end

local function pidReset(pid)
  pid.i = 0
  pid.prevErr = 0
  pid.first = true
end

local function pidStep(pid, err, dt)
  dt = math.max(dt, 1e-4)

  -- Integrator with clamping (anti-windup)
  pid.i = clamp(pid.i + err * dt, pid.iMin, pid.iMax)

  local d = 0
  if not pid.first then
    d = (err - pid.prevErr) / dt
  else
    pid.first = false
  end
  pid.prevErr = err

  local out = pid.kp * err + pid.ki * pid.i + pid.kd * d
  local clamped = clamp(out, pid.outMin, pid.outMax)

  -- Simple anti-windup backoff when saturating hard
  if out ~= clamped then
    pid.i = clamp(pid.i - err * dt * 0.3, pid.iMin, pid.iMax)
  end

  return clamped
end

-- =========================
-- Hardware adapters (map these to your peripherals)
-- =========================
local HW = {
  imu = wrapOptional(CFG.imuSide),
  main = wrapOptional(CFG.mainRotorSide),
  tail = wrapOptional(CFG.tailRotorSide),
  alt = wrapOptional(CFG.altitudeSide),
}

-- Read attitude and yaw rate if available.
-- Expected ranges:
--  rollDeg/pitchDeg in degrees, yawDeg heading in degrees, yawRateDegS in deg/s
local function readAttitude()
  local rollDeg, pitchDeg, yawDeg, yawRateDegS = 0, 0, 0, 0

  if not HW.imu then
    return rollDeg, pitchDeg, yawDeg, yawRateDegS, false
  end

  -- Placeholder mapping examples (replace with your real methods):
  if HW.imu.getRoll then rollDeg = HW.imu.getRoll() end
  if HW.imu.getPitch then pitchDeg = HW.imu.getPitch() end
  if HW.imu.getYaw then yawDeg = HW.imu.getYaw() end
  if HW.imu.getYawRate then yawRateDegS = HW.imu.getYawRate() end

  if HW.imu.getAttitude then
    local a = HW.imu.getAttitude()
    if type(a) == "table" then
      rollDeg = a.roll or a.rollDeg or rollDeg
      pitchDeg = a.pitch or a.pitchDeg or pitchDeg
      yawDeg = a.yaw or a.heading or yawDeg
      yawRateDegS = a.yawRate or a.yawRateDegS or yawRateDegS
    end
  end

  return rollDeg, pitchDeg, yawDeg, yawRateDegS, true
end

-- Read altitude in blocks/meters (optional)
local function readAltitude()
  if not HW.alt then return nil end

  -- Placeholder mapping examples (replace with your real methods):
  if HW.alt.getAltitude then
    return HW.alt.getAltitude()
  end
  if HW.alt.getY then
    return HW.alt.getY()
  end

  return nil
end

-- Apply per-blade pitch for main rotor.
-- bladePitchesDeg: array index 1..bladeCount
local function setMainRotorBladePitches(bladePitchesDeg)
  if not HW.main then return end

  -- Preferred: single bulk setter
  if HW.main.setBladePitches then
    HW.main.setBladePitches(bladePitchesDeg)
    return
  end

  -- Fallback: per-blade setter
  if HW.main.setBladePitch then
    for i = 1, #bladePitchesDeg do
      HW.main.setBladePitch(i, bladePitchesDeg[i])
    end
    return
  end

  -- Optional fallback: named methods setBlade1Pitch, setBlade2Pitch, ...
  for i = 1, #bladePitchesDeg do
    local m = HW.main["setBlade" .. i .. "Pitch"]
    if m then m(bladePitchesDeg[i]) end
  end
end

-- Tail rotor command in normalized range [-1..1]
local function setTailRotor(cmd)
  if not HW.tail then return end

  if HW.tail.setCommand then
    HW.tail.setCommand(cmd)
  elseif HW.tail.setPitch then
    HW.tail.setPitch(cmd)
  elseif HW.tail.setThrottle then
    HW.tail.setThrottle(cmd)
  end
end

-- Safe output reset on disarm/exit
local function applySafeNeutral()
  local neutral = {}
  for i = 1, CFG.rotor.bladeCount do
    neutral[i] = CFG.safe.neutralBladePitchDeg
  end
  setMainRotorBladePitches(neutral)
  setTailRotor(CFG.safe.neutralTailCmd)
end

-- =========================
-- Control allocator
-- =========================
-- Converts collective + cyclic (roll/pitch commands) into per-blade pitch.
-- Rotor does NOT tilt; only blade pitch varies around azimuth.
--
-- Azimuth model:
-- blade i azimuth = 2*pi*(i-1)/N
-- cyclic contribution = pitchCmd*cos(az) + rollCmd*sin(az)
local function allocateMainRotorBladePitch(collectiveDeg, rollCmd, pitchCmd)
  local pitches = {}
  local N = CFG.rotor.bladeCount
  local cyclicScale = CFG.rotor.cyclicGainDegPerCmd

  local collective = clamp(collectiveDeg, CFG.rotor.collectiveMinDeg, CFG.rotor.collectiveMaxDeg)

  for i = 1, N do
    local az = 2 * math.pi * (i - 1) / N
    local cyclicDeg = cyclicScale * (pitchCmd * math.cos(az) + rollCmd * math.sin(az))
    pitches[i] = clamp(
      collective + cyclicDeg,
      -CFG.rotor.maxBladePitchDeg,
      CFG.rotor.maxBladePitchDeg
    )
  end

  return pitches
end

-- =========================
-- Runtime state
-- =========================
local state = {
  running = true,
  armed = false,
  modeKey = "hover", -- hover or cruise
  targetHeading = 0,
  targetAltitude = nil,

  telemetry = {
    roll = 0, pitch = 0, yaw = 0, yawRate = 0,
    altitude = nil,
    rollCmd = 0, pitchCmd = 0,
    collectiveCmd = 0,
    tailCmd = 0,
    altitudeHoldActive = false,
    sensorsOk = false,
  }
}

local monitor = wrapOptional(CFG.monitorSide)
if monitor and monitor.setTextScale then
  monitor.setTextScale(CFG.ui.textScale)
end

local pids = {
  roll = newPID(CFG.pid.roll),
  pitch = newPID(CFG.pid.pitch),
  yaw = newPID(CFG.pid.yaw),
  altitude = newPID(CFG.pid.altitude),
}

local function resetControllers()
  pidReset(pids.roll)
  pidReset(pids.pitch)
  pidReset(pids.yaw)
  pidReset(pids.altitude)
end

local function setArmed(v)
  state.armed = v
  if not v then
    resetControllers()
    state.targetAltitude = nil
    applySafeNeutral()
  else
    local _, _, yaw = readAttitude()
    state.targetHeading = yaw or 0
    state.targetAltitude = readAltitude()
  end
end

local function switchMode(modeKey)
  if not CFG.modes[modeKey] then return end
  state.modeKey = modeKey
  resetControllers()
  local _, _, yaw = readAttitude()
  state.targetHeading = yaw or 0
  if modeKey == "hover" then
    state.targetAltitude = readAltitude()
  else
    state.targetAltitude = nil
  end
end

-- =========================
-- UI
-- =========================
local function writeLine(dev, x, y, txt, color)
  if color then dev.setTextColor(color) end
  dev.setCursorPos(x, y)
  dev.write(txt)
end

local function drawUI()
  local dev = monitor or term
  local w, h = dev.getSize()

  dev.setBackgroundColor(colors.black)
  dev.setTextColor(colors.white)
  dev.clear()

  writeLine(dev, 1, 1, "Helicopter Flight Control")
  writeLine(dev, 1, 2, string.rep("-", math.max(1, math.min(30, w))))

  writeLine(dev, 1, 4, "Armed:")
  writeLine(dev, 8, 4, state.armed and "YES" or "NO", state.armed and colors.lime or colors.red)

  writeLine(dev, 1, 5, "Mode : " .. CFG.modes[state.modeKey].name, colors.cyan)
  writeLine(dev, 1, 6, string.format("Sensors: %s", state.telemetry.sensorsOk and "OK" or "MISSING"), state.telemetry.sensorsOk and colors.lime or colors.orange)

  writeLine(dev, 1, 8, string.format("Roll/Pitch: %+6.2f / %+6.2f deg", state.telemetry.roll, state.telemetry.pitch))
  writeLine(dev, 1, 9, string.format("Yaw/Rate : %+6.2f / %+6.2f", state.telemetry.yaw, state.telemetry.yawRate))

  if state.telemetry.altitude ~= nil then
    writeLine(dev, 1, 10, string.format("Altitude : %+7.2f", state.telemetry.altitude))
  else
    writeLine(dev, 1, 10, "Altitude : N/A", colors.lightGray)
  end

  writeLine(dev, 1, 12, string.format("Cmd Roll/Pitch: %+5.2f / %+5.2f", state.telemetry.rollCmd, state.telemetry.pitchCmd))
  writeLine(dev, 1, 13, string.format("Collective   : %+5.2f deg", state.telemetry.collectiveCmd))
  writeLine(dev, 1, 14, string.format("Tail Cmd     : %+5.2f", state.telemetry.tailCmd))

  writeLine(dev, 1, 16, string.format("Alt-Hold     : %s", state.telemetry.altitudeHoldActive and "ACTIVE" or "OFF"), state.telemetry.altitudeHoldActive and colors.lime or colors.yellow)

  writeLine(dev, 1, h - 1, "A=Arm D=Disarm M=Mode Q=Quit", colors.lightGray)

  -- touch button panel for monitor
  if monitor then
    local bx1, by1 = math.max(1, w - 22), math.max(1, h - 4)
    local bx2, by2 = w, h
    local bg = state.armed and colors.red or colors.green
    monitor.setBackgroundColor(bg)
    for y = by1, by2 do
      monitor.setCursorPos(bx1, y)
      monitor.write(string.rep(" ", bx2 - bx1 + 1))
    end
    monitor.setCursorPos(bx1 + 1, by1 + 1)
    monitor.setTextColor(colors.white)
    monitor.write(state.armed and "DISARM" or "ARM")
    monitor.setCursorPos(bx1 + 1, by1 + 2)
    monitor.write("MODE")
    monitor.setBackgroundColor(colors.black)
  end
end

local function uiLoop()
  while state.running do
    local ev, p1, p2, p3 = os.pullEvent()

    if ev == "key" then
      if p1 == keys.a then setArmed(true)
      elseif p1 == keys.d then setArmed(false)
      elseif p1 == keys.m then
        switchMode(state.modeKey == "hover" and "cruise" or "hover")
      elseif p1 == keys.q then
        state.running = false
      end
      drawUI()

    elseif ev == "monitor_touch" and monitor then
      local x, y = p2, p3
      local w, h = monitor.getSize()
      local bx1, by1 = math.max(1, w - 22), math.max(1, h - 4)
      local bx2, by2 = w, h

      if x >= bx1 and x <= bx2 and y >= by1 and y <= by2 then
        if y <= by1 + 1 then
          setArmed(not state.armed)
        else
          switchMode(state.modeKey == "hover" and "cruise" or "hover")
        end
        drawUI()
      end
    end
  end
end

-- =========================
-- Control loop
-- =========================
local function controlLoop()
  local dtTarget = 1 / CFG.updateHz

  while state.running do
    local t0 = os.clock()

    local roll, pitch, yaw, yawRate, sensorsOk = readAttitude()
    local altitude = readAltitude()
    local mode = CFG.modes[state.modeKey]

    state.telemetry.roll = roll
    state.telemetry.pitch = pitch
    state.telemetry.yaw = yaw
    state.telemetry.yawRate = yawRate
    state.telemetry.altitude = altitude
    state.telemetry.sensorsOk = sensorsOk

    local rollCmd, pitchCmd = 0, 0
    local collective = mode.baseCollectiveDeg
    local tailCmd = CFG.tail.antiTorqueFeedForward * collective
    local altHoldActive = false

    if state.armed and sensorsOk then
      -- Roll/Pitch stabilization
      local rollErr = mode.targetRollDeg - roll
      local pitchErr = mode.targetPitchDeg - pitch
      rollCmd = pidStep(pids.roll, rollErr, dtTarget)
      pitchCmd = pidStep(pids.pitch, pitchErr, dtTarget)

      -- Optional altitude hold in hover mode
      if mode.altitudeHold and altitude ~= nil then
        if state.targetAltitude == nil then
          state.targetAltitude = altitude
        end
        local altErr = state.targetAltitude - altitude
        local collectiveAdj = pidStep(pids.altitude, altErr, dtTarget)
        collective = collective + collectiveAdj
        altHoldActive = true
      end

      collective = clamp(collective, CFG.rotor.collectiveMinDeg, CFG.rotor.collectiveMaxDeg)

      -- Yaw hold if heading available; otherwise damp yaw rate
      if mode.yawHold then
        local yawErr = normAngleDeg(state.targetHeading - yaw)
        local yawStab = pidStep(pids.yaw, yawErr, dtTarget)
        tailCmd = tailCmd + yawStab
      else
        local yawRateErr = -yawRate
        tailCmd = tailCmd + pidStep(pids.yaw, yawRateErr, dtTarget)
      end

      tailCmd = clamp(tailCmd, CFG.tail.minCmd, CFG.tail.maxCmd)

      local bladePitches = allocateMainRotorBladePitch(collective, rollCmd, pitchCmd)
      setMainRotorBladePitches(bladePitches)
      setTailRotor(tailCmd)
    else
      -- Disarmed / missing mandatory sensors => safe neutral
      applySafeNeutral()
      resetControllers()
      rollCmd, pitchCmd = 0, 0
      collective = CFG.safe.neutralCollectiveDeg
      tailCmd = CFG.safe.neutralTailCmd
      altHoldActive = false
    end

    state.telemetry.rollCmd = rollCmd
    state.telemetry.pitchCmd = pitchCmd
    state.telemetry.collectiveCmd = collective
    state.telemetry.tailCmd = tailCmd
    state.telemetry.altitudeHoldActive = altHoldActive

    drawUI()

    -- Deterministic update period
    local elapsed = os.clock() - t0
    local sleepTime = dtTarget - elapsed
    if sleepTime > 0 then
      sleep(sleepTime)
    else
      sleep(0)
    end
  end

  applySafeNeutral()
end

-- =========================
-- Start
-- =========================
term.setBackgroundColor(colors.black)
term.setTextColor(colors.white)
term.clear()

drawUI()
parallel.waitForAny(controlLoop, uiLoop)

applySafeNeutral()
term.setCursorPos(1, 1)
term.setBackgroundColor(colors.black)
term.setTextColor(colors.white)
term.clear()
print("Helicopter autopilot stopped.")
