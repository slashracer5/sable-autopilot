-- ap/lib/sim.lua : tiny rigid-body multirotor simulator used by `ap selftest` (pure Lua, runs inside ComputerCraft).
-- Rotors produce thrust along their tilted direction with a pure command delay; no reaction torque (the gyroscopic
-- bearings do not produce any on the real ships). Used to prove the allocator and the controller before anything flies.
local M = require("m")
local S = {}

-- cfg: {mass, inertia={x,y,z}, g, rotors={{x,y,z,kT,tmax}...}, delay (s), cone (deg), drag, dt}
function S.new(cfg)
  local s = {
    cfg = cfg,
    m = cfg.mass, I = cfg.inertia, g = cfg.g or 11,
    p = { x = 0, y = cfg.y0 or 20, z = 0 },
    v = { x = 0, y = 0, z = 0 },
    q = { w = 1, x = 0, y = 0, z = 0 },
    w = { x = 0, y = 0, z = 0 },
    t = 0,
    T = {}, tilt = {},              -- delivered thrust / tilt
    queue = {},
    thrustSign = 1,
  }
  for j, r in ipairs(cfg.rotors) do s.T[j] = 0; s.tilt[j] = { 0, 0 } end
  return s
end

local function cross(a, b)
  return { x = a.y * b.z - a.z * b.y, y = a.z * b.x - a.x * b.z, z = a.x * b.y - a.y * b.x }
end

-- advance one control period dt with commands cmdT[j] (thrust) and cmdTilt[j] = {tx,tz}; sub-stepped internally
function S.step(s, cmdT, cmdTilt, dt)
  local cfg = s.cfg
  local delay = cfg.delay or 0.2
  -- copy the commands: callers reuse their arrays every step, so queued entries must not alias them (the delay would vanish)
  local qT, qTilt = {}, {}
  for j = 1, #cfg.rotors do
    qT[j] = cmdT[j]
    local tl = cmdTilt and cmdTilt[j] or { 0, 0 }
    qTilt[j] = { tl[1], tl[2] }
  end
  s.queue[#s.queue + 1] = { T = qT, tilt = qTilt, t = s.t }
  -- release the newest command that is at least `delay` old
  local use
  while #s.queue > 0 and s.queue[1].t <= s.t - delay + 1e-9 do use = table.remove(s.queue, 1) end
  if use then
    for j = 1, #cfg.rotors do
      local r = cfg.rotors[j]
      s.T[j] = M.clamp(use.T[j] * (r.gainErr or 1), r.tmin or 0, r.tmax or math.huge)
      local tl = use.tilt and use.tilt[j] or { 0, 0 }
      local cone = math.tan((cfg.cone or 11.5) / M.DEG)
      local mag = math.sqrt(tl[1] ^ 2 + tl[2] ^ 2)
      local k = (mag > cone) and cone / mag or 1
      local ls = cfg.latScale     -- optional weak sides: tilt effectiveness per direction {xp, xn, zp, zn}
      local fx = (ls and ((tl[1] >= 0) and (ls.xp or 1) or (ls.xn or 1))) or 1
      local fz = (ls and ((tl[2] >= 0) and (ls.zp or 1) or (ls.zn or 1))) or 1
      s.tilt[j] = { tl[1] * k * fx, tl[2] * k * fz }
    end
  end
  local sub = math.max(1, math.floor(dt / 0.01 + 0.5))
  local h = dt / sub
  for _ = 1, sub do
    local F = { x = 0, y = 0, z = 0 }
    local tau = { x = 0, y = 0, z = 0 }
    -- thrust drops with air flowing along the propeller axis (v . axis): climbing or leaning INTO the motion loses lift, leaning back gains
    local axW = M.qrot(s.q, { x = 0, y = 1, z = 0 })
    local axial = s.v.x * axW.x + s.v.y * axW.y + s.v.z * axW.z
    local infl = M.clamp(1 - (cfg.inflowLoss or 0) * axial, 0.2, 1.4) * (cfg.pressure and cfg.pressure(s.p.y) or 1)
    for j, r in ipairs(cfg.rotors) do
      local tl = s.tilt[j]
      local n = math.sqrt(tl[1] ^ 2 + 1 + tl[2] ^ 2)
      local f = { x = s.T[j] * infl * tl[1] / n, y = s.T[j] * infl / n, z = s.T[j] * infl * tl[2] / n }
      F.x, F.y, F.z = F.x + f.x, F.y + f.y, F.z + f.z
      local c = cross({ x = r.x, y = r.y, z = r.z }, f)
      tau.x, tau.y, tau.z = tau.x + c.x, tau.y + c.y, tau.z + c.z
    end
    local ts = cfg.torqueScale      -- optional torque authority error {x, y, z}
    if ts then tau.x, tau.y, tau.z = tau.x * (ts.x or 1), tau.y * (ts.y or 1), tau.z * (ts.z or 1) end
    local Fw = M.qrot(s.q, F)
    local drag = cfg.drag or 0
    local sp = math.sqrt(s.v.x ^ 2 + s.v.y ^ 2 + s.v.z ^ 2)
    local ax = Fw.x / s.m - drag * sp * s.v.x
    local ay = Fw.y / s.m - s.g - drag * sp * s.v.y
    local az = Fw.z / s.m - drag * sp * s.v.z
    s.v.x, s.v.y, s.v.z = s.v.x + ax * h, s.v.y + ay * h, s.v.z + az * h
    s.p.x, s.p.y, s.p.z = s.p.x + s.v.x * h, s.p.y + s.v.y * h, s.p.z + s.v.z * h
    local Iw = { x = s.I.x * s.w.x, y = s.I.y * s.w.y, z = s.I.z * s.w.z }
    local gyro = cross(s.w, Iw)
    s.w.x = s.w.x + (tau.x - gyro.x) / s.I.x * h
    s.w.y = s.w.y + (tau.y - gyro.y) / s.I.y * h
    s.w.z = s.w.z + (tau.z - gyro.z) / s.I.z * h
    s.q = M.qintegrate(s.q, s.w, h)
    s.t = s.t + h
  end
end

-- Standard test layouts. arms in metres relative to the CoM. mass 80, g 11 like the real ships.
function S.layout(name)
  local function ring(n, rad, phase)
    local out = {}
    for j = 1, n do
      local a = (phase or 0) + 2 * math.pi * (j - 1) / n
      out[j] = { x = rad * math.cos(a), y = 1.5, z = rad * math.sin(a) }
    end
    return out
  end
  if name == "tri" then
    -- the real tricopter: aft rotor behind, two forward
    return { { x = 0, y = 1.3, z = -6.5 }, { x = 4, y = 1.3, z = 2.5 }, { x = -4, y = 1.3, z = 2.5 } }, 36.5
  elseif name == "tandem" then
    -- two-rotor tandem helicopter (the real one: mass 537, rotors 17 m apart on the x axis, 3-5 m above the centre of mass)
    return { { x = -11.1, y = 5.0, z = 0 }, { x = 5.9, y = 3.0, z = 0 } }, 537
  elseif name == "quad+" then
    return { { x = -9.7, y = 1.9, z = 0 }, { x = 8.3, y = 1.9, z = 0 }, { x = -0.7, y = 1.9, z = 10 }, { x = -0.7, y = 1.9, z = -10 } }, 80.5
  elseif name == "quadx" then return ring(4, 8, math.pi / 4), 60
  elseif name == "hex" then return ring(6, 9, 0), 90
  elseif name == "octo" then return ring(8, 10, 0.2), 110
  elseif name == "offset" then
    -- asymmetric five-rotor craft with the heavy side forward
    return { { x = 6, y = 1, z = 0 }, { x = -3, y = 1, z = 7 }, { x = -3, y = 1, z = -7 }, { x = -9, y = 1, z = 3 }, { x = -9, y = 1, z = -3 } }, 70
  end
end

-- build a sim config from a layout: hover thrust per rotor ~ weight / n, tmax = 2.4 x that
function S.config(name, over)
  local arms, mass = S.layout(name)
  local g = 11
  local rotors = {}
  local share = mass * g / #arms
  local ixx, izz, iyy = 0, 0, 0
  for j, a in ipairs(arms) do
    -- the offset craft needs one rotor to carry ~45 % of the weight, so give it more headroom
    rotors[j] = { x = a.x, y = a.y, z = a.z, tmax = share * (name == "offset" and 3.6 or 2.4), tmin = 0 }
    ixx = ixx + (a.y ^ 2 + a.z ^ 2) * mass / #arms
    izz = izz + (a.x ^ 2 + a.y ^ 2) * mass / #arms
    iyy = iyy + (a.x ^ 2 + a.z ^ 2) * mass / #arms
  end
  local cfg = { mass = mass, g = g, rotors = rotors, inertia = { x = ixx * 1.4, y = iyy * 1.4, z = izz * 1.4 }, delay = 0.2, cone = 11.5, drag = 0.0005 }
  if name == "tandem" then cfg.inertia = { x = 5547, y = 35709, z = 36127 } end      -- measured on the real tandem
  for k, v in pairs(over or {}) do cfg[k] = v end
  return cfg
end

return S
