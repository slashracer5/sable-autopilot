-- ap/lib/alloc.lua : control allocation for ANY number of upward-facing rotors at arbitrary positions.
--
-- Body frame: +Y up, rotor j at arm r_j = (x,y,z) from the centre of mass (metres). Thrust T_j along up, with a small tilt
-- (tx_j, tz_j) (tangent of the tilt angle) that the gyroscopic bearing can add. Force F = sum T_j (tx, 1, tz), torque = r x F:
--   tau_x = y Fz - z Fy      tau_y = z Fx - x Fz      tau_z = x Fy - y Fx
--
-- 1. THRUST allocation (collective + roll + pitch): the rows [1; -z; x] map thrusts to (Fy, tau_x, tau_z). T = c*u + tau_x*ax + tau_z*az with
--    u / ax / az the minimum-norm solutions of the unit demands, so u is a PURE collective (no torque) and ax/az are pure torques (no net lift).
--    Limits are met by moving c first (attitude has priority) and only then shrinking the attitude part.
--    A torque row exists only when the rotors are spread along the matching axis. A TANDEM (rotors all on the x axis, z = 0) cannot make roll
--    torque with thrust (rollTilt = true); rotors all on the z axis cannot make pitch torque (pitchTilt = true). Those torques then come from
--    the bearing TILTS (lever arm = height of the rotors above the centre of mass): roll tau_x = sum y T tz, pitch tau_z = -sum y T tx.
-- 2. TILT allocation: unknowns (tx_j, tz_j). Rows: lateral force Fx (unless pitch uses the tx), Fz (unless roll uses the tz), yaw torque
--    tau_y, and the roll / pitch torque of the tilt-driven axes. Minimum-norm solution, capped by the cone angle. The torque the tilts cause on a
--    thrust-driven axis is returned so the thrust allocation can cancel it.
local M = require("m")
local A = {}

local SPREAD = 0.75        -- m: rotors closer than this to an axis count as lying on it
local LEVER = 0.5          -- m: mean height above the centre of mass needed for a tilt-driven torque

-- rotors: array of {x=,y=,z=, tmax=, tmin=}; weights: optional cost weight per rotor (default 1)
function A.new(rotors, weights)
  local n = #rotors
  if n < 2 then return nil, "need at least 2 rotors" end
  local zspread, xspread, ysum = false, false, 0
  for _, r in ipairs(rotors) do
    if math.abs(r.z) > SPREAD then zspread = true end
    if math.abs(r.x) > SPREAD then xspread = true end
    ysum = ysum + math.abs(r.y)
  end
  if not (zspread or xspread) then return nil, "all rotors sit at one point: no lever arm for any torque" end
  local rollTilt, pitchTilt = not zspread, not xspread
  if (rollTilt or pitchTilt) and ysum / n < LEVER then
    return nil, "the rotors are collinear and level with the centre of mass: no way to make roll/pitch torque (needs rotors spread both ways, or above/below the centre of mass)"
  end
  local rows = { function() return 1 end }
  local rollRow, pitchRow
  if zspread then rows[#rows + 1] = function(r) return -r.z end; rollRow = #rows end
  if xspread then rows[#rows + 1] = function(r) return r.x end; pitchRow = #rows end
  if #rows > n then return nil, "not enough rotors to control lift and the torques their spread allows" end
  local B = M.mat(#rows, n, function(i, j) return rows[i](rotors[j]) end)
  local w = {}
  for j = 1, n do w[j] = weights and weights[j] or 1 end
  local function unit(k)
    local b = {}
    for i = 1, #rows do b[i] = (i == k) and 1 or 0 end
    return M.minNorm(B, b, w)
  end
  local u = unit(1)
  local ax = rollRow and unit(rollRow) or nil
  local az = pitchRow and unit(pitchRow) or nil
  if not u or (rollRow and not ax) or (pitchRow and not az) then return nil, "rotor layout cannot produce the lift and torques it should (degenerate)" end
  return { n = n, rotors = rotors, B = B, u = u, ax = ax, az = az, w = w, rollTilt = rollTilt, pitchTilt = pitchTilt }
end

-- Thrusts for total vertical force c, torques tx, tz (only the thrust-driven ones are used). Returns T (array), info
function A.thrust(self, c, tx, tz)
  local n = self.n
  local a, T = {}, {}
  for j = 1, n do a[j] = (self.ax and tx * self.ax[j] or 0) + (self.az and tz * self.az[j] or 0) end
  local function interval(k)
    local lo, hi = -math.huge, math.huge
    for j = 1, n do
      local r = self.rotors[j]
      local base = k * a[j]
      local uj = self.u[j]
      local tl, th = (r.tmin or 0) - base, (r.tmax or math.huge) - base
      if uj > 1e-9 then
        lo, hi = math.max(lo, tl / uj), math.min(hi, th / uj)
      elseif uj < -1e-9 then
        lo, hi = math.max(lo, th / uj), math.min(hi, tl / uj)
      elseif base < (r.tmin or 0) - 1e-9 or base > (r.tmax or math.huge) + 1e-9 then
        return 1, 0           -- infeasible regardless of c
      end
    end
    return lo, hi
  end
  local k = 1
  local lo, hi = interval(1)
  if lo > hi then
    -- attitude demand alone does not fit: shrink it until some collective works
    local klo, khi = 0, 1
    for _ = 1, 30 do
      local km = (klo + khi) / 2
      local l, h = interval(km)
      if l <= h then klo = km else khi = km end
    end
    k = klo
    lo, hi = interval(k)
    if lo > hi then lo, hi = c, c end
  end
  local cc = M.clamp(c, lo, hi)
  for j = 1, n do T[j] = cc * self.u[j] + k * a[j] end
  return T, { c = cc, attScale = k, saturated = (k < 1) or (cc ~= c) }
end

-- Tilts (tx_j, tz_j) for lateral force (Fx, Fz), yaw torque tauY and, for tilt-driven axes, roll torque tauRoll / pitch torque tauPitch, given the
-- rotor thrusts T. coneTan = tan(max tilt). Returns tilts = {{tx,tz},...}, info{scale, tauX, tauZ (torques the tilts add), Fx, Fz, tauY produced}
function A.tilts(self, T, Fx, Fz, tauY, coneTan, tauRoll, tauPitch)
  local n = self.n
  local R = self.rotors
  local rows, rhs = {}, {}
  local function add(fn, v)
    local row = {}
    for j = 1, 2 * n do row[j] = 0 end
    for j = 1, n do
      local ctx, ctz = fn(R[j], T[j])
      row[j], row[n + j] = ctx, ctz
    end
    rows[#rows + 1] = row
    rhs[#rhs + 1] = v
  end
  if not self.pitchTilt then add(function(r, t) return t, 0 end, Fx) end                       -- lateral force along x
  if not self.rollTilt then add(function(r, t) return 0, t end, Fz) end                        -- lateral force along z
  add(function(r, t) return r.z * t, -r.x * t end, tauY)                                       -- yaw torque
  if self.rollTilt then add(function(r, t) return 0, r.y * t end, tauRoll or 0) end            -- roll torque from the z tilts
  if self.pitchTilt then add(function(r, t) return -r.y * t, 0 end, tauPitch or 0) end         -- pitch torque from the x tilts
  local sol = M.minNorm(rows, rhs)
  if not sol then return nil, "tilt allocation singular" end
  local worst = 0
  for j = 1, n do worst = math.max(worst, math.sqrt(sol[j] ^ 2 + sol[n + j] ^ 2)) end
  local s = 1
  if worst > coneTan then s = coneTan / worst end
  local out, tauX, tauZ, pFx, pFz, pTy = {}, 0, 0, 0, 0, 0
  for j = 1, n do
    local tx, tz = sol[j] * s, sol[n + j] * s
    out[j] = { tx, tz }
    tauX = tauX + R[j].y * T[j] * tz
    tauZ = tauZ - R[j].y * T[j] * tx
    pFx = pFx + T[j] * tx
    pFz = pFz + T[j] * tz
    pTy = pTy + T[j] * (R[j].z * tx - R[j].x * tz)
  end
  return out, { scale = s, tauX = tauX, tauZ = tauZ, Fx = pFx, Fz = pFz, tauY = pTy }
end

return A
