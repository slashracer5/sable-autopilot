-- ap/lib/m.lua : small pure math helpers (vectors, quaternions, dense linear algebra). No peripherals.
local M = {}

M.DEG = 180 / math.pi

function M.clamp(v, lo, hi) if v < lo then return lo elseif v > hi then return hi end return v end
function M.wrap180(a) return (a + 180) % 360 - 180 end

-- quaternions {w,x,y,z}, body->world ----------------------------------------------------------
function M.qnorm(q)
  local n = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
  if n == 0 then return { w = 1, x = 0, y = 0, z = 0 } end
  return { w = q.w / n, x = q.x / n, y = q.y / n, z = q.z / n }
end
function M.qconj(q) return { w = q.w, x = -q.x, y = -q.y, z = -q.z } end
function M.qrot(q, v)
  local tx = 2 * (q.y * v.z - q.z * v.y)
  local ty = 2 * (q.z * v.x - q.x * v.z)
  local tz = 2 * (q.x * v.y - q.y * v.x)
  return {
    x = v.x + q.w * tx + (q.y * tz - q.z * ty),
    y = v.y + q.w * ty + (q.z * tx - q.x * tz),
    z = v.z + q.w * tz + (q.x * ty - q.y * tx),
  }
end
function M.qmul(a, b)
  return {
    w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  }
end
-- integrate body angular velocity w (body frame, rad/s) over dt
function M.qintegrate(q, w, dt)
  local ang = math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z) * dt
  if ang < 1e-12 then return q end
  local s = math.sin(ang / 2) / (ang / dt)
  return M.qnorm(M.qmul(q, { w = math.cos(ang / 2), x = w.x * s, y = w.y * s, z = w.z * s }))
end

-- rotation that takes orientation qa to qb, as a rotation VECTOR in the body frame of qa (rad): angle * axis
function M.qdelta(qa, qb)
  local r = M.qmul(M.qconj(M.qnorm(qa)), M.qnorm(qb))
  if r.w < 0 then r = { w = -r.w, x = -r.x, y = -r.y, z = -r.z } end
  local s = math.sqrt(r.x * r.x + r.y * r.y + r.z * r.z)
  if s < 1e-12 then return { x = 0, y = 0, z = 0 } end
  local ang = 2 * math.atan2(s, r.w)
  return { x = r.x / s * ang, y = r.y / s * ang, z = r.z / s * ang }
end

-- Attitude from a body->world quaternion. thX/thZ: tilt angles (deg) of body up about the body X/Z axes,
-- psi: heading (deg, rotation about world +Y, body +X projected on the ground plane), tilt total.
function M.attitude(q)
  q = M.qnorm(q)
  local up = M.qrot(M.qconj(q), { x = 0, y = 1, z = 0 })
  local upW = M.qrot(q, { x = 0, y = 1, z = 0 })
  local fw = M.qrot(q, { x = 1, y = 0, z = 0 })
  return {
    thX = math.atan2(-up.x, up.y) * M.DEG,
    thZ = math.atan2(-up.z, up.y) * M.DEG,
    tilt = math.acos(M.clamp(upW.y, -1, 1)) * M.DEG,
    cosTilt = upW.y,
    psi = -math.atan2(fw.z, fw.x) * M.DEG,
  }
end

-- dense linear algebra on arrays of rows --------------------------------------------------------
function M.mat(r, c, f)
  local A = {}
  for i = 1, r do A[i] = {} for j = 1, c do A[i][j] = f and f(i, j) or 0 end end
  return A
end
function M.transpose(A)
  local r, c = #A, #A[1]
  return M.mat(c, r, function(i, j) return A[j][i] end)
end
function M.mul(A, B)
  local r, k, c = #A, #B, #B[1]
  return M.mat(r, c, function(i, j)
    local s = 0
    for t = 1, k do s = s + A[i][t] * B[t][j] end
    return s
  end)
end
function M.mulv(A, v)
  local out = {}
  for i = 1, #A do
    local s = 0
    for j = 1, #v do s = s + A[i][j] * v[j] end
    out[i] = s
  end
  return out
end

-- Solve A x = b (square) by Gaussian elimination with partial pivoting. Returns nil if singular.
function M.solve(A, b)
  local n = #A
  local a = {}
  for i = 1, n do
    a[i] = {}
    for j = 1, n do a[i][j] = A[i][j] end
    a[i][n + 1] = b[i]
  end
  for col = 1, n do
    local piv, best = col, math.abs(a[col][col])
    for r = col + 1, n do
      if math.abs(a[r][col]) > best then piv, best = r, math.abs(a[r][col]) end
    end
    if best < 1e-12 then return nil end
    a[col], a[piv] = a[piv], a[col]
    for r = col + 1, n do
      local f = a[r][col] / a[col][col]
      if f ~= 0 then for c = col, n + 1 do a[r][c] = a[r][c] - f * a[col][c] end end
    end
  end
  local x = {}
  for i = n, 1, -1 do
    local s = a[i][n + 1]
    for j = i + 1, n do s = s - a[i][j] * x[j] end
    x[i] = s / a[i][i]
  end
  return x
end

-- Minimum-norm solution of  A x = b  with a per-unknown weight w (cost = sum w_j x_j^2), A is m x n with m <= n.
-- x = W^-1 A^T (A W^-1 A^T)^-1 b.  Returns nil if A is rank deficient.
function M.minNorm(A, b, w)
  local m, n = #A, #A[1]
  local G = M.mat(m, m)
  for i = 1, m do
    for j = 1, m do
      local s = 0
      for k = 1, n do s = s + A[i][k] * A[j][k] / (w and w[k] or 1) end
      G[i][j] = s
    end
  end
  local y = M.solve(G, b)
  if not y then return nil end
  local x = {}
  for k = 1, n do
    local s = 0
    for i = 1, m do s = s + A[i][k] * y[i] end
    x[k] = s / (w and w[k] or 1)
  end
  return x
end

return M
