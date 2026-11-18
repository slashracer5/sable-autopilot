-- build.lua : runs INSIDE ComputerCraft. Bundles /ap and /ap.lua into one self-extracting installer.
--   build [outfile] [version] [srcdir]   default outfile: apinstall.lua, default srcdir: "." (the folder holding ap.lua and ap/)
-- In this repository the sources are in src/, so copy the repo to the computer and run:  build dist/apinstall.lua 1.5.3 src
-- The installer needs nothing but the CC shell:  apinstall            (install / update)
--                                               apinstall check      (verify an installed copy)
local outName = ({ ... })[1] or "apinstall.lua"
local version = ({ ... })[2] or os.date("%Y-%m-%d %H:%M")
local srcDir = ({ ... })[3] or ""

-- paths in `files` are relative to srcDir (they become the install paths); real paths are fs.combine(srcDir, p)
local files = {}
local function walk(dir)
  for _, n in ipairs(fs.list(fs.combine(srcDir, dir))) do
    local p = fs.combine(dir, n)
    if fs.isDir(fs.combine(srcDir, p)) then walk(p) elseif p:match("%.lua$") then files[#files + 1] = p end
  end
end
walk("ap")
if fs.exists(fs.combine(srcDir, "ap.lua")) then files[#files + 1] = "ap.lua" end
table.sort(files)

local function readAll(p) local f = fs.open(fs.combine(srcDir, p), "r") local s = f.readAll() f.close() return s end

local out = {}
local function emit(s) out[#out + 1] = s end
emit("-- apinstall: self-extracting installer for the general multirotor autopilot (ap). Version " .. version .. "\n")
emit("local VERSION = " .. string.format("%q", version) .. "\n")
emit("local FILES = {\n")
local total = 0
for _, p in ipairs(files) do
  local src = readAll(p)
  total = total + #src
  local level = 0
  while src:find("]" .. string.rep("=", level) .. "]", 1, true) do level = level + 1 end
  local eq = string.rep("=", level)
  emit(string.format("  [%q] = [%s[\n", p, eq))
  emit(src)
  emit(string.format("]%s],\n", eq))
end
emit("}\n")
emit([==[
local args = { ... }
local names = {}
for p in pairs(FILES) do names[#names + 1] = p end
table.sort(names)

local function read(p) local f = fs.open(p, "r") if not f then return nil end local s = f.readAll() f.close() return s end

if args[1] == "check" then
  local bad = 0
  for _, p in ipairs(names) do
    if read("/" .. p) ~= FILES[p] then print("differs or missing: /" .. p) bad = bad + 1 end
  end
  print(bad == 0 and ("ap " .. VERSION .. ": all " .. #names .. " files match") or (bad .. " file(s) need updating: run apinstall"))
  return
end

print("Installing ap " .. VERSION)
for _, p in ipairs(names) do
  local f = fs.open("/" .. p, "w")
  if not f then error("cannot write /" .. p, 0) end
  f.write(FILES[p])
  f.close()
  if read("/" .. p) ~= FILES[p] then error("verify failed for /" .. p, 0) end
  print("  /" .. p)
end
local v = fs.open("/ap/VERSION", "w") v.write(VERSION) v.close()
-- run `ap recover` first thing at boot: if a flight was interrupted (chunk unload, crash) it lands the ship; otherwise it does nothing
local line = 'shell.run("/ap.lua", "recover")'
local st = read("/startup.lua") or ""
if not st:find('"recover"', 1, true) then
  local sf = fs.open("/startup.lua", "w")
  sf.write(line .. "\n" .. st)
  sf.close()
  print("added `ap recover` to /startup.lua (lands the ship after an interrupted flight)")
end
if not fs.exists("/ap_data") then fs.makeDir("/ap_data") end
print(#names .. " files installed. Your settings in /ap_data are kept.")
print("Next:  ap selftest     (checks the code, nothing flies)")
print("       ap scan         (finds the rotors on this ship)")
]==])
local f = fs.open(outName, "w")
f.write(table.concat(out))
f.close()
print(string.format("built %s: %d files, %d bytes of source, %d bytes installer", outName, #files, total, fs.getSize(outName)))
