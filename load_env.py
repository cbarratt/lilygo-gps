Import("env")
import os

# Inject build-time string defines from a gitignored .env (KEY=VALUE per line).
# Environment variables of the same name (e.g. GitHub Actions secrets) override .env.
vals = {}
envfile = os.path.join(env["PROJECT_DIR"], ".env")
if os.path.isfile(envfile):
    for line in open(envfile):
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            vals[k.strip()] = v.strip()

for k in ("WIFI_SSID", "WIFI_PASS", "OTA_TOKEN"):
    if os.environ.get(k):
        vals[k] = os.environ[k]

for k, v in vals.items():
    env.Append(CPPDEFINES=[(k, env.StringifyMacro(v))])
    print(f"load_env: injected {k}")
