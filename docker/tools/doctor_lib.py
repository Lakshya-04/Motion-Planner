#!/usr/bin/env python3
"""Environment doctor: what is wrong with this machine before you build or run the stack.

Run it through docker/tools/doctor.sh (or `make doctor`). The decision logic is pure functions (thresholds,
parsers, classifiers, JSON assembly) so the tests can feed them fakes; the checks at the bottom only gather
facts from the machine and hand them to those functions. Stdlib only.
"""
import argparse
import concurrent.futures
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

OK, WARN, FAIL, SKIP, INFO = "ok", "warn", "fail", "skip", "info"

DISK_WARN_GB = 40.0
DISK_FAIL_GB = 15.0
RAM_WARN_GB = 16.0
RAM_FAIL_GB = 8.0
MIN_CC_WARN = 7.5

IMAGES = [  # tag, target, required
    ("motion-planner:build", "build", True),
    ("motion-planner:sim-gazebo", "sim-gazebo", False),
    ("motion-planner:gpu", "gpu", False),
    ("motion-planner:full", "full", False),
]
ISAAC_IMAGE = "nvcr.io/nvidia/isaac-sim"

UBUNTU_MIRRORS = [
    ("archive.ubuntu.com", "http://archive.ubuntu.com/ubuntu/dists/noble/InRelease", "noble"),
    ("security.ubuntu.com", "http://security.ubuntu.com/ubuntu/dists/noble-security/InRelease", "noble-security"),
    ("azure.archive.ubuntu.com", "http://azure.archive.ubuntu.com/ubuntu/dists/noble/InRelease", "noble"),
]
TLS_HOSTS = [
    ("GitHub", "github.com", "https://github.com/"),
    ("PyPI", "pypi.org", "https://pypi.org/simple/pip/"),
    ("PyTorch wheels", "download.pytorch.org", "https://download.pytorch.org/whl/cpu/"),
    ("conda", "conda.anaconda.org", "https://conda.anaconda.org/conda-forge/noarch/current_repodata.json"),
]
INRELEASE_MIN_BYTES = 5000  # a real InRelease is tens of KB; less is an error or portal page

PUBLIC_CA = (
    "let's encrypt", "isrg", "digicert", "amazon", "google trust", "sectigo", "globalsign", "microsoft",
    "cloudflare", "usertrust", "starfield", "godaddy", "entrust", "comodo", "baltimore", "certum", "identrust",
    "buypass", "ssl.com",
)
INTERCEPT_MARKERS = (
    "fortinet", "fortigate", "palo alto", "zscaler", "netskope", "umbrella", "sophos", "check point",
    "checkpoint", "sonicwall", "blue coat", "bluecoat", "forcepoint", "mcafee", "skyhigh", "watchguard",
    "iboss", "barracuda", "firewall", "inspection", "proxy", "websense", "decrypt", "mitm", "interception",
)


class Result:
    def __init__(self, section, name, status, detail="", hint=""):
        self.section, self.name, self.status, self.detail, self.hint = section, name, status, detail, hint

    def as_dict(self):
        return {"section": self.section, "name": self.name, "status": self.status,
                "detail": self.detail, "hint": self.hint}


# ---------------------------------------------------------------- pure logic

def disk_status(free_gb):
    if free_gb < DISK_FAIL_GB:
        return FAIL
    if free_gb < DISK_WARN_GB:
        return WARN
    return OK


def ram_status(total_gb):
    if total_gb < RAM_FAIL_GB:
        return FAIL
    if total_gb < RAM_WARN_GB:
        return WARN
    return OK


def parse_version(text):
    """First dotted number in text as a tuple of ints, or None. 'Docker version 27.3.1, build x' -> (27, 3, 1)."""
    m = re.search(r"(\d+)(?:\.(\d+))?(?:\.(\d+))?", text or "")
    if not m:
        return None
    return tuple(int(g) for g in m.groups() if g is not None)


def parse_mujoco_version(header_text):
    """mjVERSION_HEADER is major*100 + minor*10 + patch (331 -> 3.3.1, 400 -> 4.0.0)."""
    m = re.search(r"#\s*define\s+mjVERSION_HEADER\s+(\d+)", header_text or "")
    if not m:
        return None
    v = int(m.group(1))
    return "%d.%d.%d" % (v // 100, (v // 10) % 10, v % 10)


def parse_df_free_gb(output):
    """Available space in GB from `df -Pk <path>` output, or None."""
    lines = [ln for ln in (output or "").strip().splitlines() if ln.strip()]
    if len(lines) < 2:
        return None
    cols = lines[-1].split()
    if len(cols) < 4 or not cols[3].isdigit():
        return None
    return int(cols[3]) / (1024.0 * 1024.0)


def parse_meminfo_gb(text):
    m = re.search(r"MemTotal:\s+(\d+)\s*kB", text or "")
    return int(m.group(1)) / (1024.0 * 1024.0) if m else None


def parse_nvidia_smi(csv_text):
    """Rows of `name, driver, memory[, compute_cap]` -> list of dicts. compute_cap is None on old drivers."""
    gpus = []
    for line in (csv_text or "").strip().splitlines():
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 3 or not parts[0]:
            continue
        cc = None
        if len(parts) >= 4:
            try:
                cc = float(parts[3])
            except ValueError:
                cc = None
        gpus.append({"name": parts[0], "driver": parts[1], "memory": parts[2], "cc": cc})
    return gpus


def classify_issuer(issuer):
    """('public' | 'interception' | 'unknown', short name) from an `openssl x509 -issuer` line."""
    text = (issuer or "").strip()
    if not text:
        return "unknown", ""
    low = text.lower()
    name = re.sub(r"^issuer\s*=\s*", "", text, flags=re.I).strip()
    m = re.search(r"\bO\s*=\s*([^,/]+)", name) or re.search(r"\bCN\s*=\s*([^,/]+)", name)
    short = m.group(1).strip().strip('"') if m else name
    if any(k in low for k in INTERCEPT_MARKERS):
        return "interception", short
    if any(k in low for k in PUBLIC_CA):
        return "public", short
    return "unknown", short


def classify_docker_error(stderr):
    low = (stderr or "").lower()
    if "permission denied" in low:
        return "permission"
    if "cannot connect" in low or "is the docker daemon running" in low or "no such file" in low \
            or "connection refused" in low:
        return "daemon_down"
    return "other"


def human_size(n):
    n = float(n)
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1000 or unit == "GB":
            return "%.0f %s" % (n, unit) if unit in ("B", "KB") else "%.1f %s" % (n, unit)
        n /= 1000.0


def classify_inrelease(code, size):
    """(status, detail) for one InRelease fetch."""
    if code != 200:
        return FAIL, "HTTP %s" % code
    if size < INRELEASE_MIN_BYTES:
        return WARN, "only %d bytes, looks like an error or portal page" % size
    return OK, "%d KB" % (size // 1024)


def compare_mirrors(fetched):
    """fetched: {host: (suite, sha256)} for successful fetches. Hosts serving the same suite must agree."""
    by_suite = {}
    for host, (suite, digest) in fetched.items():
        by_suite.setdefault(suite, {}).setdefault(digest, []).append(host)
    out = []
    for suite, groups in by_suite.items():
        if len(groups) > 1:
            out.append((suite, sorted(sum(groups.values(), []))))
    return out


def check_ros_env(env):
    """Results for the ROS variables in env. Pure, so a fake environment can drive it."""
    sec = "ROS environment"
    distro = env.get("ROS_DISTRO")
    if not distro:
        return [Result(sec, "ROS_DISTRO", SKIP, "not sourced (normal on the host; containers source it)")]
    r = [Result(sec, "ROS_DISTRO", OK if distro in ("jazzy", "humble") else WARN, distro,
                "" if distro in ("jazzy", "humble") else "the stack targets jazzy (humble builds too)")]
    dom = env.get("ROS_DOMAIN_ID")
    if dom is None:
        r.append(Result(sec, "ROS_DOMAIN_ID", INFO, "unset (0)",
                        "give each simulation its own id, see docs/docker.md"))
    elif dom.isdigit() and 0 <= int(dom) <= 232:
        r.append(Result(sec, "ROS_DOMAIN_ID", OK, dom))
    else:
        r.append(Result(sec, "ROS_DOMAIN_ID", FAIL, repr(dom), "use an integer from 0 to 232"))
    rmw = env.get("RMW_IMPLEMENTATION")
    r.append(Result(sec, "RMW_IMPLEMENTATION", INFO if rmw else SKIP, rmw or "unset (distro default)"))
    if env.get("ROS_LOCALHOST_ONLY"):
        r.append(Result(sec, "ROS_LOCALHOST_ONLY", WARN, "set to %s, deprecated" % env["ROS_LOCALHOST_ONLY"],
                        "use ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST instead"))
    rng = env.get("ROS_AUTOMATIC_DISCOVERY_RANGE")
    if rng:
        r.append(Result(sec, "ROS_AUTOMATIC_DISCOVERY_RANGE", INFO, rng))
    for var in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE", "CYCLONEDDS_URI"):
        val = env.get(var)
        if not val:
            continue
        path = val[len("file://"):] if val.startswith("file://") else val
        if val.startswith("<"):
            r.append(Result(sec, var, INFO, "inline XML"))
        elif not os.path.exists(path):
            r.append(Result(sec, var, WARN, "points at a missing file: %s" % path, "fix the path or unset it"))
        else:
            r.append(Result(sec, var, INFO, path))
    fast = env.get("FASTRTPS_DEFAULT_PROFILES_FILE") or env.get("FASTDDS_DEFAULT_PROFILES_FILE")
    if fast and rmw and "fastrtps" not in rmw:
        r.append(Result(sec, "DDS profile vs RMW", WARN, "a Fast DDS profile is set but RMW is %s" % rmw,
                        "unset the profile or switch RMW_IMPLEMENTATION to rmw_fastrtps_cpp"))
    if env.get("CYCLONEDDS_URI") and rmw and "cyclone" not in rmw:
        r.append(Result(sec, "DDS profile vs RMW", WARN, "CYCLONEDDS_URI is set but RMW is %s" % rmw,
                        "unset it or switch RMW_IMPLEMENTATION to rmw_cyclonedds_cpp"))
    return r


def summarize(results):
    counts = {s: 0 for s in (OK, WARN, FAIL, SKIP, INFO)}
    for r in results:
        counts[r.status] += 1
    return counts


def exit_code(results):
    return 1 if summarize(results)[FAIL] else 0


def build_json(results):
    return {"summary": summarize(results), "exit_code": exit_code(results),
            "checks": [r.as_dict() for r in results]}


def render(results, color=False, quiet=False):
    tag = {OK: "ok  ", WARN: "warn", FAIL: "FAIL", SKIP: "skip", INFO: "info"}
    col = {OK: "32", WARN: "33", FAIL: "31", SKIP: "90", INFO: "36"}
    lines, section = [], None
    for r in results:
        if quiet and r.status not in (WARN, FAIL):
            continue
        if r.section != section:
            section = r.section
            lines.append("")
            lines.append(("\033[1m%s\033[0m" if color else "%s") % section)
        t = "[%s]" % tag[r.status]
        if color:
            t = "\033[%sm%s\033[0m" % (col[r.status], t)
        detail = (": " + r.detail) if r.detail else ""
        lines.append("  %s %s%s" % (t, r.name, detail))
        if r.hint and r.status in (WARN, FAIL, INFO, SKIP):
            lines.append("         -> %s" % r.hint)
    c = summarize(results)
    lines.append("")
    lines.append("summary: %d ok, %d warning(s), %d failure(s), %d skipped" % (c[OK], c[WARN], c[FAIL], c[SKIP]))
    return "\n".join(lines)


# ---------------------------------------------------------------- machine access

def run(cmd, timeout=15, stdin=None):
    """(rc, stdout, stderr); rc is None when the command is missing or timed out."""
    if not shutil.which(cmd[0]):
        return None, "", "%s: not found" % cmd[0]
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, input=stdin)
        return p.returncode, p.stdout, p.stderr
    except subprocess.TimeoutExpired:
        return None, "", "%s: timed out after %ss" % (cmd[0], timeout)
    except OSError as e:
        return None, "", str(e)


def df_free_gb(path):
    """Free GB at path; walks up to the nearest readable parent. Returns (gb, measured_at) or (None, why)."""
    if not shutil.which("df"):
        return None, "df not found"
    p = path
    while True:
        rc, out, err = run(["df", "-Pk", p])
        if rc == 0:
            gb = parse_df_free_gb(out)
            return (gb, p) if gb is not None else (None, "could not read df output")
        parent = os.path.dirname(p.rstrip("/")) or "/"
        if parent == p or p == "/":
            return None, (err.strip() or "df failed")
        p = parent


def check_host(ctx):
    sec = "Host"
    out = []
    osname = ""
    try:
        with open("/etc/os-release") as f:
            m = re.search(r'^PRETTY_NAME="?([^"\n]+)', f.read(), re.M)
            osname = m.group(1) if m else ""
    except OSError:
        pass
    rc, kern, _ = run(["uname", "-r"])
    out.append(Result(sec, "OS", INFO if osname else SKIP,
                      "%s, kernel %s" % (osname or "unknown", kern.strip() if rc == 0 else "?"),
                      "" if osname else "no /etc/os-release"))
    if osname and not re.search(r"Ubuntu (22|24)\.04", osname):
        out[-1].hint = "images are Ubuntu 24.04 (Jazzy); other hosts work through Docker but are untested"
    for label, path in ctx["disk_paths"]:
        if path is None:
            out.append(Result(sec, "free disk, %s" % label, SKIP, "path unknown (Docker not reachable)"))
            continue
        gb, at = df_free_gb(path)
        if gb is None:
            out.append(Result(sec, "free disk, %s" % label, SKIP, at))
            continue
        st = disk_status(gb)
        where = path if at == path else "%s (measured at %s)" % (path, at)
        hint = ""
        if st != OK:
            hint = "free space (docker image ls, old build volumes); full images need tens of GB, " \
                   "warn under %d GB, fail under %d GB" % (DISK_WARN_GB, DISK_FAIL_GB)
        out.append(Result(sec, "free disk, %s" % label, st, "%.0f GB at %s" % (gb, where), hint))
    try:
        with open(ctx["env"].get("DOCTOR_MEMINFO", "/proc/meminfo")) as f:  # override is for the tests
            gb = parse_meminfo_gb(f.read())
    except OSError:
        gb = None
    if gb is None:
        out.append(Result(sec, "RAM", SKIP, "/proc/meminfo not readable"))
    else:
        st = ram_status(gb)
        out.append(Result(sec, "RAM", st, "%.0f GB" % gb,
                          "" if st == OK else "colcon and cuRobo builds want 16 GB or more; lower WORKERS"))
    return out


def check_docker(ctx):
    sec = "Docker"
    out = []
    if not shutil.which("docker"):
        ctx["docker"] = False
        out.append(Result(sec, "docker CLI", FAIL, "not installed", "install Docker Engine: https://docs.docker.com/engine/install/"))
        for n in ("daemon", "buildx", "BuildKit", "images"):
            out.append(Result(sec, n, SKIP, "docker not installed"))
        return out
    rc, ver, _ = run(["docker", "--version"])
    out.append(Result(sec, "docker CLI", OK, ver.strip() if rc == 0 else "present"))
    rc, info, err = run(["docker", "info", "--format", "{{.ServerVersion}}|{{.DockerRootDir}}|{{json .Runtimes}}"])
    if rc != 0:
        kind = classify_docker_error(err)
        ctx["docker"] = False
        if kind == "permission":
            out.append(Result(sec, "daemon access", FAIL, "permission denied on the Docker socket",
                              "sudo usermod -aG docker $USER, then log out and back in"))
        elif kind == "daemon_down":
            out.append(Result(sec, "daemon access", FAIL, "daemon not reachable",
                              "start it: sudo systemctl start docker"))
        else:
            out.append(Result(sec, "daemon access", FAIL, (err.strip().splitlines() or ["docker info failed"])[0],
                              "run `docker info` to see the full error"))
        for n in ("buildx", "BuildKit", "images"):
            out.append(Result(sec, n, SKIP, "daemon not usable"))
        return out
    ctx["docker"] = True
    parts = info.strip().split("|", 2)
    server, root = parts[0], (parts[1] if len(parts) > 1 else "")
    runtimes = parts[2] if len(parts) > 2 else ""
    ctx["docker_root"], ctx["docker_runtimes"] = root or None, runtimes
    out.append(Result(sec, "daemon access", OK, "server %s, root %s" % (server, root or "?")))
    bx_rc, bx, _ = run(["docker", "buildx", "version"])
    if bx_rc == 0:
        out.append(Result(sec, "buildx", OK, bx.strip().split(" ")[1] if len(bx.split()) > 1 else bx.strip()))
    else:
        out.append(Result(sec, "buildx", WARN, "not installed",
                          "install docker-buildx-plugin; the cache mounts in docker/Dockerfile need BuildKit"))
    sv = parse_version(server)
    if os.environ.get("DOCKER_BUILDKIT") == "0":
        out.append(Result(sec, "BuildKit", FAIL, "disabled by DOCKER_BUILDKIT=0", "unset DOCKER_BUILDKIT"))
    elif bx_rc == 0 or (sv and sv >= (23,)):
        out.append(Result(sec, "BuildKit", OK, "default builder"))
    else:
        out.append(Result(sec, "BuildKit", WARN, "engine %s predates BuildKit by default" % server,
                          "upgrade Docker to 23 or newer, or install buildx"))
    out += check_images(ctx)
    return out


def image_size(tag):
    rc, o, _ = run(["docker", "image", "inspect", "-f", "{{.Size}}", tag])
    if rc == 0 and o.strip().isdigit():
        return int(o.strip())
    return None


def check_images(ctx):
    sec = "Docker"
    out = []
    ctx["images"] = {}
    for tag, target, required in IMAGES:
        size = image_size(tag)
        ctx["images"][tag] = size
        build = "docker build -f docker/Dockerfile --target %s -t %s .   (make image-%s)" % (target, tag, target)
        if size is not None:
            out.append(Result(sec, "image %s" % tag, OK, human_size(size)))
        elif required:
            out.append(Result(sec, "image %s" % tag, WARN, "not built", build))
        else:
            out.append(Result(sec, "image %s" % tag, INFO, "not built (optional)", build))
    return out


def check_gpu(ctx):
    sec = "GPU"
    out = []
    if not shutil.which("nvidia-smi"):
        out.append(Result(sec, "nvidia-smi", WARN, "not found, GPU planners (cuRobo, VAMP-GPU paths) unavailable",
                          "install the NVIDIA driver; CPU-only work is fine without it"))
        out.append(Result(sec, "container runtime", SKIP, "no NVIDIA driver on the host"))
        if ctx["gpu_smoke"]:
            out.append(Result(sec, "docker --gpus smoke", SKIP, "no NVIDIA driver on the host"))
        return out
    q = "name,driver_version,memory.total,compute_cap"
    rc, o, _ = run(["nvidia-smi", "--query-gpu=" + q, "--format=csv,noheader"])
    if rc != 0:
        rc, o, _ = run(["nvidia-smi", "--query-gpu=name,driver_version,memory.total", "--format=csv,noheader"])
    gpus = parse_nvidia_smi(o) if rc == 0 else []
    if not gpus:
        out.append(Result(sec, "nvidia-smi", FAIL, "installed but reports no GPU",
                          "check the driver (`nvidia-smi`), a reboot often fixes a driver/kernel mismatch"))
    for g in gpus:
        out.append(Result(sec, "GPU", OK, "%s, driver %s, %s" % (g["name"], g["driver"], g["memory"])))
        if g["cc"] is None:
            out.append(Result(sec, "compute capability", SKIP, "driver too old to report it"))
        elif g["cc"] < MIN_CC_WARN:
            out.append(Result(sec, "compute capability", WARN, "%.1f" % g["cc"],
                              "recent PyTorch CUDA wheels may not include this architecture; check torch.cuda.get_arch_list()"))
        else:
            out.append(Result(sec, "compute capability", OK, "%.1f" % g["cc"]))
    if not ctx.get("docker"):
        out.append(Result(sec, "container runtime", SKIP, "Docker not usable"))
    elif "nvidia" in (ctx.get("docker_runtimes") or ""):
        out.append(Result(sec, "container runtime", OK, "nvidia runtime registered with Docker"))
    else:
        out.append(Result(sec, "container runtime", WARN, "no nvidia runtime registered",
                          "install nvidia-container-toolkit, then: sudo nvidia-ctk runtime configure --runtime=docker "
                          "&& sudo systemctl restart docker"))
    if ctx["gpu_smoke"]:
        out.append(smoke_gpu(ctx))
    return out


def smoke_gpu(ctx):
    sec, name = "GPU", "docker --gpus smoke"
    if not ctx.get("docker"):
        return Result(sec, name, SKIP, "Docker not usable")
    if ctx.get("images", {}).get("motion-planner:gpu") is None:
        return Result(sec, name, SKIP, "motion-planner:gpu not built",
                      "docker build -f docker/Dockerfile --target gpu -t motion-planner:gpu .   (make image-gpu)")
    rc, o, err = run(["docker", "run", "--rm", "--gpus", "all", "motion-planner:gpu", "nvidia-smi", "-L"], timeout=180)
    if rc == 0:
        return Result(sec, name, OK, (o.strip().splitlines() or ["ran"])[0])
    msg = (err.strip().splitlines() or ["failed"])[-1]
    return Result(sec, name, FAIL, msg, "see docs/docker.md, GPU; usually the container toolkit is not configured")


def curl_probe(url, out_path="/dev/null"):
    """(curl rc, http code, bytes) for one GET with a short timeout; rc None when curl is missing."""
    cmd = ["curl", "-sS", "-L", "--connect-timeout", "6", "--max-time", "20", "-o", out_path,
           "-w", "%{http_code} %{size_download}", url]
    if out_path == "/dev/null":
        cmd[2:2] = ["-r", "0-0"]  # one byte is enough to prove the TLS handshake and the route
    rc, o, err = run(cmd, timeout=30)
    if rc is None:
        return None, 0, 0, err
    parts = o.strip().split()
    code = int(parts[0]) if parts and parts[0].isdigit() else 0
    size = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 0
    return rc, code, size, err


def tls_issuer(host):
    if not shutil.which("openssl"):
        return None
    rc, o, _ = run(["openssl", "s_client", "-connect", "%s:443" % host, "-servername", host],
                   timeout=12, stdin="")
    if rc is None or not o.strip():
        return None
    rc2, iss, _ = run(["openssl", "x509", "-noout", "-issuer"], timeout=8, stdin=o)
    return iss.strip() if rc2 == 0 and iss.strip() else None


CA_HINT = ("give builds the CA instead of turning verification off: copy the CA PEM to a file, then in the image "
           "COPY it to /usr/local/share/ca-certificates/ and run update-ca-certificates (or mount it and set "
           "SSL_CERT_FILE, REQUESTS_CA_BUNDLE, pip config global.cert); do not use --trusted-host or curl -k")


def probe_mirror(entry):
    host, url, suite = entry
    with tempfile.NamedTemporaryFile(delete=False) as tf:
        path = tf.name
    try:
        rc, code, size, err = curl_probe(url, path)
        digest = None
        if rc == 0 and code == 200:
            with open(path, "rb") as f:
                digest = hashlib.sha256(f.read()).hexdigest()
        return host, suite, rc, code, size, err, digest
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass


def probe_tls(entry):
    label, host, url = entry
    rc, code, size, err = curl_probe(url)
    iss = tls_issuer(host) if rc is not None else None
    return label, host, rc, code, err, iss


def check_network(ctx):
    sec = "Network"
    if ctx["no_network"]:
        return [Result(sec, "network checks", SKIP, "--no-network given")]
    if not shutil.which("curl"):
        return [Result(sec, "network checks", SKIP, "curl not installed")]
    out = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as ex:
        mirrors = list(ex.map(probe_mirror, UBUNTU_MIRRORS))
        tls = list(ex.map(probe_tls, TLS_HOSTS))
    fetched = {}
    for host, suite, rc, code, size, err, digest in mirrors:
        name = "Ubuntu mirror %s" % host
        if rc != 0:
            out.append(Result(sec, name, WARN, "unreachable (%s)" % (err.strip().splitlines() or ["curl failed"])[-1],
                              "apt inside the build needs it; check DNS, proxy and firewall, or pick another mirror"))
            continue
        st, detail = classify_inrelease(code, size)
        if st == OK and digest:
            fetched[host] = (suite, digest)
        out.append(Result(sec, name, st, "InRelease " + detail,
                          "" if st == OK else "a proxy or captive portal may be answering instead of the mirror"))
    for suite, hosts in compare_mirrors(fetched):
        out.append(Result(sec, "Ubuntu mirrors in sync", WARN, "%s InRelease differs between %s" % (suite, ", ".join(hosts)),
                          "apt shows 'Hash Sum mismatch' while a mirror syncs; retry later or build against one mirror"))
    for label, host, rc, code, err, iss in tls:
        name = "%s (%s)" % (label, host)
        kind, short = classify_issuer(iss) if iss else ("unknown", "")
        if rc == 60 or rc == 35:
            msg = "TLS verification failed"
            if kind == "interception":
                msg += ", certificate issued by %s (TLS inspection)" % short
            elif short:
                msg += ", issuer %s" % short
            out.append(Result(sec, name, FAIL, msg, CA_HINT))
        elif rc != 0:
            out.append(Result(sec, name, WARN, "unreachable (%s)" % (err.strip().splitlines() or ["curl failed"])[-1],
                              "builds and pip installs need it; check DNS, proxy and firewall"))
        elif code >= 400:
            out.append(Result(sec, name, WARN, "HTTP %d" % code, "retry; if it persists a proxy may be filtering it"))
        elif kind == "interception":
            out.append(Result(sec, name, WARN, "reachable, but the certificate comes from %s (TLS inspection)" % short,
                              "the host trusts it, containers do not: " + CA_HINT))
        else:
            out.append(Result(sec, name, OK, "reachable" + (", issuer %s" % short if short else "")))
    return out


def check_ros(ctx):
    return check_ros_env(ctx["env"])


def check_sims(ctx):
    sec = "Simulators"
    out = []
    root = ctx["env"].get("MUJOCO_ROOT") or ("/opt/mujoco" if os.path.isdir("/opt/mujoco") else "")
    if not root:
        out.append(Result(sec, "MuJoCo SDK", SKIP, "MUJOCO_ROOT unset and /opt/mujoco absent",
                          "only needed for the sim_mujoco plugin, see docs/sim_mujoco.md (Build)"))
    else:
        hdr = os.path.join(root, "include", "mujoco", "mujoco.h")
        if not os.path.isfile(hdr):
            out.append(Result(sec, "MuJoCo SDK", WARN, "%s has no include/mujoco/mujoco.h" % root,
                              "point MUJOCO_ROOT at the unpacked SDK"))
        else:
            try:
                with open(hdr, errors="replace") as f:
                    ver = parse_mujoco_version(f.read(200000))
            except OSError:
                ver = None
            out.append(Result(sec, "MuJoCo SDK", OK, "%s, version %s" % (root, ver or "unknown")))
    if not ctx.get("docker"):
        out.append(Result(sec, "Gazebo image", SKIP, "Docker not usable"))
    elif ctx.get("images", {}).get("motion-planner:sim-gazebo") is not None:
        out.append(Result(sec, "Gazebo image", OK, "motion-planner:sim-gazebo present"))
    else:
        out.append(Result(sec, "Gazebo image", INFO, "motion-planner:sim-gazebo not built (optional)",
                          "docker build -f docker/Dockerfile --target sim-gazebo -t motion-planner:sim-gazebo .   (make image-sim-gazebo)"))
    return out


def check_repo(ctx):
    sec = "Repository"
    root = ctx["root"]
    out = []
    if not shutil.which("git"):
        out.append(Result(sec, "git", WARN, "not installed", "install git; the tools use it to find tracked files"))
    else:
        out.append(Result(sec, "git", OK, run(["git", "--version"])[1].strip()))
        rc, o, _ = run(["git", "-C", root, "submodule", "status"])
        if rc != 0:
            out.append(Result(sec, "submodules", SKIP, "not a git checkout"))
        elif o.strip():
            out.append(Result(sec, "submodules", WARN, "%d found, none expected" % len(o.strip().splitlines()),
                              "git submodule deinit --all, or ask why one was added"))
        else:
            out.append(Result(sec, "submodules", OK, "none"))
    pdir = os.path.join(root, "src/description/mobile_manipulator_description/config/profiles")
    files = sorted(f for f in os.listdir(pdir) if f.endswith(".yaml")) if os.path.isdir(pdir) else []
    if not files:
        out.append(Result(sec, "robot profiles", SKIP, "profiles directory not found under %s" % root))
    else:
        try:
            import yaml
        except ImportError:
            out.append(Result(sec, "robot profiles", SKIP, "%d files, PyYAML not installed" % len(files),
                              "python3 -m pip install pyyaml"))
            yaml = None
        if yaml:
            bad = []
            for f in files:
                try:
                    with open(os.path.join(pdir, f)) as fh:
                        yaml.safe_load(fh)
                except Exception as e:  # noqa: BLE001
                    bad.append("%s (%s)" % (f, str(e).splitlines()[0]))
            if bad:
                out.append(Result(sec, "robot profiles", FAIL, "; ".join(bad), "fix the YAML syntax"))
            else:
                out.append(Result(sec, "robot profiles", OK, "%d load" % len(files)))
    if ctx["repo_scan"]:
        script = os.path.join(root, "docker/tools/check_public_readiness.py")
        if not os.path.isfile(script):
            out.append(Result(sec, "public-readiness scan", SKIP, "script not found"))
        else:
            rc, o, err = run([sys.executable, script, "--root", root], timeout=120)
            last = (o.strip().splitlines() or [err.strip()])[-1]
            out.append(Result(sec, "public-readiness scan", OK if rc == 0 else FAIL, last,
                              "" if rc == 0 else "run python3 docker/tools/check_public_readiness.py for the list"))
    else:
        out.append(Result(sec, "public-readiness scan", SKIP, "pass --repo to run it"))
    return out


def check_extras(ctx):
    sec = "Optional extras"
    if not ctx.get("docker"):
        return [Result(sec, "Isaac Sim image", SKIP, "Docker not usable")]
    rc, o, _ = run(["docker", "images", "--format", "{{.Repository}}:{{.Tag}} {{.Size}}", ISAAC_IMAGE])
    if rc == 0 and o.strip():
        return [Result(sec, "Isaac Sim image", INFO, o.strip().splitlines()[0])]
    return [Result(sec, "Isaac Sim image", INFO, "not present (never required)",
                   "only for sim_isaac, see docs/sim_isaac.md")]


def run_all(ctx):
    results = []
    # Docker first: it tells the host check where the Docker root is.
    docker = check_docker(ctx)
    ctx["disk_paths"] = [("Docker root", ctx.get("docker_root")), ("repo", ctx["root"])]
    results += check_host(ctx) + docker + check_gpu(ctx) + check_network(ctx) + check_ros(ctx) \
        + check_sims(ctx) + check_repo(ctx) + check_extras(ctx)
    return results


def main(argv=None):
    ap = argparse.ArgumentParser(description="Check this machine for what the stack needs.")
    ap.add_argument("--quiet", action="store_true", help="print only warnings, failures and the summary")
    ap.add_argument("--json", action="store_true", help="print machine-readable JSON only")
    ap.add_argument("--gpu", action="store_true", help="also run a docker --gpus smoke test (needs motion-planner:gpu)")
    ap.add_argument("--repo", action="store_true", help="also run the public-readiness scan")
    ap.add_argument("--no-network", action="store_true", help="skip the network checks")
    ap.add_argument("--root", default=os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
    args = ap.parse_args(argv)
    ctx = {"env": dict(os.environ), "root": args.root, "gpu_smoke": args.gpu, "repo_scan": args.repo,
           "no_network": args.no_network}
    results = run_all(ctx)
    if args.json:
        print(json.dumps(build_json(results), indent=2))
    else:
        color = sys.stdout.isatty() and "NO_COLOR" not in os.environ
        print(render(results, color=color, quiet=args.quiet))
    return exit_code(results)


if __name__ == "__main__":
    sys.exit(main())
