import json
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(__file__)
TOOLS = os.path.join(HERE, "..")
sys.path.insert(0, TOOLS)
import doctor_lib as d  # noqa: E402


# ---- thresholds

@pytest.mark.parametrize("gb,want", [(100, d.OK), (40, d.OK), (39.9, d.WARN), (15, d.WARN), (14.9, d.FAIL), (0, d.FAIL)])
def test_disk_status(gb, want):
    assert d.disk_status(gb) == want


@pytest.mark.parametrize("gb,want", [(64, d.OK), (16, d.OK), (15.9, d.WARN), (8, d.WARN), (7.9, d.FAIL)])
def test_ram_status(gb, want):
    assert d.ram_status(gb) == want


# ---- parsers

def test_parse_version():
    assert d.parse_version("Docker version 27.3.1, build abc") == (27, 3, 1)
    assert d.parse_version("v0.17.1-desktop.1") == (0, 17, 1)
    assert d.parse_version("24") == (24,)
    assert d.parse_version("none") is None
    assert d.parse_version(None) is None


def test_parse_mujoco_version():
    assert d.parse_mujoco_version("#define mjVERSION_HEADER 331\n") == "3.3.1"
    assert d.parse_mujoco_version("#  define mjVERSION_HEADER 400") == "4.0.0"
    assert d.parse_mujoco_version("nothing here") is None


def test_parse_df():
    out = "Filesystem 1024-blocks Used Available Capacity Mounted on\n/dev/sda1 100 50 %d 50%% /\n" % (30 * 1024 * 1024)
    assert d.parse_df_free_gb(out) == pytest.approx(30.0)
    assert d.parse_df_free_gb("garbage") is None
    assert d.parse_df_free_gb("h\n/dev/x 1 2 abc 4 /") is None


def test_parse_meminfo():
    assert d.parse_meminfo_gb("MemTotal:       16777216 kB\nMemFree: 1 kB") == pytest.approx(16.0)
    assert d.parse_meminfo_gb("") is None


def test_parse_nvidia_smi():
    g = d.parse_nvidia_smi("NVIDIA GeForce RTX 3050, 550.54, 4096 MiB, 8.6\n")
    assert g == [{"name": "NVIDIA GeForce RTX 3050", "driver": "550.54", "memory": "4096 MiB", "cc": 8.6}]
    assert d.parse_nvidia_smi("Tesla K80, 470.1, 11441 MiB")[0]["cc"] is None
    assert d.parse_nvidia_smi("") == []


def test_human_size():
    assert d.human_size(512) == "512 B"
    assert d.human_size(5_200_000_000) == "5.2 GB"


# ---- classifiers

@pytest.mark.parametrize("issuer,kind", [
    ("issuer=C = US, O = Let's Encrypt, CN = R10", "public"),
    ("issuer=C = US, O = DigiCert Inc, CN = DigiCert Global G2 TLS RSA SHA256 2020 CA1", "public"),
    ("issuer=C = GB, O = Sectigo Limited, CN = Sectigo RSA Domain Validation Secure Server CA", "public"),
    ("issuer=C = US, O = Fortinet, CN = FortiGate CA", "interception"),
    ("issuer=O = Sophos, OU = NSG, CN = Sophos SSL CA_abc", "interception"),
    ("issuer=O = Zscaler Inc., CN = Zscaler Intermediate Root CA", "interception"),
    ("issuer=O = Acme Labs, CN = Acme Root", "unknown"),
    ("", "unknown"),
])
def test_classify_issuer(issuer, kind):
    assert d.classify_issuer(issuer)[0] == kind


def test_classify_issuer_short_name():
    assert d.classify_issuer("issuer=C = GB, O = Sophos, CN = x")[1] == "Sophos"
    assert d.classify_issuer("issuer= /C=US/O=Fortinet/CN=FG")[1] == "Fortinet"


def test_classify_docker_error():
    assert d.classify_docker_error("permission denied while trying to connect to the Docker daemon socket") == "permission"
    assert d.classify_docker_error("Cannot connect to the Docker daemon at unix:///var/run/docker.sock") == "daemon_down"
    assert d.classify_docker_error("weird") == "other"


def test_classify_inrelease():
    assert d.classify_inrelease(200, 250_000)[0] == d.OK
    assert d.classify_inrelease(200, 900)[0] == d.WARN
    assert d.classify_inrelease(503, 0)[0] == d.FAIL


def test_compare_mirrors():
    same = {"a": ("noble", "h1"), "b": ("noble", "h1"), "s": ("noble-security", "h9")}
    assert d.compare_mirrors(same) == []
    diff = {"a": ("noble", "h1"), "b": ("noble", "h2"), "s": ("noble-security", "h9")}
    assert d.compare_mirrors(diff) == [("noble", ["a", "b"])]


# ---- ROS environment

def by_name(results):
    return {r.name: r for r in results}


def test_ros_unset_is_skipped():
    assert [x.status for x in d.check_ros_env({})] == [d.SKIP]


def test_ros_env_cases(tmp_path):
    r = by_name(d.check_ros_env({"ROS_DISTRO": "jazzy", "ROS_DOMAIN_ID": "51"}))
    assert r["ROS_DISTRO"].status == d.OK and r["ROS_DOMAIN_ID"].status == d.OK
    assert by_name(d.check_ros_env({"ROS_DISTRO": "jazzy", "ROS_DOMAIN_ID": "300"}))["ROS_DOMAIN_ID"].status == d.FAIL
    assert by_name(d.check_ros_env({"ROS_DISTRO": "jazzy", "ROS_DOMAIN_ID": "x"}))["ROS_DOMAIN_ID"].status == d.FAIL
    assert by_name(d.check_ros_env({"ROS_DISTRO": "foxy"}))["ROS_DISTRO"].status == d.WARN
    lo = by_name(d.check_ros_env({"ROS_DISTRO": "jazzy", "ROS_LOCALHOST_ONLY": "1"}))["ROS_LOCALHOST_ONLY"]
    assert lo.status == d.WARN and "ROS_AUTOMATIC_DISCOVERY_RANGE" in lo.hint
    miss = by_name(d.check_ros_env({"ROS_DISTRO": "jazzy", "FASTRTPS_DEFAULT_PROFILES_FILE": str(tmp_path / "no.xml")}))
    assert miss["FASTRTPS_DEFAULT_PROFILES_FILE"].status == d.WARN
    f = tmp_path / "p.xml"
    f.write_text("<x/>")
    env = {"ROS_DISTRO": "jazzy", "FASTRTPS_DEFAULT_PROFILES_FILE": str(f), "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"}
    assert by_name(d.check_ros_env(env))["DDS profile vs RMW"].status == d.WARN


# ---- summary and JSON

def sample():
    return [d.Result("A", "one", d.OK, "fine"), d.Result("A", "two", d.WARN, "meh", "do x"),
            d.Result("B", "three", d.FAIL, "bad", "do y"), d.Result("B", "four", d.SKIP, "n/a")]


def test_summary_and_exit_code():
    assert d.summarize(sample()) == {"ok": 1, "warn": 1, "fail": 1, "skip": 1, "info": 0}
    assert d.exit_code(sample()) == 1
    assert d.exit_code([r for r in sample() if r.status != d.FAIL]) == 0


def test_build_json():
    j = d.build_json(sample())
    assert j["exit_code"] == 1 and j["summary"]["warn"] == 1
    assert j["checks"][2] == {"section": "B", "name": "three", "status": "fail", "detail": "bad", "hint": "do y"}
    json.dumps(j)


def test_render_quiet_and_color():
    text = d.render(sample(), quiet=True)
    assert "one" not in text and "two" in text and "-> do y" in text
    assert "summary: 1 ok, 1 warning(s), 1 failure(s), 1 skipped" in text
    assert "\033[" not in text
    assert "\033[" in d.render(sample(), color=True)


# ---- end to end with stubbed tools on PATH

DF = '#!/bin/sh\necho "Filesystem 1024-blocks Used Available Capacity Mounted on"\necho "/dev/x 1 1 ${STUB_DF_KB:-104857600} 1% /"\n'
DOCKER = r'''#!/bin/sh
case "$1" in
  --version) echo "Docker version 27.3.1, build abc" ;;
  info) [ -n "$STUB_DOCKER_DENIED" ] && { echo "permission denied while trying to connect" >&2; exit 1; }
        echo '27.3.1|/var/lib/docker|{"runc":{},"nvidia":{}}' ;;
  buildx) echo "github.com/docker/buildx v0.17.1 abc" ;;
  image) echo 1000000000 ;;
  images) echo "nvcr.io/nvidia/isaac-sim:5.1.0 15GB" ;;
  run) echo "GPU 0: Fake GPU" ;;
esac
'''
SMI = '#!/bin/sh\necho "Fake RTX 3050, 550.54, 4096 MiB, 8.6"\n'
CURL = r'''#!/bin/sh
out=/dev/null
while [ $# -gt 0 ]; do
  if [ "$1" = "-o" ]; then out="$2"; shift; fi
  shift
done
[ "$out" != /dev/null ] && printf 'same-body' > "$out"
if [ "${STUB_CURL_RC:-0}" != 0 ]; then echo "000 0"; echo "curl: fake failure" >&2; exit "$STUB_CURL_RC"; fi
echo "200 60000"
'''
OPENSSL = r'''#!/bin/sh
case "$1" in
  s_client) echo "-----BEGIN CERTIFICATE-----" ;;
  x509) echo "issuer=${STUB_ISSUER:-O = DigiCert Inc, CN = Some CA}" ;;
esac
'''
GIT = '#!/bin/sh\n[ "$1" = "--version" ] && echo "git version 2.43.0"\nexit 0\n'
FULL = {"git": GIT, "df": DF, "docker": DOCKER, "nvidia-smi": SMI, "curl": CURL, "openssl": OPENSSL}


def run_doctor(tmp_path, tools, extra_env=None, args=()):
    bindir = tmp_path / "bin"
    bindir.mkdir(exist_ok=True)
    for name, body in tools.items():
        p = bindir / name
        p.write_text(body)
        p.chmod(0o755)
    mem = tmp_path / "meminfo"
    mem.write_text("MemTotal:       33554432 kB\n")
    root = tmp_path / "root"
    root.mkdir(exist_ok=True)
    env = {"PATH": str(bindir), "HOME": str(tmp_path), "DOCTOR_MEMINFO": str(mem)}
    env.update(extra_env or {})
    p = subprocess.run([sys.executable, os.path.join(TOOLS, "doctor_lib.py"), "--root", str(root), *args],
                       env=env, capture_output=True, text=True, timeout=120)
    return p.returncode, p.stdout


def test_e2e_everything_ok(tmp_path):
    rc, out = run_doctor(tmp_path, FULL)
    assert rc == 0, out
    assert "0 failure(s)" in out and "[FAIL]" not in out, out
    assert "[warn]" not in out, out


def test_e2e_json_output(tmp_path):
    rc, out = run_doctor(tmp_path, FULL, args=("--json",))
    j = json.loads(out)
    assert rc == 0 and j["exit_code"] == 0 and j["summary"]["fail"] == 0
    assert any(c["name"] == "buildx" and c["status"] == "ok" for c in j["checks"])


def test_e2e_gpu_smoke_flag(tmp_path):
    rc, out = run_doctor(tmp_path, FULL, args=("--gpu",))
    assert rc == 0 and "docker --gpus smoke: GPU 0: Fake GPU" in out, out


def test_e2e_docker_missing(tmp_path):
    rc, out = run_doctor(tmp_path, {k: v for k, v in FULL.items() if k != "docker"})
    assert rc == 1
    assert "[FAIL] docker CLI: not installed" in out and "install Docker Engine" in out
    assert "Traceback" not in out


def test_e2e_disk_low_fails_and_mid_warns(tmp_path):
    rc, out = run_doctor(tmp_path, FULL, {"STUB_DF_KB": str(10 * 1024 * 1024)})
    assert rc == 1 and "[FAIL] free disk, repo: 10 GB" in out, out
    rc, out = run_doctor(tmp_path, FULL, {"STUB_DF_KB": str(30 * 1024 * 1024)})
    assert rc == 0 and "[warn] free disk, repo: 30 GB" in out, out


def test_e2e_tls_inspection_trusted_by_host_warns(tmp_path):
    rc, out = run_doctor(tmp_path, FULL, {"STUB_ISSUER": "C = US, O = Fortinet, CN = FortiGate CA"})
    assert rc == 0
    assert "TLS inspection" in out and "containers do not" in out and "update-ca-certificates" in out, out


def test_e2e_tls_inspection_untrusted_fails(tmp_path):
    rc, out = run_doctor(tmp_path, FULL, {"STUB_ISSUER": "O = Fortinet, CN = FortiGate CA", "STUB_CURL_RC": "60"})
    assert rc == 1
    assert "[FAIL] GitHub (github.com): TLS verification failed, certificate issued by Fortinet" in out, out
    assert "do not use --trusted-host" in out


def test_e2e_gpu_missing_is_a_warning(tmp_path):
    rc, out = run_doctor(tmp_path, {k: v for k, v in FULL.items() if k != "nvidia-smi"}, args=("--gpu",))
    assert rc == 0, out
    assert "[warn] nvidia-smi: not found" in out
    assert "docker --gpus smoke: no NVIDIA driver" in out


def test_e2e_docker_permission_denied(tmp_path):
    rc, out = run_doctor(tmp_path, FULL, {"STUB_DOCKER_DENIED": "1"})
    assert rc == 1 and "permission denied on the Docker socket" in out and "usermod -aG docker" in out, out


def test_shim_runs():
    p = subprocess.run([os.path.join(TOOLS, "doctor.sh"), "--help"], capture_output=True, text=True, timeout=30)
    assert p.returncode == 0 and "--gpu" in p.stdout
