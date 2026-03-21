'''
按照yaml创建、管理docker容器。需要以root权限运行
loguru作为日志系统

容器管理
手动启动的命令如下
``` bash
docker run -itd --rm --privileged --shm-size=2g \
  -v ~/data/android_data_11:/data \
  -p 5555:5555 \
  erstt/redroid:11.0.0_houdini_ChromeOS \
  ro.enable.native.bridge.exec64=1 \
  androidboot.redroid_gpu_mode=guest \
  androidboot.redroid_width=720 / height=1080\
  ro.dalvik.vm.native.bridge=libhoudini.so
```

创建，启动，移除容器，为不同container管理data挂载点以data快照。提供python函数接口和命令行接口。配置应该由yaml文件提供，格式参考device_config.yaml

默认挂载点/data/redroid/android_data，每个设备按id创建一个子目录作为挂载点，设备移除后释放,若设备初始化时存在目录，首先将其移除

默认data镜像存放点/data/redroid/images，镜像被打包成xxx.tar.gz或.sqsh文件

快照手动创建，此程序只需要管理加载，在使用指北中提供一个创建镜像用的linux命令
若在创建容器时指定快照，读取并加载镜像（解压到data挂载点）

管理container 的adb转发端口，端口 = id + 5555。镜像内安卓系统启动后立即重连adb disconnect -> connect

TODO 阶段2 容器内服务

设备启动后提供设备的：
- MNTDevice对象
- 设备按键控制接口（基于adb，支持返回，home，菜单，音量）
- 截屏接口（基于adb，返回PIL Image）
- 状态接口 mCurrentFocus和mFocusedApp
- 应用接口
    - 列出已安装应用，包含包名和应用名（可选是否包含系统和用户应用）
    - 启动应用
'''

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import tarfile
import time
from dataclasses import dataclass
from typing import Iterable

import yaml
from loguru import logger

import sys
import shutil as _shutil

DATA_ROOT_BASE = "/data/redroid"
DATA_MOUNT_BASE = os.path.join(DATA_ROOT_BASE, "android_data")
IMAGE_BASE = os.path.join(DATA_ROOT_BASE, "images")

ADB_KEYBOARD_APK_NAME = "ADBKeyBoard.apk"
ADB_KEYBOARD_URL = "https://github.com/senzhk/ADBKeyBoard/releases/download/v2.4-dev/keyboardservice-debug.apk"
ADB_KEYBOARD_PKG = "com.android.adbkeyboard"
ADB_KEYBOARD_IME = "com.android.adbkeyboard/.AdbIME"

DEFAULT_DOCKER_SHM_SIZE = "2g"
DEFAULT_GPU_MODE = "guest"
DEFAULT_ADB_BASE_PORT = 5555
DEFAULT_CONTAINER_NAME_PREFIX = "redroid"
DEFAULT_ARM_COMPATIBILITY = "libhoudini"

ARM_COMPATIBILITY_CHOICES = {"libhoudini", "libndk", "none"}

ARM_COMPATIBILITY_ANDROID_ARGS = {
  "libhoudini": [
    "ro.enable.native.bridge.exec64=1",
    "ro.dalvik.vm.native.bridge=libhoudini.so",
  ],
  "libndk": [
    "ro.product.cpu.abilist=x86_64,arm64-v8a,x86,armeabi-v7a,armeabi",
    "ro.product.cpu.abilist64=x86_64,arm64-v8a",
    "ro.product.cpu.abilist32=x86,armeabi-v7a,armeabi",
    "ro.dalvik.vm.isa.arm=x86",
    "ro.dalvik.vm.isa.arm64=x86_64",
    "ro.enable.native.bridge.exec=1",
    "ro.vendor.enable.native.bridge.exec=1",
    "ro.vendor.enable.native.bridge.exec64=1",
    "ro.dalvik.vm.native.bridge=libndk_translation.so",
    "ro.ndk_translation.version=0.2.3",
  ],
  "none": [],
}


@dataclass(frozen=True)
class DeviceConfig:
  device_id: int
  container_img: str
  screen_resolution: str
  data_snapshot: str | None = None
  readonly_mount: bool = False
  arm_compatibility: str = DEFAULT_ARM_COMPATIBILITY
  no_destory: bool = False

  @property
  def adb_port(self) -> int:
    return DEFAULT_ADB_BASE_PORT + self.device_id

  @property
  def container_name(self) -> str:
    return f"{DEFAULT_CONTAINER_NAME_PREFIX}_{self.device_id}"

  @property
  def mount_dir(self) -> str:
    return os.path.join(DATA_MOUNT_BASE, str(self.device_id))


def _run_cmd(cmd: list[str], check: bool = True) -> subprocess.CompletedProcess:
  logger.debug("Running command: {}", " ".join(cmd))
  res = subprocess.run(cmd, check=False, text=True, capture_output=True)
  if res.returncode != 0 and check:
    stderr = (res.stderr or "").strip()
    stdout = (res.stdout or "").strip()
    logger.error(
      "Command failed (exit={}): {}\nstdout: {}\nstderr: {}",
      res.returncode,
      " ".join(cmd),
      stdout,
      stderr,
    )
    raise RuntimeError(
      f"Command failed (exit={res.returncode}): {' '.join(cmd)}\n"
      f"stdout: {stdout}\n"
      f"stderr: {stderr}"
    )
  return res


def _ensure_storage_layout() -> None:
  """Ensure storage layout exists under /data/redroid."""
  os.makedirs(DATA_MOUNT_BASE, exist_ok=True)
  os.makedirs(IMAGE_BASE, exist_ok=True)


def _safe_extract_tar(tar_path: str, dest_dir: str) -> None:
  with tarfile.open(tar_path, "r:gz") as tar:
    for member in tar.getmembers():
      member_path = os.path.join(dest_dir, member.name)
      if not os.path.realpath(member_path).startswith(os.path.realpath(dest_dir)):
        raise ValueError(f"Unsafe path in tar: {member.name}")
    tar.extractall(dest_dir)


def load_config(config_path: str) -> list[DeviceConfig]:
  with open(config_path, "r", encoding="utf-8") as f:
    raw = yaml.safe_load(f) or []

  devices: list[DeviceConfig] = []
  for item in raw:
    device = item.get("device", {})
    device_id = int(device["id"])
    container_img = device["container_img"]
    screen_resolution = device["screen_resolution"]
    data_snapshot = device.get("data_snapshot")
    readonly_mount = device.get("readonly_mount", False)
    # keep compatibility with requested key name `no_destory`
    no_destory = bool(device.get("no_destory", device.get("no_destroy", False)))
    arm_compatibility = str(device.get("arm_compatibility", DEFAULT_ARM_COMPATIBILITY)).strip().lower()
    if arm_compatibility not in ARM_COMPATIBILITY_CHOICES:
      raise ValueError(
        f"Invalid arm_compatibility for device {device_id}: {arm_compatibility}. "
        f"Supported values: {sorted(ARM_COMPATIBILITY_CHOICES)}"
      )
    devices.append(
      DeviceConfig(
        device_id=device_id,
        container_img=container_img,
        screen_resolution=screen_resolution,
        data_snapshot=data_snapshot,
        readonly_mount=readonly_mount,
        arm_compatibility=arm_compatibility,
        no_destory=no_destory,
      )
    )
  return devices


def _parse_resolution(resolution: str) -> tuple[int, int]:
  if "x" not in resolution:
    raise ValueError(f"Invalid screen_resolution: {resolution}")
  width_str, height_str = resolution.lower().split("x", 1)
  return int(width_str), int(height_str)


def _ensure_clean_mount_dir(mount_dir: str) -> None:
  if os.path.isdir(mount_dir):
    # Check if directory is mounted (for readonly squashfs)
    if _is_mounted(mount_dir):
      _unmount(mount_dir)
    logger.info("Removing existing mount dir: {}", mount_dir)
    shutil.rmtree(mount_dir)
  os.makedirs(mount_dir, exist_ok=True)


def _load_snapshot_if_needed(mount_dir: str, snapshot: str | None, readonly: bool = False) -> None:
  if not snapshot:
    return
  snapshot_path = os.path.join(IMAGE_BASE, snapshot)
  if not os.path.isfile(snapshot_path):
    raise FileNotFoundError(f"Snapshot not found: {snapshot_path}")
  logger.info("Loading snapshot: {}", snapshot_path)
  
  # Support readonly mount for squashfs images
  lower = snapshot.lower()
  if readonly and lower.endswith(".sqsh"):
    squashfuse = shutil.which("squashfuse")
    if not squashfuse:
      raise RuntimeError("squashfuse not found. Install squashfuse for readonly mounting")
    
    # Ensure mount directory exists and is empty
    if os.path.isdir(mount_dir) and os.listdir(mount_dir):
      logger.info("Mount dir not empty, removing: {}", mount_dir)
      shutil.rmtree(mount_dir)
    os.makedirs(mount_dir, exist_ok=True)
    
    # Mount squashfs image read-only
    logger.info("Mounting squashfs image read-only: {} -> {}", snapshot_path, mount_dir)
    res = _run_cmd([squashfuse, snapshot_path, mount_dir, "-o", "ro"], check=False)
    if res.returncode != 0:
      logger.error("squashfuse failed: {}", res.stderr)
      raise RuntimeError(f"squashfuse failed: {res.stderr}")
    return
  
  # Support squashfs images (.sqsh) as preferred snapshot format
  lower = snapshot.lower()
  if lower.endswith(".sqsh"):
    unsquashfs = shutil.which("unsquashfs")
    if not unsquashfs:
      raise RuntimeError("unsquashfs not found. Install squashfs-tools (unsquashfs)")
    # unsquashfs -f -d <dest> <image>
    logger.info("Using unsquashfs to extract {} -> {}", snapshot_path, mount_dir)
    # ensure dest exists and is empty
    if os.path.isdir(mount_dir) and os.listdir(mount_dir):
      logger.info("Mount dir not empty, removing: {}", mount_dir)
      shutil.rmtree(mount_dir)
    os.makedirs(mount_dir, exist_ok=True)
    res = _run_cmd([unsquashfs, "-f", "-d", mount_dir, snapshot_path], check=False)
    if res.returncode != 0:
      logger.error("unsquashfs failed: {}", res.stderr)
      raise RuntimeError(f"unsquashfs failed: {res.stderr}")
    return

  # Fallback: assume gz tarball. Ensure it's a safe tar archive (no absolute symlinks)
  if lower.endswith(".tar.gz") or lower.endswith(".tgz") or lower.endswith(".tar"):
    _safe_extract_tar(snapshot_path, mount_dir)
    return

  raise RuntimeError(f"Unsupported snapshot format: {snapshot}")


def _container_exists(container_name: str) -> bool:
  result = _run_cmd(
    ["docker", "ps", "-a", "--filter", f"name=^{container_name}$", "-q"],
    check=False,
  )
  return bool(result.stdout.strip())


def _remove_container_if_exists(container_name: str) -> None:
  if _container_exists(container_name):
    logger.info("Removing existing container: {}", container_name)
    _run_cmd(["docker", "rm", "-f", container_name], check=False)


def _adb_reconnect(adb_port: int, retries: int = 5, delay: float = 2.0) -> None:
  target = f"127.0.0.1:{adb_port}"
  _run_cmd(["adb", "disconnect", target], check=False)
  for attempt in range(1, retries + 1):
    result = _run_cmd(["adb", "connect", target], check=False)
    if "connected" in (result.stdout or "").lower():
      logger.info("ADB connected: {}", target)
      return
    logger.warning("ADB connect attempt {}/{} failed", attempt, retries)
    time.sleep(delay)
  raise RuntimeError(f"ADB connect failed: {target}")


def _adb_wait_for_boot(adb_port: int, timeout: int = 120, poll_interval: float = 2.0) -> bool:
  target = f"127.0.0.1:{adb_port}"
  logger.info("Waiting for device boot completion: {}", target)
  start = time.time()

  # First, wait for device to be online
  _run_cmd(["adb", "-s", target, "wait-for-device"], check=False)

  while time.time() - start < timeout:
    res = _run_cmd(["adb", "-s", target, "shell", "getprop", "sys.boot_completed"], check=False)
    if (res.stdout or "").strip() == "1":
      logger.info("Device boot completed: {}", target)
      return True
    time.sleep(poll_interval)

  logger.warning("Boot not completed within timeout for {}", target)
  return False


def _download_file(url: str, dest_path: str) -> None:
  logger.info("Downloading {} to {}", url, dest_path)
  # Use curl to download
  res = _run_cmd(["curl", "-L", "-o", dest_path, url], check=False)
  if res.returncode != 0:
    raise RuntimeError(f"Failed to download {url}: {res.stderr}")


def _ensure_adb_keyboard_apk() -> str:
  apk_path = os.path.join(IMAGE_BASE, ADB_KEYBOARD_APK_NAME)
  if not os.path.exists(apk_path):
    # check if IMAGE_BASE exists
    os.makedirs(IMAGE_BASE, exist_ok=True)
    _download_file(ADB_KEYBOARD_URL, apk_path)
  return apk_path


def _setup_adb_keyboard(adb_port: int) -> None:
  # Check if installed
  target = f"127.0.0.1:{adb_port}"
  # Ensure device is online and booted before installing IME
  _adb_wait_for_boot(adb_port)
  res = _run_cmd(["adb", "-s", target, "shell", "pm", "list", "packages", ADB_KEYBOARD_PKG], check=False)
  if ADB_KEYBOARD_PKG not in (res.stdout or ""):
    logger.info("Installing ADB Keyboard to {}", target)
    apk_path = _ensure_adb_keyboard_apk()
    install_res = _run_cmd(["adb", "-s", target, "install", "-r", apk_path], check=False)
    if install_res.returncode != 0:
      logger.error("Failed to install ADB Keyboard: {}", install_res.stderr)
      # Proceed anyway, might fail later but we don't block
  
  # Set as default IME
  logger.info("Setting ADB Keyboard as default IME for {}", target)
  _run_cmd(["adb", "-s", target, "shell", "ime", "enable", ADB_KEYBOARD_IME], check=False)
  _run_cmd(["adb", "-s", target, "shell", "ime", "set", ADB_KEYBOARD_IME], check=False)


def create_device(device: DeviceConfig) -> None:
  _ensure_storage_layout()
  logger.info("Creating device: {}", device.device_id)
  _remove_container_if_exists(device.container_name)
  _ensure_clean_mount_dir(device.mount_dir)
  _load_snapshot_if_needed(device.mount_dir, device.data_snapshot, device.readonly_mount)

  width, height = _parse_resolution(device.screen_resolution)
  android_args = ARM_COMPATIBILITY_ANDROID_ARGS[device.arm_compatibility] + [
    f"androidboot.redroid_gpu_mode={DEFAULT_GPU_MODE}",
    f"androidboot.redroid_width={width}",
    f"androidboot.redroid_height={height}",
  ]

  cmd = [
    "docker",
    "run",
    "-d",
    "--privileged",
    "--shm-size",
    DEFAULT_DOCKER_SHM_SIZE,
    "--name",
    device.container_name,
    "-v",
    f"{device.mount_dir}:/data",
    "-p",
    f"{device.adb_port}:5555",
    device.container_img,
  ] + android_args

  if not device.no_destory:
    cmd.insert(3, "--rm")

  _run_cmd(cmd)
  _adb_reconnect(device.adb_port)
  _adb_wait_for_boot(device.adb_port)
  _setup_adb_keyboard(device.adb_port)


def start_device(device: DeviceConfig) -> None:
  _ensure_storage_layout()
  logger.info("Starting device: {}", device.device_id)
  _run_cmd(["docker", "start", device.container_name])
  _adb_reconnect(device.adb_port)
  _adb_wait_for_boot(device.adb_port)
  _setup_adb_keyboard(device.adb_port)


def _is_mounted(mount_dir: str) -> bool:
  """Check if a directory is mounted."""
  try:
    res = _run_cmd(["mount"], check=False)
    return mount_dir in res.stdout
  except Exception:
    return False


def _unmount(mount_dir: str) -> None:
  """Unmount a directory."""
  logger.info("Unmounting: {}", mount_dir)
  _run_cmd(["umount", mount_dir], check=False)


def remove_device(device: DeviceConfig) -> None:
  logger.info("Removing device: {}", device.device_id)
  _remove_container_if_exists(device.container_name)
  if os.path.isdir(device.mount_dir):
    # Check if directory is mounted (for readonly squashfs)
    if _is_mounted(device.mount_dir):
      _unmount(device.mount_dir)
    logger.info("Removing mount dir: {}", device.mount_dir)
    shutil.rmtree(device.mount_dir)


def create_devices(devices: Iterable[DeviceConfig]) -> None:
  for device in devices:
    create_device(device)


def start_devices(devices: Iterable[DeviceConfig]) -> None:
  for device in devices:
    start_device(device)


def remove_devices(devices: Iterable[DeviceConfig]) -> None:
  for device in devices:
    remove_device(device)


def _filter_devices(devices: list[DeviceConfig], device_ids: list[int] | None) -> list[DeviceConfig]:
  if not device_ids:
    return devices
  selected = [d for d in devices if d.device_id in set(device_ids)]
  missing = set(device_ids) - {d.device_id for d in selected}
  if missing:
    raise ValueError(f"Device id not found in config: {sorted(missing)}")
  return selected


def _build_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(description="redroid keeper")
  parser.add_argument(
    "--config",
    default="/home/zch/Documents/AgentEnv/device_config.yaml",
    help="path to device config yaml",
  )

  subparsers = parser.add_subparsers(dest="command", required=True)

  def add_device_id_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--id", type=int, action="append", dest="ids", help="device id")

  create_parser = subparsers.add_parser("create", help="create and start devices")
  add_device_id_args(create_parser)

  start_parser = subparsers.add_parser("start", help="start devices")
  add_device_id_args(start_parser)

  remove_parser = subparsers.add_parser("remove", help="remove devices")
  add_device_id_args(remove_parser)

  return parser


def main() -> int:
  # Ensure running as root: if not, try to re-exec with sudo
  if os.geteuid() != 0:
    sudo = _shutil.which("sudo")
    if sudo:
      logger.info("Not root — elevating with sudo")
      os.execvp(sudo, [sudo, sys.executable] + sys.argv)
    else:
      print("This program must be run as root. Please run with sudo.", file=sys.stderr)
      return 1
  parser = _build_parser()
  args = parser.parse_args()
  _ensure_storage_layout()
  devices = load_config(args.config)
  selected = _filter_devices(devices, args.ids)

  if args.command == "create":
    create_devices(selected)
  elif args.command == "start":
    start_devices(selected)
  elif args.command == "remove":
    remove_devices(selected)
  else:
    raise ValueError(f"Unknown command: {args.command}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())