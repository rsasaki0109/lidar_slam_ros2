#!/usr/bin/env python3
"""Release-facing ELF/ODR gate for an external RegistrationPlugin DSO.

The gate is deliberately independent of a live SLAM node.  It validates the
installed XML/DSO pair before a pluginlib load, compares the external DSO's
high-risk implementation symbols with a clean host DSO, and (unless
``--skip-load-smoke`` is requested) builds a C++14 consumer against the
installed loader and exercises a complete plugin session lifetime.

No result from this tool changes the built-in registration path.  A static
pass without the load smoke is reported as ``PASS_STATIC_ONLY`` and is never
treated as a production promotion.
"""

# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function

import argparse
import hashlib
import json
import os
import re
import shlex
import shutil
import stat
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


SCHEMA_VERSION = 1
BUILD_TIMEOUT_SECONDS = 120
SMOKE_TIMEOUT_SECONDS = 30
INTERFACE_BASE = "lidarslam::plugins::registration::RegistrationPlugin"
DEFAULT_NEEDED = frozenset(
    {
        "libc.so.6",
        "libm.so.6",
        "libpthread.so.0",
        "librt.so.1",
        "libdl.so.2",
        "libgcc_s.so.1",
        "libstdc++.so.6",
        "ld-linux-x86-64.so.2",
    }
)
ODR_PREFIXES = (
    "pclomp::",
    "small_gicp::",
    "fast_gicp::",
    "lidarslam::plugins::registration::RegistrationPlugin::",
)
ALLOWED_INTERFACE_ABI = (
    "typeinfo for " + INTERFACE_BASE,
    "typeinfo name for " + INTERFACE_BASE,
    "vtable for " + INTERFACE_BASE,
    "lidarslam::plugins::registration::PluginMetadata::~PluginMetadata()",
    # The public interface provides a no-op default for non-interruptible
    # providers. A plugin may emit this weak inline definition without owning
    # registration processing; real implementation methods remain rejected.
    "lidarslam::plugins::registration::RegistrationPlugin::requestCancel()",
)
UNDEFINED_PREFIXES = (
    "class_loader::",
    "console_bridge::",
    "std::",
    "typeinfo for std::",
    "vtable for std::",
    "VTT for std::",
    "vtable for __cxxabiv1::",
    "typeinfo for __cxxabiv1::",
    "operator ",
    "__cxa_",
    "__gxx_",
    "__gnu_",
    "_ITM_",
    "_Unwind_",
    "operator new",
    "operator delete",
)
ALLOWED_UNDEFINED_EXACT = frozenset(
    {
        "__gmon_start__",
        "__libc_single_threaded",
        "__stack_chk_fail",
        "calloc",
        "free",
        "getenv",
        "malloc",
        "memcmp",
        "memcpy",
        "memmove",
        "memset",
        "realloc",
        "strlen",
        "pthread_mutex_lock",
        "pthread_mutex_unlock",
        "pthread_mutex_init",
        "pthread_mutex_destroy",
        "pthread_once",
        "dladdr",
        "dlclose",
        "dlerror",
        "dlopen",
        "dlsym",
        "dlvsym",
        "dl_iterate_phdr",
    }
)


class GateError(RuntimeError):
    """A fail-closed gate error with a stable human-readable reason."""


def _path(value, label):
    path = Path(value).expanduser()
    if not path.is_absolute():
        raise GateError("{} must be absolute: {}".format(label, value))
    return path


def _regular(path, label, allow_symlink=False):
    path = _path(path, label)
    try:
        link = path.is_symlink()
        if link and not allow_symlink:
            raise GateError("{} must not be a symlink: {}".format(label, path))
        if not path.is_file():
            raise GateError("{} is not a regular file: {}".format(label, path))
    except OSError as exc:
        raise GateError("{} cannot be inspected: {}".format(label, exc))
    return path


def sha256_file(path):
    digest = hashlib.sha256()
    with open(str(path), "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _run(argv, label):
    try:
        completed = subprocess.run(
            list(argv), stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            universal_newlines=True, check=False,
        )
    except OSError as exc:
        raise GateError("{} could not run: {}".format(label, exc))
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip().splitlines()
        raise GateError(
            "{} failed with exit {}: {}".format(
                label, completed.returncode, detail[-1] if detail else "no diagnostic"
            )
        )
    return completed.stdout


def _tool_version(tool):
    path = shutil.which(tool)
    if path is None:
        return "missing"
    try:
        output = subprocess.run(
            [path, "--version"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            universal_newlines=True, check=False,
        ).stdout.strip().splitlines()
        return output[0] if output else "unknown"
    except OSError:
        return "unavailable"


def _regular_directory(value, label):
    path = _path(value, label)
    if path.is_symlink() or not path.is_dir():
        raise GateError("{} must be an existing non-symlink directory: {}".format(label, path))
    return path


def _tree_digest(path, label):
    """Hash a complete small source/install tree without following symlink dirs."""
    path = _regular_directory(path, label)
    entries = []
    for item in sorted(path.rglob("*"), key=lambda value: value.relative_to(path).as_posix()):
        relative = item.relative_to(path).as_posix()
        mode = stat.S_IMODE(os.lstat(str(item)).st_mode)
        if item.is_symlink():
            entries.append({"kind": "symlink", "mode": mode, "path": relative,
                            "target": os.readlink(str(item))})
        elif item.is_dir():
            entries.append({"kind": "directory", "mode": mode, "path": relative})
        elif item.is_file():
            entries.append({"kind": "file", "mode": mode, "path": relative,
                            "sha256": sha256_file(item), "size": item.stat().st_size})
        else:
            raise GateError("{} contains an unsupported filesystem entry: {}".format(label, item))
    payload = json.dumps(entries, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _git_provenance(repo_root):
    repo_root = _regular_directory(repo_root, "repository root")

    def git(*arguments):
        return _run(["git", "-C", str(repo_root)] + list(arguments), "git " + arguments[0]).strip()

    revision = git("rev-parse", "HEAD")
    tree = git("rev-parse", "HEAD^{tree}")
    status = git("status", "--porcelain=v1", "--untracked-files=all")
    diff = git("diff", "--binary", "HEAD", "--")
    untracked = []
    for name in git("ls-files", "--others", "--exclude-standard").splitlines():
        if not name:
            continue
        candidate = repo_root / name
        if candidate.is_symlink():
            untracked.append({"path": name, "kind": "symlink", "target": os.readlink(str(candidate))})
        elif candidate.is_file():
            untracked.append({"path": name, "kind": "file", "sha256": sha256_file(candidate)})
        elif candidate.is_dir():
            untracked.append({"path": name, "kind": "directory", "tree_sha256": _tree_digest(candidate, "untracked directory")})
        else:
            raise GateError("untracked repository entry is unsupported: {}".format(candidate))
    dirty_payload = {
        "diff_sha256": hashlib.sha256(diff.encode("utf-8")).hexdigest(),
        "status": status,
        "untracked": untracked,
    }
    dirty_text = json.dumps(dirty_payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return {
        "root": str(repo_root),
        "revision": revision,
        "head_tree": tree,
        "dirty": bool(status),
        "dirty_tree_sha256": hashlib.sha256(dirty_text.encode("utf-8")).hexdigest(),
        "dirty_tree_components": dirty_payload,
    }


def _canonical_command(value):
    if not value:
        return None
    try:
        parsed = json.loads(value)
        if not isinstance(parsed, list) or not all(isinstance(item, str) for item in parsed):
            raise ValueError
        argv = parsed
    except (TypeError, ValueError, json.JSONDecodeError):
        argv = shlex.split(value)
    if not argv:
        raise GateError("plugin build command must contain at least one argv element")
    canonical = json.dumps(argv, sort_keys=False, separators=(",", ":"), ensure_ascii=True)
    return {"argv": argv, "sha256": hashlib.sha256(canonical.encode("utf-8")).hexdigest()}


def _ros_distribution_version(ros_prefix):
    package_xml = Path(ros_prefix) / "share" / "ros_environment" / "package.xml"
    if package_xml.is_file() and not package_xml.is_symlink():
        try:
            version = ET.parse(str(package_xml)).getroot().findtext("version")
        except (OSError, ET.ParseError):
            version = None
        if version:
            return {"version": version.strip(), "source": str(package_xml),
                    "source_sha256": sha256_file(package_xml)}
    return {"version": "unverified", "source": None, "source_sha256": None}


def _installed_artifact(prefix, relative_name):
    """Find an artifact in a merged or isolated colcon install prefix."""
    prefix = Path(prefix)
    direct = prefix / relative_name
    if direct.is_file():
        return direct
    matches = sorted(
        path for path in prefix.glob("*/{}".format(relative_name)) if path.is_file()
    )
    if len(matches) == 1:
        return matches[0]
    if not matches:
        return None
    raise GateError("installed prefix has ambiguous {} artifacts: {}".format(
        relative_name, ", ".join(str(path) for path in matches)))


def _parse_symbols(text):
    defined = []
    undefined = []
    for line in text.splitlines():
        fields = line.strip().split(None, 2)
        if len(fields) < 2:
            continue
        if fields[0] in ("U", "w", "v"):
            symbol_type = fields[0]
            name = " ".join(fields[1:])
            destination = undefined
        elif len(fields) >= 3 and len(fields[1]) == 1:
            symbol_type = fields[1]
            name = fields[2]
            destination = defined
        else:
            continue
        destination.append({"type": symbol_type, "name": name})
    return defined, undefined


def _parse_dynamic(text):
    needed = []
    soname = None
    rpath = []
    runpath = []
    for line in text.splitlines():
        needed_match = re.search(r"Shared library: \[([^]]+)\]", line)
        if needed_match:
            needed.append(needed_match.group(1))
        soname_match = re.search(r"Library soname: \[([^]]+)\]", line)
        if soname_match:
            soname = soname_match.group(1)
        rpath_match = re.search(r"\(RPATH\).*?: \[([^]]*)\]", line)
        if rpath_match:
            rpath.append(rpath_match.group(1))
        runpath_match = re.search(r"\(RUNPATH\).*?: \[([^]]*)\]", line)
        if runpath_match:
            runpath.append(runpath_match.group(1))
    return {"needed": needed, "soname": soname, "rpath": rpath, "runpath": runpath}


def parse_elf(path):
    """Return a structured readelf/nm view, without executing the DSO."""
    path = _regular(path, "external DSO")
    header = _run(["readelf", "-hW", str(path)], "readelf header")
    if "ELF" not in header or "Type:" not in header or "DYN" not in header:
        raise GateError("external DSO is not an ELF shared object: {}".format(path))
    dynamic = _parse_dynamic(_run(["readelf", "-dW", str(path)], "readelf dynamic"))
    symbols_text = _run(["nm", "-D", "-C", "--defined-only", str(path)], "nm defined")
    undefined_text = _run(["nm", "-D", "-C", "--undefined-only", str(path)], "nm undefined")
    all_symbols_text = _run(["nm", "-C", "--defined-only", str(path)], "nm all defined")
    defined, _ = _parse_symbols(symbols_text)
    _, undefined = _parse_symbols(undefined_text)
    all_defined, _ = _parse_symbols(all_symbols_text)
    return {
        "path": str(path),
        "sha256": sha256_file(path),
        "header": header,
        "soname": dynamic["soname"],
        "needed": dynamic["needed"],
        "rpath": dynamic["rpath"],
        "runpath": dynamic["runpath"],
        "defined": defined,
        # The dynamic table is the load boundary; the complete symbol table
        # additionally catches a plugin that hides a bundled interface/ODR
        # implementation with local visibility.
        "all_defined": all_defined,
        "undefined": undefined,
    }


def _parse_manifest(path, class_id):
    path = _regular(path, "plugin manifest", allow_symlink=True)
    try:
        root = ET.parse(str(path)).getroot()
    except (OSError, ET.ParseError) as exc:
        raise GateError("plugin manifest is not valid XML: {}".format(exc))
    if root.tag != "library":
        raise GateError("plugin manifest root must be <library>")
    library_name = root.attrib.get("path", "")
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", library_name):
        raise GateError("plugin manifest library path is unsafe: {}".format(library_name))
    classes = []
    for node in root.findall("class"):
        name = node.attrib.get("name", "")
        class_type = node.attrib.get("type", "")
        base = node.attrib.get("base_class_type", "")
        if not name or not class_type or not base:
            raise GateError("plugin manifest has an incomplete <class> entry")
        classes.append({"name": name, "type": class_type, "base": base})
    if not classes:
        raise GateError("plugin manifest declares no classes")
    if len({entry["name"] for entry in classes}) != len(classes):
        raise GateError("plugin manifest declares duplicate class IDs")
    selected = next((entry for entry in classes if entry["name"] == class_id), None)
    if selected is None:
        raise GateError("class ID is not declared by the manifest: {}".format(class_id))
    if selected["base"] != INTERFACE_BASE:
        raise GateError("manifest base class mismatch: {}".format(selected["base"]))
    return {"path": str(path), "sha256": sha256_file(path), "library": library_name,
            "classes": classes, "selected": selected}


def _symbol_names(symbols):
    return {entry["name"] for entry in symbols}


def _all_definition_names(elf):
    return _symbol_names(elf["defined"]) | _symbol_names(elf.get("all_defined", []))


def _interface_report(elf, manifest):
    names = _all_definition_names(elf)
    selected_type = manifest["selected"]["type"]
    factories = sorted(
        name for name in names
        if "registerPlugin<" in name and INTERFACE_BASE in name
    )
    selected_factory = [name for name in factories if selected_type in name]
    derived_vtable = "vtable for {}".format(selected_type)
    derived_typeinfo = "typeinfo for {}".format(selected_type)
    unexpected = []
    for name in names:
        if name.startswith("lidarslam::plugins::registration::RegistrationPlugin::"):
            if not any(name.startswith(allowed) for allowed in ALLOWED_INTERFACE_ABI):
                unexpected.append(name)
    if not factories:
        raise GateError("DSO does not own a RegistrationPlugin factory symbol")
    if not selected_factory:
        raise GateError("DSO factory symbols do not mention manifest class type {}".format(selected_type))
    if derived_vtable not in names or derived_typeinfo not in names:
        raise GateError("DSO does not own the selected class RTTI/vtable")
    if unexpected:
        raise GateError("DSO exports unexpected RegistrationPlugin implementation symbols: {}".format(
            ", ".join(sorted(unexpected))))
    interface_symbols = sorted(
        name for name in names
        if INTERFACE_BASE in name or "lidarslam::plugins::registration::" in name
    )
    return {
        "selected_type": selected_type,
        "factory_symbols": factories,
        "selected_factory_symbols": selected_factory,
        "selected_vtable": derived_vtable,
        "selected_typeinfo": derived_typeinfo,
        "interface_symbols": interface_symbols,
        "unexpected_interface_symbols": unexpected,
    }


def _resolve_dependency(name, dso, prefixes):
    candidates = [dso.parent / name]
    for prefix in prefixes:
        candidates.append(Path(prefix) / "lib" / name)
        candidates.append(Path(prefix) / "lib64" / name)
    for candidate in candidates:
        try:
            if candidate.is_file() and not candidate.is_symlink():
                return str(candidate)
        except OSError:
            continue
    # ldconfig is used only as a resolver; the DSO is never loaded here.
    try:
        output = subprocess.run(
            ["ldconfig", "-p"], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            universal_newlines=True, check=False,
        ).stdout
        for line in output.splitlines():
            if re.search(r"\b{}\s+\(".format(re.escape(name)), line):
                candidate = line.rsplit("=>", 1)[-1].strip()
                if Path(candidate).is_file():
                    return candidate
    except OSError:
        pass
    return None


def _undefined_unknown(elf):
    unknown = []
    for entry in elf["undefined"]:
        if entry["type"] not in ("U", "w"):
            continue
        name = entry["name"].split("@", 1)[0]
        if name in ALLOWED_UNDEFINED_EXACT or any(
            name.startswith(prefix) for prefix in UNDEFINED_PREFIXES):
            continue
        unknown.append(name)
    return sorted(set(unknown))


def _odr_symbols(names):
    result = set()
    for name in names:
        if not any(name.startswith(prefix) for prefix in ODR_PREFIXES):
            continue
        if any(name.startswith(allowed) for allowed in ALLOWED_INTERFACE_ABI):
            continue
        if "typeinfo" in name or "typeinfo name" in name or "vtable" in name:
            continue
        if "registerPlugin<" in name or "MetaObject<" in name:
            continue
        result.add(name)
    return result


def _check_static(prefix, dso, manifest, host_dsos, allow_needed):
    elf = parse_elf(dso)
    expected_soname = Path(dso).name
    if elf["soname"] != expected_soname:
        raise GateError("SONAME mismatch: expected {}, got {}".format(expected_soname, elf["soname"]))
    if elf["rpath"] or elf["runpath"]:
        raise GateError("RPATH/RUNPATH is forbidden for an installed external DSO")
    allowed = set(DEFAULT_NEEDED) | set(allow_needed)
    dependency_paths = {}
    unresolved_needed = []
    for needed in elf["needed"]:
        if needed not in allowed:
            raise GateError("non-allowlisted DT_NEEDED dependency: {}".format(needed))
        dependency_prefixes = [prefix]
        if Path("/opt/ros/jazzy").is_dir():
            dependency_prefixes.append(Path("/opt/ros/jazzy"))
        resolved = _resolve_dependency(needed, dso, dependency_prefixes)
        if resolved is None:
            unresolved_needed.append(needed)
        else:
            dependency_paths[needed] = resolved
    if unresolved_needed:
        raise GateError("unresolved allowlisted DT_NEEDED dependencies: {}".format(
            ", ".join(unresolved_needed)))
    unknown = _undefined_unknown(elf)
    if unknown:
        raise GateError("unresolved non-allowlisted dynamic symbols: {}".format(
            ", ".join(unknown[:12])))
    manifest_library = manifest.get("library") if isinstance(manifest, dict) else None
    if not manifest_library:
        raise GateError("plugin manifest library binding is missing")
    expected_library_soname = "lib{}.so".format(manifest_library)
    if elf["soname"] != expected_library_soname:
        raise GateError(
            "plugin manifest library does not bind the inspected DSO: expected SONAME {}, got {}".format(
                expected_library_soname, elf["soname"]))
    interface = _interface_report(elf, manifest)
    host_reports = []
    external_odr = _odr_symbols(_all_definition_names(elf))
    collisions = set()
    for host_dso in host_dsos:
        host = parse_elf(host_dso)
        host_names = _all_definition_names(host)
        host_odr = _odr_symbols(host_names)
        collision = sorted(external_odr & host_odr)
        collisions.update(collision)
        host_reports.append({"path": host["path"], "sha256": host["sha256"],
                             "odr_symbols": sorted(host_odr)})
    if collisions:
        raise GateError("duplicate host/plugin ODR implementation symbols: {}".format(
            ", ".join(sorted(collisions)[:12])))
    return {
        "dso": {key: value for key, value in elf.items()
                if key not in ("header", "defined", "all_defined", "undefined")},
        "dynamic_defined_symbol_count": len(elf["defined"]),
        "all_defined_symbol_count": len(elf.get("all_defined", [])),
        "manifest": manifest,
        "dependencies": dependency_paths,
        "allowlisted_needed": sorted(allowed),
        "undefined_symbol_count": len(elf["undefined"]),
        "interface_ownership": interface,
        "external_odr_symbols": sorted(external_odr),
        "host_dsos": host_reports,
        "odr_collisions": sorted(collisions),
    }


CONSUMER_SOURCE = r'''#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include "lidarslam_registration_loader/registration_plugin_loader.hpp"

namespace registration = lidarslam::plugins::registration;
namespace shell = lidarslam::plugins::registration::shell;

int main(int argc, char ** argv)
{
  if (argc != 3) { std::cerr << "usage: smoke CLASS MANIFEST\n"; return 2; }
  std::shared_ptr<shell::RegistrationPluginSession> session;
  {
    shell::RegistrationPluginLoader loader(shell::kRegistrationPluginBasePackage, {argv[2]});
    shell::LoadRequest request;
    request.class_id = argv[1];
    const shell::LoadResult loaded = loader.load(request);
    if (!loaded.ok()) { std::cerr << loaded.failure.message << "\n"; return 3; }
    session = loaded.session;
    if (!session || !session->hasExternalLoaderLease()) { return 4; }
    if (session->classId() != request.class_id) { return 5; }
  }
  if (!session || !session->hasExternalLoaderLease()) { return 6; }
  registration::PointCloud::Ptr cloud(new registration::PointCloud());
  registration::PointT point;
  point.x = 1.0F; point.y = 2.0F; point.z = 3.0F; point.intensity = 4.0F;
  cloud->push_back(point);
  std::string error;
  if (!session->plugin()->setInputTarget(cloud, &error)) { std::cerr << error << "\n"; return 7; }
  registration::AlignmentRequest request;
  request.source = cloud;
  request.initial_guess_enabled = false;
  const registration::AlignmentResult result = session->plugin()->align(request);
  if (result.failure != registration::FailureCode::kNone || !result.converged ||
      !result.aligned_source || result.aligned_source->size() != cloud->size()) { return 8; }
  session->plugin()->reset();
  return 0;
}
'''

INTERFACE_CONSUMER_SOURCE = r'''#include <lidarslam_plugin_interfaces/registration.hpp>

namespace registration = lidarslam::plugins::registration;

class Cxx14Consumer final : public registration::RegistrationPlugin
{
public:
  registration::PluginMetadata metadata() const override
  {
    registration::PluginMetadata value;
    value.class_id = "external/cxx14-consumer";
    value.implementation_version = "gate";
    value.license = "BSD-2-Clause";
    value.api_version = registration::kHostApiVersion;
    return value;
  }

  registration::Capabilities capabilities() const override
  {
    return registration::Capabilities().add(registration::Capability::kDeterministic);
  }

  bool configure(const registration::ParameterMap &, std::string *) override {return true;}

  bool setInputTarget(const registration::PointCloudConstPtr & target, std::string *) override
  {
    return static_cast<bool>(target);
  }

  registration::AlignmentResult align(const registration::AlignmentRequest &) override
  {
    registration::AlignmentResult result;
    result.failure = registration::FailureCode::kNone;
    result.converged = true;
    return result;
  }

  void reset() noexcept override {}
};

int main()
{
  Cxx14Consumer consumer;
  return consumer.metadata().api_version.major == registration::kHostApiVersion.major &&
         consumer.capabilities().has(registration::Capability::kDeterministic) ? 0 : 1;
}
'''


def _compile_and_run_smoke(
    prefix, loader_include, interface_include, loader_library, ros_prefix,
    dso, manifest, class_id, work
):
    compiler = shutil.which("g++") or shutil.which("c++")
    if compiler is None:
        raise GateError("C++14 consumer smoke requires g++ or c++")
    prefix = Path(prefix)
    ros_prefix = Path(ros_prefix)
    source = work / "registration_plugin_dso_smoke.cpp"
    interface_source = work / "registration_plugin_interface_cpp14.cpp"
    interface_object = work / "registration_plugin_interface_cpp14.o"
    executable = work / "registration_plugin_dso_smoke"
    source.write_text(CONSUMER_SOURCE, encoding="utf-8")
    interface_source.write_text(INTERFACE_CONSUMER_SOURCE, encoding="utf-8")
    include_dirs = [
        prefix / "include", Path(loader_include), Path(interface_include), ros_prefix / "include",
        ros_prefix / "include" / "pluginlib",
        ros_prefix / "include" / "class_loader", ros_prefix / "include" / "ament_index_cpp",
        ros_prefix / "include" / "rcpputils", ros_prefix / "include" / "rcutils",
        Path("/usr/include/eigen3"),
        Path("/usr/include/pcl-1.14"),
    ]
    # PCL 1.14 exposes GNU anonymous structs in its public point_types header;
    # use the same warning policy as the package's clean C++14 consumer checks
    # while keeping those third-party pedantic diagnostics out of this gate.
    interface_command = [compiler, "-std=c++14", "-O2", "-Wall", "-Wextra", "-Werror"]
    interface_command.extend(["-I" + str(path) for path in include_dirs])
    interface_command.extend(["-c", str(interface_source), "-o", str(interface_object)])
    try:
        interface_build = subprocess.run(
            interface_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            universal_newlines=True, check=False, timeout=BUILD_TIMEOUT_SECONDS)
    except subprocess.TimeoutExpired:
        raise GateError("C++14 interface consumer build exceeded its timeout")
    if interface_build.returncode != 0:
        diagnostic = (interface_build.stderr or interface_build.stdout).strip()
        raise GateError("C++14 interface consumer build failed: {}".format(diagnostic[-5000:]))
    command = [compiler, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror"]
    command.extend(["-I" + str(path) for path in include_dirs])
    command.extend([str(source), "-o", str(executable), "-L" + str(loader_library.parent),
                    "-L" + str(prefix / "lib"),
                    "-L" + str(ros_prefix / "lib"),
                    "-Wl,-rpath," + str(prefix / "lib") + ":" + str(ros_prefix / "lib")])
    command.extend([
        "-llidarslam_registration_loader", "-lpcl_common", "-lboost_system",
        "-lboost_filesystem", "-lboost_atomic", "-lboost_iostreams",
        "-lboost_serialization", "-lament_index_cpp", "-lclass_loader",
        "-lconsole_bridge", "-lrcpputils", "-lrcutils", "-ltinyxml2", "-ldl",
        "-latomic", "-pthread",
    ])
    try:
        completed = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   universal_newlines=True, check=False,
                                   timeout=BUILD_TIMEOUT_SECONDS)
    except subprocess.TimeoutExpired:
        raise GateError("C++ loader consumer build exceeded its timeout")
    if completed.returncode != 0:
        diagnostic = (completed.stderr or completed.stdout).strip()
        raise GateError("C++14 clean consumer build failed: {}".format(diagnostic[-5000:]))
    env = os.environ.copy()
    package_prefixes = [
        str(prefix), str(loader_library.parent.parent), str(Path(dso).parent.parent),
        str(Path(interface_include).parent), str(ros_prefix)
    ]
    env["AMENT_PREFIX_PATH"] = ":".join(dict.fromkeys(package_prefixes))
    env["LD_LIBRARY_PATH"] = "{}:{}:{}".format(
        loader_library.parent, Path(dso).parent, ros_prefix / "lib")
    try:
        run = subprocess.run([str(executable), class_id, str(manifest)], env=env,
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             universal_newlines=True, check=False,
                             timeout=SMOKE_TIMEOUT_SECONDS)
    except subprocess.TimeoutExpired:
        raise GateError("C++ plugin session smoke exceeded its timeout")
    if run.returncode != 0:
        raise GateError("C++14 plugin session smoke failed with exit {}: {}".format(
            run.returncode, (run.stderr or run.stdout).strip()[-400:]))
    return {
        "status": "PASS",
        "interface_standard": "c++14",
        "loader_standard": "c++17 (Jazzy pluginlib shell)",
        "compiler": compiler,
        "interface_source_sha256": sha256_file(interface_source),
        "interface_object_sha256": sha256_file(interface_object),
        "interface_build_argv_sha256": hashlib.sha256(
            json.dumps(interface_command, separators=(",", ":")).encode("utf-8")).hexdigest(),
        "source_sha256": sha256_file(source),
        "executable_sha256": sha256_file(executable),
        "build_argv_sha256": hashlib.sha256(
            json.dumps(command, separators=(",", ":")).encode("utf-8")).hexdigest(),
        "run_argv_sha256": hashlib.sha256(
            json.dumps([str(executable), class_id, str(manifest)], separators=(",", ":")).encode("utf-8")).hexdigest(),
    }


def _durable_evidence_metadata(args, prefix, smoke):
    requested = [
        "evidence_root", "repo_root", "sdk_source", "sdk_install_prefix", "plugin_source",
        "plugin_build_command", "negative_coverage_version", "negative_coverage_source",
    ]
    supplied = [getattr(args, name, None) for name in requested]
    if not any(value is not None for value in supplied):
        return None
    missing = [name for name, value in zip(requested, supplied) if value is None]
    if missing:
        raise GateError("durable evidence metadata is incomplete: {}".format(", ".join(missing)))

    ros_prefix = _regular_directory(args.ros_prefix, "ROS prefix")
    sdk_source = _regular_directory(args.sdk_source, "SDK source tree")
    sdk_install_prefix = _regular_directory(args.sdk_install_prefix, "SDK install prefix")
    plugin_source = _regular_directory(args.plugin_source, "external plugin source tree")
    negative_source = _regular(args.negative_coverage_source, "negative gate source")
    evidence_root = _regular_directory(args.evidence_root, "evidence root")
    sdk_header = getattr(args, "sdk_header", None)
    if sdk_header is None:
        sdk_header = sdk_source / "include" / "lidarslam_plugin_interfaces" / "registration.hpp"
    sdk_header = _regular(sdk_header, "SDK public header")
    host_install_prefix = getattr(args, "host_install_prefix", None)
    host_install = None
    if host_install_prefix is not None:
        host_install = _regular_directory(host_install_prefix, "host install prefix")

    metadata = {
        "schema_version": 1,
        "evidence_root": str(evidence_root),
        "ros": {
            "distribution": ros_prefix.name,
            "prefix": str(ros_prefix),
            "version": _ros_distribution_version(ros_prefix),
        },
        "compiler": {
            "g++": _tool_version("g++"),
            "interface_consumer_standard": smoke.get("interface_standard"),
            "loader_consumer_standard": smoke.get("loader_standard"),
        },
        "repository": _git_provenance(args.repo_root),
        "installed_prefix": {
            "path": str(prefix),
            "tree_sha256": _tree_digest(prefix, "installed host prefix"),
        },
        "sdk": {
            "source_path": str(sdk_source),
            "source_tree_sha256": _tree_digest(sdk_source, "SDK source tree"),
            "install_prefix": str(sdk_install_prefix),
            "install_tree_sha256": _tree_digest(sdk_install_prefix, "SDK install prefix"),
            "public_header": str(sdk_header),
            "public_header_sha256": sha256_file(sdk_header),
        },
        "external_plugin": {
            "source_path": str(plugin_source),
            "source_tree_sha256": _tree_digest(plugin_source, "external plugin source tree"),
            "build_command": _canonical_command(args.plugin_build_command),
        },
        "negative_gate_coverage": {
            "version": str(args.negative_coverage_version),
            "source_path": str(negative_source),
            "source_sha256": sha256_file(negative_source),
        },
    }
    if host_install is not None:
        metadata["host_install_prefix"] = {
            "path": str(host_install),
            "tree_sha256": _tree_digest(host_install, "host install prefix"),
        }
    return metadata


def _canonical_json(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True) + "\n"


def _seal(path, data):
    path = _path(path, "receipt")
    if path.exists() or path.is_symlink():
        raise GateError("receipt overwrite or symlink refused: {}".format(path))
    parent = path.parent
    if not parent.is_dir() or parent.is_symlink():
        raise GateError("receipt parent must be an existing non-symlink directory: {}".format(parent))
    payload = data.encode("utf-8")
    temp = parent / ("." + path.name + ".tmp-{}-{}".format(os.getpid(), os.urandom(5).hex()))
    fd = os.open(str(temp), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(str(temp), 0o444)
        os.link(str(temp), str(path))
    except FileExistsError:
        raise GateError("receipt overwrite refused: {}".format(path))
    finally:
        try:
            temp.unlink()
        except FileNotFoundError:
            pass
    return path


def seal_receipt(path, receipt):
    payload = _canonical_json(receipt)
    receipt_path = _seal(path, payload)
    digest = hashlib.sha256(payload.encode("utf-8")).hexdigest()
    sidecar = Path(str(receipt_path) + ".sha256")
    _seal(sidecar, "{}  {}\n".format(digest, receipt_path.name))
    return str(receipt_path), digest


def run_gate(args):
    prefix = _path(args.prefix, "installed host prefix")
    if not prefix.is_dir() or prefix.is_symlink():
        raise GateError("installed host prefix must be a non-symlink directory: {}".format(prefix))
    dso = _regular(args.dso, "external DSO")
    manifest = _parse_manifest(args.manifest, args.class_id)
    host_dsos = [_regular(item, "host DSO") for item in (args.host_dso or [])]
    if not host_dsos:
        raise GateError("at least one clean installed host DSO is required for ODR ownership")
    static = _check_static(prefix, dso, manifest, host_dsos, args.allow_needed or [])
    smoke = {"status": "SKIPPED", "reason": "--skip-load-smoke"}
    if not args.skip_load_smoke:
        ros_prefix = _path(args.ros_prefix, "ROS prefix")
        loader_library = _regular(
            args.loader_library or str(
                _installed_artifact(prefix, "lib/liblidarslam_registration_loader.so") or ""),
            "installed loader library")
        loader_include = _path(
            args.loader_include or str(prefix / "include"), "loader include directory")
        if args.loader_include is None and not (loader_include / "lidarslam_registration_loader" /
                                                "registration_plugin_loader.hpp").is_file():
            candidate = _installed_artifact(
                prefix, "include/lidarslam_registration_loader/registration_plugin_loader.hpp")
            if candidate is not None:
                loader_include = candidate.parent.parent
        interface_include = _path(
            args.interface_include or str(prefix / "include"), "interface include directory")
        if args.interface_include is None and not (interface_include / "lidarslam_plugin_interfaces" /
                                                   "registration.hpp").is_file():
            candidate = _installed_artifact(
                prefix, "include/lidarslam_plugin_interfaces/registration.hpp")
            if candidate is not None:
                interface_include = candidate.parent.parent
        if not (loader_include / "lidarslam_registration_loader" /
                "registration_plugin_loader.hpp").is_file():
            raise GateError("installed loader header is missing: {}".format(loader_include))
        if not (interface_include / "lidarslam_plugin_interfaces" /
                "registration.hpp").is_file():
            raise GateError("installed interface header is missing: {}".format(interface_include))
        with tempfile.TemporaryDirectory(prefix="registration-plugin-dso-gate-") as temp:
            smoke = _compile_and_run_smoke(
                prefix, loader_include, interface_include, loader_library, ros_prefix,
                dso, manifest["path"], args.class_id, Path(temp))
    status = "PASS_STATIC_ONLY" if args.skip_load_smoke else "PASS"
    result = {
        "schema_version": SCHEMA_VERSION,
        "kind": "lidarslam-registration-plugin-dso-odr-gate",
        "status": status,
        "production_promotion": False,
        "default_builtin_behavior_changed": False,
        "host_prefix": str(prefix),
        "class_id": args.class_id,
        "static_gate": static,
        "load_session_smoke": smoke,
        "tools": {tool: _tool_version(tool) for tool in ("readelf", "nm", "g++", "ldconfig")},
        "humble_status": "NO_GO_UNVERIFIED" if not args.humble_verified else "VERIFIED",
    }
    durable_metadata = _durable_evidence_metadata(args, prefix, smoke)
    if durable_metadata is not None:
        result["durable_evidence"] = durable_metadata
    return result


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", required=True, help="clean installed host prefix")
    parser.add_argument("--dso", required=True, help="external plugin DSO")
    parser.add_argument("--manifest", required=True, help="pluginlib XML manifest")
    parser.add_argument("--class-id", required=True, help="exact manifest class ID to smoke")
    parser.add_argument("--host-dso", action="append", required=True,
                        help="clean host DSO used for ODR ownership; repeatable")
    parser.add_argument("--allow-needed", action="append", default=[],
                        help="additional exact DT_NEEDED name; repeatable")
    parser.add_argument("--ros-prefix", default="/opt/ros/jazzy")
    parser.add_argument(
        "--loader-include",
        help="installed loader include root; only use a source root for a deliberate clean-build diagnostic",
    )
    parser.add_argument(
        "--interface-include",
        help="installed interface include root; only use a source root for a deliberate clean-build diagnostic",
    )
    parser.add_argument("--loader-library", help="installed loader shared library")
    parser.add_argument("--receipt", help="new immutable JSON receipt path")
    parser.add_argument("--evidence-root", help="fresh durable evidence root")
    parser.add_argument("--repo-root", help="repository root for revision/dirty-tree binding")
    parser.add_argument("--sdk-source", help="public SDK source tree")
    parser.add_argument("--sdk-install-prefix", help="clean SDK install package prefix")
    parser.add_argument("--sdk-header", help="public SDK header; inferred when omitted")
    parser.add_argument("--plugin-source", help="independently built external plugin source tree")
    parser.add_argument("--plugin-build-command", help="JSON argv or shell-like build command to hash")
    parser.add_argument("--host-install-prefix", help="clean host plugin install package prefix")
    parser.add_argument("--negative-coverage-version", help="negative-gate fixture/version identifier")
    parser.add_argument("--negative-coverage-source", help="negative-gate test source to hash")
    parser.add_argument("--skip-load-smoke", action="store_true")
    parser.add_argument("--humble-verified", action="store_true")
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    try:
        receipt = run_gate(args)
        if args.receipt:
            receipt_path, digest = seal_receipt(args.receipt, receipt)
            receipt["receipt_path"] = receipt_path
            receipt["receipt_sha256"] = digest
        print(_canonical_json(receipt), end="")
        return 0
    except GateError as exc:
        failure = {
            "schema_version": SCHEMA_VERSION,
            "kind": "lidarslam-registration-plugin-dso-odr-gate",
            "status": "FAIL_CLOSED",
            "failure": str(exc),
        }
        if args.receipt:
            try:
                path, digest = seal_receipt(args.receipt, failure)
                failure["receipt_path"] = path
                failure["receipt_sha256"] = digest
            except GateError as seal_error:
                failure["receipt_seal_error"] = str(seal_error)
        print(_canonical_json(failure), file=sys.stderr, end="")
        return 1


if __name__ == "__main__":
    sys.exit(main())
