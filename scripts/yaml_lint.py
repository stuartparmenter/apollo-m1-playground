# © Copyright 2025 Stuart Parmenter
# SPDX-License-Identifier: MIT

#!/usr/bin/env python3
import sys, os
from collections import defaultdict, namedtuple
from ruamel.yaml import YAML
from ruamel.yaml.constructor import ConstructorError, DuplicateKeyError
from ruamel.yaml.parser import ParserError
from ruamel.yaml.scanner import ScannerError

# ---------- Types / globals

Origin = namedtuple("Origin", ["file", "line"])
ctx_holder = {"ctx": None}  # global access for constructors

# ---------- Small helpers

def _node_base_dir(node):
    name = getattr(getattr(node, "start_mark", None), "name", None)
    if name and not os.path.isdir(name):
        return os.path.dirname(name)
    return name or os.getcwd()

def _merge_docs(docs, path_for_label):
    docs = [d for d in docs if d is not None]
    if not docs:
        return {"__file__": os.path.abspath(path_for_label)}
    if all(isinstance(d, dict) for d in docs):
        out = {}
        for d in docs:
            out.update(d)
        out.setdefault("__file__", os.path.abspath(path_for_label))
        return out
    if len(docs) == 1:
        d = docs[0]
        if isinstance(d, dict):
            d.setdefault("__file__", os.path.abspath(path_for_label))
        return d
    return {"__file__": os.path.abspath(path_for_label), "__docs__": docs}

# ---------- Secrets support

class Context:
    def __init__(self, top_config_path):
        self.top_config_dir = os.path.dirname(os.path.abspath(top_config_path))
        self.secrets = None

    def _load_secrets(self, fallback_dir):
        candidates = [
            os.path.join(self.top_config_dir, "secrets.yaml"),
            os.path.join(fallback_dir, "secrets.yaml"),
        ]
        for p in candidates:
            if os.path.isfile(p):
                try:
                    local_yaml = _new_yaml()
                    with open(p, "r", encoding="utf-8") as f:
                        data = local_yaml.load(f) or {}
                    if not isinstance(data, dict):
                        data = {}
                    self.secrets = data
                    return
                except Exception:
                    self.secrets = {}
                    return
        self.secrets = {}

# ---------- Tag constructors + YAML factory

def _read_yaml(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            text = f.read()
        if text.strip() == "":
            return {"__file__": os.path.abspath(path)}
        local_yaml = _new_yaml()
        docs = list(local_yaml.load_all(text))
        return _merge_docs(docs, path)
    except (ScannerError, ParserError, DuplicateKeyError) as e:
        raise ConstructorError(f"YAML parse error in {path}: {e}") from e
    except FileNotFoundError as e:
        raise ConstructorError(f"Included path not found: {path}") from e

def _include_constructor(loader, node):
    if not isinstance(node.value, str):
        raise ConstructorError("!include expects a string path")
    base_dir = _node_base_dir(node)
    full = os.path.normpath(os.path.join(base_dir, node.value))
    if os.path.isdir(full):
        raise ConstructorError(f"!include given a directory ({full}). Use !include_dir_* instead.")
    return _read_yaml(full)

def _include_dir_named_constructor(loader, node):
    if not isinstance(node.value, str):
        raise ConstructorError("!include_dir_named expects a string path")
    base_dir = _node_base_dir(node)
    dir_path = os.path.normpath(os.path.join(base_dir, node.value))
    if not os.path.isdir(dir_path):
        raise ConstructorError(f"!include_dir_named: '{dir_path}' is not a directory")
    merged = {}
    for fname in sorted(os.listdir(dir_path)):
        if not fname.lower().endswith((".yaml", ".yml")):
            continue
        key = os.path.splitext(fname)[0]
        full = os.path.join(dir_path, fname)
        merged[key] = _read_yaml(full)
    merged.setdefault("__file__", dir_path)
    return merged

def _include_dir_merge_named_constructor(loader, node):
    if not isinstance(node.value, str):
        raise ConstructorError("!include_dir_merge_named expects a string path")
    base_dir = _node_base_dir(node)
    dir_path = os.path.normpath(os.path.join(base_dir, node.value))
    if not os.path.isdir(dir_path):
        raise ConstructorError(f"!include_dir_merge_named: '{dir_path}' is not a directory")
    merged = {}
    for fname in sorted(os.listdir(dir_path)):
        if not fname.lower().endswith((".yaml", ".yml")):
            continue
        full = os.path.join(dir_path, fname)
        d = _read_yaml(full)
        if isinstance(d, dict):
            merged.update(d)
    merged.setdefault("__file__", dir_path)
    return merged

def _include_dir_list_constructor(loader, node):
    if not isinstance(node.value, str):
        raise ConstructorError("!include_dir_list expects a string path")
    base_dir = _node_base_dir(node)
    dir_path = os.path.normpath(os.path.join(base_dir, node.value))
    if not os.path.isdir(dir_path):
        raise ConstructorError(f"!include_dir_list: '{dir_path}' is not a directory")
    out = []
    for fname in sorted(os.listdir(dir_path)):
        if not fname.lower().endswith((".yaml", ".yml")):
            continue
        full = os.path.join(dir_path, fname)
        out.append(_read_yaml(full))
    return out

def _include_dir_merge_list_constructor(loader, node):
    if not isinstance(node.value, str):
        raise ConstructorError("!include_dir_merge_list expects a string path")
    base_dir = _node_base_dir(node)
    dir_path = os.path.normpath(os.path.join(base_dir, node.value))
    if not os.path.isdir(dir_path):
        raise ConstructorError(f"!include_dir_merge_list: '{dir_path}' is not a directory")
    out = []
    for fname in sorted(os.listdir(dir_path)):
        if not fname.lower().endswith((".yaml", ".yml")):
            continue
        full = os.path.join(dir_path, fname)
        d = _read_yaml(full)
        if isinstance(d, list):
            out.extend(d)
        else:
            out.append(d)
    return out

def _secret_constructor(loader, node):
    key = str(node.value)
    ctx = ctx_holder["ctx"]
    base_dir = _node_base_dir(node)
    if ctx.secrets is None:
        ctx._load_secrets(base_dir)
    val = ctx.secrets.get(key)
    return val if val is not None else f"<<UNRESOLVED_SECRET:{key}>>"

def _register_constructors(y):
    y.Constructor.add_constructor(u'!include', _include_constructor)
    y.Constructor.add_constructor(u'!include_dir_named', _include_dir_named_constructor)
    y.Constructor.add_constructor(u'!include_dir_list', _include_dir_list_constructor)
    y.Constructor.add_constructor(u'!include_dir_merge_named', _include_dir_merge_named_constructor)
    y.Constructor.add_constructor(u'!include_dir_merge_list', _include_dir_merge_list_constructor)
    y.Constructor.add_constructor(u'!secret', _secret_constructor)

def _new_yaml():
    y = YAML(typ="rt")
    y.preserve_quotes = True
    y.allow_duplicate_keys = True
    _register_constructors(y)
    return y

# One global for the top-level read (includes use fresh instances)
yaml = _new_yaml()

# ---------- Lint core

def _file_of(doc):
    return doc.get("__file__", "<memory>") if isinstance(doc, dict) else "<memory>"

def _walk(doc, callback, path=(), pkg_index=None, pkg_order=None):
    """
    Generic tree walk over dicts/lists. Tracks:
      - pkg_index: current package tier index (int) if inside a package subtree
      - pkg_order: list of (package_doc, index) in discovery order
    Calls callback(node, pkg_index)
    """
    if isinstance(doc, dict):
        callback(doc, pkg_index)

        # Handle 'packages' specially: may be dict OR list in ESPHome
        if "packages" in doc:
            pkgs = doc.get("packages")
            # Case A: packages as dict (name -> package-doc)
            if isinstance(pkgs, dict):
                for _, child in pkgs.items():
                    if isinstance(child, dict):
                        idx = len(pkg_order)
                        pkg_order.append((child, idx))
                        _walk(child, callback, path + ("packages",), pkg_index=idx, pkg_order=pkg_order)
            # Case B: packages as list of package-docs (from '- !include file.yaml')
            elif isinstance(pkgs, list):
                for child in pkgs:
                    if isinstance(child, dict):
                        idx = len(pkg_order)
                        pkg_order.append((child, idx))
                        _walk(child, callback, path + ("packages",), pkg_index=idx, pkg_order=pkg_order)
                # Also walk non-dict items in the list (rare, but be safe)
                for child in pkgs:
                    if not isinstance(child, dict):
                        _walk(child, callback, path + ("packages",), pkg_index=pkg_index, pkg_order=pkg_order)

        # Recurse into all other values to catch substitutions anywhere
        for k, v in list(doc.items()):
            if k == "packages":
                continue  # already handled with indices above
            _walk(v, callback, path + (k,), pkg_index=pkg_index, pkg_order=pkg_order)

    elif isinstance(doc, list):
        for i, it in enumerate(doc):
            _walk(it, callback, path + (i,), pkg_index=pkg_index, pkg_order=pkg_order)

def lint(top_config):
    ctx_holder["ctx"] = Context(top_config)
    root = _read_yaml(top_config)
    if not isinstance(root, dict):
        print("Top-level YAML must be a mapping.", file=sys.stderr)
        return 2

    # Discovery pass: collect substitutions everywhere and remember package tiers
    subs_found = []  # (tier: int|None, key, value, Origin)
    pkg_order = []   # [(pkg_doc, index), ...]

    def visit(node, pkg_idx):
        if not isinstance(node, dict):
            return
        subs = node.get("substitutions")
        if isinstance(subs, dict):
            for k, v in subs.items():
                line = getattr(getattr(k, "lc", None), "line", None)
                line = (line + 1) if isinstance(line, int) else None
                subs_found.append( (pkg_idx, str(k), str(v), Origin(_file_of(node), line)) )

    _walk(root, visit, pkg_order=pkg_order)

    # Summaries
    num_pkgs = len(pkg_order)
    num_subs = len(subs_found)
    print(f"🔎 Discovered {num_pkgs} package(s), {num_subs} substitution(s).")

    # Partition into package vs main (None)
    pkg_subs = []
    main_subs = []
    for tier, k, v, origin in subs_found:
        if isinstance(tier, int):
            pkg_subs.append( (tier, k, v, origin) )
        else:
            main_subs.append( (k, v, origin) )

    # Collisions across packages (same key in two+ packages)
    by_key = defaultdict(list)
    for i, k, v, origin in pkg_subs:
        by_key[k].append((i, v, origin))
    collisions = {k: items for k, items in by_key.items() if len(items) > 1}

    # Effective values: earlier package < later package < main
    effective = {}
    winners = {}
    for i, k, v, origin in sorted(pkg_subs, key=lambda x: x[0]):
        effective[k] = (v, origin, f"package[{i}]")
        winners[k] = origin
    for k, v, origin in main_subs:
        effective[k] = (v, origin, "main")

    problems = 0
    if collisions:
        print("⚠️  Colliding substitutions across packages (later package wins):")
        for k, items in sorted(collisions.items()):
            print(f"  - '{k}':")
            for i, v, origin in sorted(items, key=lambda x: x[0]):
                loc = f"{origin.file}:{origin.line}" if origin.line else origin.file
                print(f"      package[{i}] -> {loc}  value={v}")
            w = max(items, key=lambda x: x[0])
            wloc = f"{w[2].file}:{w[2].line}" if w[2].line else w[2].file
            print(f"      ➜ winner: package[{w[0]}] @ {wloc}")
        problems += len(collisions)

    # Main masking package keys (info)
    masked = []
    pkg_keys = set(by_key.keys())
    for k, v, origin in main_subs:
        if k in pkg_keys:
            masked.append((k, origin, winners.get(k)))
    if masked:
        print("\nℹ️  Main file overrides package substitutions:")
        for k, main_origin, pkg_origin in masked:
            mloc = f"{main_origin.file}:{main_origin.line}" if main_origin.line else main_origin.file
            ploc = f"{pkg_origin.file}:{pkg_origin.line}" if pkg_origin and pkg_origin.line else (pkg_origin.file if pkg_origin else "?")
            print(f"  - '{k}': main {mloc} overrides package {ploc}")

    print("\n✅  Effective substitutions (after precedence):")
    if not effective:
        print("  (none found)")
    else:
        for k in sorted(effective.keys()):
            v, origin, tier = effective[k]
            loc = f"{origin.file}:{origin.line}" if origin.line else origin.file
            print(f"  {k} = {v}    [{tier} @ {loc}]")

    return 1 if problems else 0

# ---------- CLI

def _register_constructors(y):
    y.Constructor.add_constructor(u'!include', _include_constructor)
    y.Constructor.add_constructor(u'!include_dir_named', _include_dir_named_constructor)
    y.Constructor.add_constructor(u'!include_dir_list', _include_dir_list_constructor)
    y.Constructor.add_constructor(u'!include_dir_merge_named', _include_dir_merge_named_constructor)
    y.Constructor.add_constructor(u'!include_dir_merge_list', _include_dir_merge_list_constructor)
    y.Constructor.add_constructor(u'!secret', _secret_constructor)

def _new_yaml():
    y = YAML(typ="rt")
    y.preserve_quotes = True
    y.allow_duplicate_keys = True
    _register_constructors(y)
    return y

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: yaml_lint.py <path/to/top_config.yaml>", file=sys.stderr)
        sys.exit(2)
    sys.exit(lint(sys.argv[1]))
