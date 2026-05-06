#!/usr/bin/env python3
"""server.py — Flask backend for the drone log viewer.

Single-user local tool. Binds to loopback only.
"""

from __future__ import annotations

import json
from pathlib import Path

from flask import Flask, Response, abort, jsonify, send_from_directory

# parse_log (and pymavlink) are imported lazily on cache miss — see get_log().
# A fresh JSON cache is served without ever touching pymavlink, so the bundled
# demo log loads with just Flask installed.


BASE_DIR = Path(__file__).resolve().parent
LOG_DIR = BASE_DIR / "logs"
STATIC_DIR = BASE_DIR / "static"
VALID_EXTS = {".bin", ".tlog", ".log"}


app = Flask(__name__, static_folder=None)


@app.after_request
def no_cache(resp):
    # Local dev tool — never let the browser serve a stale viewer.html or
    # /api/log/<name> after you edit the source. Flask's default 12-hour
    # static cache silently masks code changes through Ctrl+Shift+R.
    resp.headers["Cache-Control"] = "no-store, max-age=0"
    return resp


def is_valid_log(p: Path) -> bool:
    # Skip dotfiles and JSON cache files; only honor known log extensions.
    if p.name.startswith("."):
        return False
    if p.suffix.lower() == ".json":
        return False
    return p.is_file() and p.suffix.lower() in VALID_EXTS


@app.route("/")
def index() -> Response:
    return send_from_directory(STATIC_DIR, "viewer.html")


@app.route("/api/logs")
def list_logs() -> Response:
    names = sorted(p.name for p in LOG_DIR.iterdir() if is_valid_log(p))
    return jsonify(names)


@app.route("/api/log/", defaults={"name": ""})
@app.route("/api/log/<path:name>")
def get_log(name: str) -> Response:
    # Path-traversal guard: <path:> normalizes ".." segments before dispatch,
    # so we also need a catch-all 400 for any request that didn't survive to
    # this view with its original token intact.
    if not name or "/" in name or "\\" in name or ".." in name:
        abort(400)

    src = LOG_DIR / name
    if not src.is_file() or src.suffix.lower() not in VALID_EXTS:
        abort(404)

    cache = LOG_DIR / f"{name}.json"
    # mtime cache: parsing a multi-MB DataFlash log is slow; reuse the cached
    # JSON unless the source has been modified more recently.
    if cache.is_file() and cache.stat().st_mtime >= src.stat().st_mtime:
        return Response(cache.read_text(), mimetype="application/json")

    # Cache miss — pymavlink is required to (re-)parse. Lazy-imported so the
    # bundled demo log can be served with only Flask installed.
    try:
        from parse import parse_log
    except ImportError:
        return Response(
            "pymavlink not installed; run `pip install -r requirements.txt` "
            "to enable log parsing",
            status=503,
        )

    try:
        data = parse_log(src)
    except Exception as e:
        app.logger.exception("parse failed for %s", src)
        return Response(f"parse failed: {e}", status=500)

    text = json.dumps(data, default=str)
    cache.write_text(text)
    return Response(text, mimetype="application/json")


if __name__ == "__main__":
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    app.run(host="127.0.0.1", port=5000, debug=True)
