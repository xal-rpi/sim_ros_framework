"""Local editor file service, confined to one export's selections directory.

The Windows browser talks to this WSL/Linux process; saves therefore use the
server's export directory rather than the browser's Downloads directory.
"""

from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import time

import numpy as np
from urllib.parse import urlparse, parse_qs

NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,79}")
KINDS = {
    "tracks": "0.3-track-recipe",
    "areas": "0.2-native-selection",
    "routes": "0.1-route-selection",
}


def selection_path(folder, kind, name):
    if kind not in KINDS or not NAME.fullmatch(name) or name.endswith("."):
        raise ValueError(
            "Use a simple name with letters, numbers, underscores or hyphens"
        )
    export = Path(folder).resolve()
    root = (export / "selections").resolve()
    if not root.is_relative_to(export):
        raise ValueError("Selections directory escapes the export")
    path = root / kind / (name.removesuffix(".json") + ".json")
    if not path.resolve().is_relative_to(root):
        raise ValueError("Selection path escapes the export")
    return path


def validate_document(folder, kind, document):
    if kind not in KINDS or not isinstance(document, dict):
        raise ValueError("Unknown selection kind or malformed document")
    if kind == "tracks":
        from track_recipe import validate_recipe

        validate_recipe(folder, document)
        return
    if kind == "routes":
        from route_selection import validate_recipe

        validate_recipe(folder, document)
        return
    from native_scene import load_survey

    meta = load_survey(folder)
    if (
        document.get("schema_version") != KINDS[kind]
        or document.get("frame") != "local_ENU"
    ):
        raise ValueError("Wrong selection document type or frame")
    if (
        document.get("source_mission") != meta["source"]["mission"]
        or document.get("source_files_sha256") != meta["files_sha256"]
    ):
        raise ValueError("Selection belongs to a different source export")
    bounds = np.asarray(document.get("bounds_enu_m"), float)
    focus = np.asarray(document.get("focus_xyz"), float)
    if (
        bounds.shape != (6,)
        or not np.isfinite(bounds).all()
        or (bounds[:3] >= bounds[3:]).any()
        or focus.shape != (3,)
        or not np.isfinite(focus).all()
    ):
        raise ValueError("Area selection needs finite ENU bounds and focus XYZ")


def save_document(folder, kind, name, document):
    path = selection_path(folder, kind, name)
    validate_document(folder, kind, document)
    content = json.dumps(document, indent=2, allow_nan=False) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        history = Path(folder) / "selections" / ".history" / kind
        if not history.resolve().is_relative_to(Path(folder).resolve()):
            raise ValueError("History directory escapes the export")
        history.mkdir(parents=True, exist_ok=True)
        (history / (path.stem + "." + str(time.time_ns()) + ".json")).write_bytes(
            path.read_bytes()
        )
    temp = path.with_name(path.name + "." + str(time.time_ns()) + ".tmp")
    temp.write_text(content)
    temp.replace(path)
    return path


class EditorHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, directory, **kwargs):
        self.export = Path(directory).resolve()
        super().__init__(*args, directory=str(self.export), **kwargs)

    def json_response(self, value, status=200):
        body = json.dumps(value).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        request = urlparse(self.path)
        try:
            if request.path == "/api/info":
                from route_selection import digest

                return self.json_response(
                    {
                        "export_directory": str(self.export),
                        "save_directory": str(self.export / "selections"),
                        "metadata_sha256": digest(self.export / "metadata.json"),
                    }
                )
            if request.path == "/api/selections":
                files = []
                for kind in KINDS:
                    for p in sorted((self.export / "selections" / kind).glob("*.json")):
                        if not p.is_symlink():
                            files.append({"kind": kind, "name": p.stem})
                return self.json_response({"files": files})
            if request.path == "/api/selection":
                query = parse_qs(request.query)
                path = selection_path(self.export, query["kind"][0], query["name"][0])
                return self.json_response(json.loads(path.read_text()))
            # Avoid following links out of the selected export even for static
            # reads. The server does not expose arbitrary local filesystem paths.
            path = Path(self.translate_path(self.path)).resolve()
            if not path.is_relative_to(self.export):
                return self.json_response({"error": "Outside export"}, 403)
            return super().do_GET()
        except (ValueError, KeyError, FileNotFoundError) as e:
            self.json_response({"error": str(e)}, 400)

    def do_POST(self):
        try:
            endpoint = urlparse(self.path).path
            if endpoint not in ["/api/save", "/api/export-route"]:
                return self.json_response({"error": "Unknown endpoint"}, 404)
            origin = self.headers.get("Origin")
            if origin and urlparse(origin).netloc != self.headers.get("Host"):
                return self.json_response(
                    {"error": "Use the editor from this local server"}, 403
                )
            if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                raise ValueError("Expected application/json")
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 10_000_000:
                raise ValueError("Selection body must be between 1 byte and 10 MB")
            body = json.loads(self.rfile.read(size))
            if endpoint == "/api/export-route":
                from route_selection import materialize

                name = body["name"]
                document = body["document"]
                selection_path(self.export, "routes", name)  # same bounded name rules
                validate_document(self.export, "routes", document)
                if document["name"] != name:
                    raise ValueError("Export name differs from recipe")
                output = (self.export / "exports" / name).resolve()
                if not output.is_relative_to(self.export):
                    raise ValueError("Export escapes source folder")
                if output.exists():
                    raise ValueError("Named export already exists; choose a new name")
                materialize(self.export, document, output)
                save_document(self.export, "routes", name, document)
                return self.json_response(
                    {"path": output.relative_to(self.export).as_posix()}
                )
            path = save_document(
                self.export, body["kind"], body["name"], body["document"]
            )
            self.json_response(
                {"path": path.relative_to(self.export).as_posix(), "name": path.stem}
            )
        except (ValueError, KeyError, OSError) as e:
            self.json_response({"error": str(e)}, 400)


def serve(folder, port):
    folder = Path(folder).resolve()
    if not (folder / "inspect.html").exists():
        raise ValueError("Generate inspect.html before serving the export")
    server = ThreadingHTTPServer(
        ("127.0.0.1", port), partial(EditorHandler, directory=str(folder))
    )
    print(f"Editor: http://localhost:{server.server_port}/inspect.html", flush=True)
    print(f"Saves: {folder}/selections/{{tracks,areas}}/<name>.json", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
