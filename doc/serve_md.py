#!/usr/bin/env python3
"""Serve Markdown files from a folder in your browser.

- Renders `.md` files to HTML.
- Falls back to showing raw text if the `markdown` package is unavailable.
- Provides a simple directory listing.

Usage:
  python3 serve_md.py --root . --port 8000
  python3 serve_md.py --root . --open

Then open:
  http://127.0.0.1:8000/
"""

from __future__ import annotations

import argparse
import html
import mimetypes
import os
import posixpath
import sys
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


def _try_markdown_to_html(text: str) -> tuple[str, bool]:
    """Return (html, rendered) where rendered indicates real Markdown rendering."""
    try:
        import markdown  # type: ignore

        return (
            markdown.markdown(
                text,
                extensions=[
                    "fenced_code",
                    "tables",
                    "toc",
                    "codehilite",
                ],
                output_format="html5",
            ),
            True,
        )
    except Exception:
        escaped = html.escape(text)
        return (f"<pre>{escaped}</pre>", False)


def _safe_join(root: Path, url_path: str) -> Path | None:
    """Resolve a URL path under root. Returns None if it escapes the root."""
    parts = [p for p in url_path.split("/") if p and p not in (".", "..")]
    candidate = root
    for part in parts:
        candidate = candidate / part
    try:
        candidate_resolved = candidate.resolve(strict=False)
        root_resolved = root.resolve(strict=True)
    except FileNotFoundError:
        # Root missing is a hard error; root is validated earlier.
        return None
    if root_resolved == candidate_resolved or root_resolved in candidate_resolved.parents:
        return candidate
    return None


def _html_page(title: str, body: str) -> bytes:
    css = """
    body { font-family: system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif; margin: 0; }
    header { padding: 12px 16px; border-bottom: 1px solid #ddd; background: #f7f7f7; }
    main { padding: 16px; max-width: 1000px; margin: 0 auto; }
    a { text-decoration: none; }
    a:hover { text-decoration: underline; }
    pre { overflow-x: auto; padding: 12px; background: #f6f8fa; border: 1px solid #ddd; }
    code { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace; }
    .note { color: #555; font-size: 0.95em; }
    """.strip()

    return (
        "<!doctype html>\n"
        "<html><head>\n"
        "<meta charset='utf-8' />\n"
        "<meta name='viewport' content='width=device-width, initial-scale=1' />\n"
        f"<title>{html.escape(title)}</title>\n"
        f"<style>{css}</style>\n"
        "</head><body>\n"
        f"<header><strong>{html.escape(title)}</strong></header>\n"
        f"<main>{body}</main>\n"
        "</body></html>\n"
    ).encode("utf-8")


class DocHandler(BaseHTTPRequestHandler):
    server_version = "serve_md/1.0"

    def do_GET(self) -> None:
        root: Path = self.server.root  # type: ignore[attr-defined]

        parsed = urllib.parse.urlparse(self.path)
        url_path = parsed.path
        url_path = posixpath.normpath(urllib.parse.unquote(url_path))
        if url_path.startswith("../") or url_path == "..":
            self.send_error(400, "Bad path")
            return

        if url_path in ("", "/"):
            target = root
        else:
            target = _safe_join(root, url_path.lstrip("/"))
            if target is None:
                self.send_error(403, "Forbidden")
                return

        if target.is_dir():
            self._send_dir_listing(root, target, url_path)
            return

        if not target.exists():
            self.send_error(404, "Not found")
            return

        if target.suffix.lower() == ".md":
            self._send_markdown(root, target, url_path)
            return

        # For non-md files: serve as static content with a safe content-type.
        ctype, _ = mimetypes.guess_type(str(target))
        ctype = ctype or "application/octet-stream"
        try:
            data = target.read_bytes()
        except OSError as exc:
            self.send_error(500, f"Failed to read file: {exc}")
            return
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_dir_listing(self, root: Path, folder: Path, url_path: str) -> None:
        rel = "/" if folder == root else "/" + str(folder.relative_to(root)).replace(os.sep, "/") + "/"

        entries: list[tuple[str, str]] = []
        try:
            for child in sorted(folder.iterdir(), key=lambda p: (not p.is_dir(), p.name.lower())):
                name = child.name + ("/" if child.is_dir() else "")
                href = urllib.parse.quote(rel + child.name + ("/" if child.is_dir() else ""))
                entries.append((name, href))
        except OSError as exc:
            self.send_error(500, f"Failed to list directory: {exc}")
            return

        parent_link = ""
        if folder != root:
            parent = "/" + str(folder.parent.relative_to(root)).replace(os.sep, "/")
            if not parent.endswith("/"):
                parent += "/"
            parent_link = f"<p><a href='{html.escape(parent)}'>..</a></p>"

        items = "\n".join(
            f"<li><a href='{html.escape(href)}'>{html.escape(name)}</a></li>" for name, href in entries
        )
        body = (
            f"<p class='note'>Browsing: <code>{html.escape(rel)}</code></p>"
            f"{parent_link}"
            f"<ul>{items}</ul>"
        )
        page = _html_page(f"Docs {rel}", body)
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(page)))
        self.end_headers()
        self.wfile.write(page)

    def _send_markdown(self, root: Path, md_file: Path, url_path: str) -> None:
        try:
            text = md_file.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            text = md_file.read_text(encoding="utf-8", errors="replace")
        except OSError as exc:
            self.send_error(500, f"Failed to read file: {exc}")
            return

        rendered_html, did_render = _try_markdown_to_html(text)

        # Header links.
        rel = "/" + str(md_file.relative_to(root)).replace(os.sep, "/")
        parent = "/" + str(md_file.parent.relative_to(root)).replace(os.sep, "/")
        if not parent.endswith("/"):
            parent += "/"

        note = "" if did_render else (
            "<p class='note'>Note: Python package <code>markdown</code> not found; showing raw text. "
            "Install with: <code>python3 -m pip install markdown</code></p>"
        )

        body = (
            f"<p class='note'><a href='{html.escape(parent)}'>Back</a> · "
            f"<code>{html.escape(rel)}</code></p>"
            f"{note}"
            f"{rendered_html}"
        )
        page = _html_page(md_file.name, body)
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(page)))
        self.end_headers()
        self.wfile.write(page)

    def log_message(self, fmt: str, *args: object) -> None:
        # Keep logs concise.
        sys.stderr.write("%s - - [%s] %s\n" % (self.client_address[0], self.log_date_time_string(), fmt % args))


def main() -> int:
    parser = argparse.ArgumentParser(description="Serve Markdown docs in a browser.")
    parser.add_argument("--root", default=".", help="Root folder to serve (default: current directory)")
    parser.add_argument("--host", default="127.0.0.1", help="Bind address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8000, help="Port (default: 8000)")
    parser.add_argument("--open", action="store_true", help="Open the browser after starting")
    args = parser.parse_args()

    root = Path(args.root).expanduser().resolve()
    if not root.exists() or not root.is_dir():
        print(f"Root does not exist or is not a directory: {root}", file=sys.stderr)
        return 2

    httpd = ThreadingHTTPServer((args.host, args.port), DocHandler)
    httpd.root = root  # type: ignore[attr-defined]

    url = f"http://{args.host}:{args.port}/"
    print(f"Serving {root} at {url}")

    if args.open:
        try:
            import webbrowser

            webbrowser.open(url)
        except Exception:
            pass

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
