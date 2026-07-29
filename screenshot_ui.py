#!/usr/bin/env python3
"""Boots the flask mock server, loads the real UI against it, and saves a
screenshot. Used in CI so UI changes are visually reviewable from the run's
artifacts, without needing real hardware."""

import subprocess
import sys
import time
import urllib.request

from playwright.sync_api import sync_playwright

MOCK_SERVER_URL = "http://127.0.0.1:5000/"
OUTPUT_PATH = "ui_screenshot.png"
STARTUP_TIMEOUT_SECONDS = 10


def wait_for_server(url, timeout_seconds):
    deadline = time.time() + timeout_seconds
    while time.time() < deadline:
        try:
            urllib.request.urlopen(url, timeout=1)
            return True
        except Exception:
            time.sleep(0.3)
    return False


def main():
    server = subprocess.Popen(
        [sys.executable, "flask_mock_server.py"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        if not wait_for_server(MOCK_SERVER_URL, STARTUP_TIMEOUT_SECONDS):
            print("flask_mock_server.py never came up", file=sys.stderr)
            sys.exit(1)

        with sync_playwright() as p:
            browser = p.chromium.launch()
            page = browser.new_page(viewport={"width": 500, "height": 700})
            page.goto(MOCK_SERVER_URL)
            page.wait_for_timeout(1500)  # let the /status poll populate the UI
            page.screenshot(path=OUTPUT_PATH, full_page=True)
            browser.close()

        print(f"Saved {OUTPUT_PATH}")
    finally:
        server.terminate()
        server.wait(timeout=5)


if __name__ == "__main__":
    main()
