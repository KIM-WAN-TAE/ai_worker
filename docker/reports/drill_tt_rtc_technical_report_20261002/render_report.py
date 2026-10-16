#!/usr/bin/env python3
"""Update the DOCX table of contents with LibreOffice and export final PDF."""

from __future__ import annotations

import shutil
import subprocess
import tempfile
import time
from pathlib import Path

import uno
from com.sun.star.beans import PropertyValue


HERE = Path(__file__).resolve().parent
DOCX = HERE / "DRILL_TT_RTC_TECHNICAL_REPORT_2026-10-02.docx"
PDF = HERE / "DRILL_TT_RTC_TECHNICAL_REPORT_2026-10-02.pdf"
PORT = 2084


def prop(name: str, value) -> PropertyValue:
    item = PropertyValue()
    item.Name = name
    item.Value = value
    return item


def connect():
    local = uno.getComponentContext()
    resolver = local.ServiceManager.createInstanceWithContext(
        "com.sun.star.bridge.UnoUrlResolver", local
    )
    last_error = None
    for _ in range(50):
        try:
            return resolver.resolve(
                f"uno:socket,host=127.0.0.1,port={PORT};"
                "urp;StarOffice.ComponentContext"
            )
        except Exception as error:  # LibreOffice needs a short startup window.
            last_error = error
            time.sleep(0.1)
    raise RuntimeError(f"cannot connect to LibreOffice: {last_error}")


def render() -> Path:
    profile = Path(tempfile.mkdtemp(prefix="drill_ttrtc_lo_"))
    process = subprocess.Popen(
        [
            "libreoffice",
            "--headless",
            "--nologo",
            "--nodefault",
            "--nofirststartwizard",
            f"-env:UserInstallation={profile.as_uri()}",
            f"--accept=socket,host=127.0.0.1,port={PORT};urp;StarOffice.ComponentContext",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        context = connect()
        manager = context.ServiceManager
        desktop = manager.createInstanceWithContext(
            "com.sun.star.frame.Desktop", context
        )
        document = desktop.loadComponentFromURL(
            uno.systemPathToFileUrl(str(DOCX)),
            "_blank",
            0,
            (prop("Hidden", True), prop("UpdateDocMode", 3)),
        )
        if document is None:
            raise RuntimeError("LibreOffice could not load the report DOCX")
        indexes = document.getDocumentIndexes()
        for index in range(indexes.getCount()):
            indexes.getByIndex(index).update()
        document.store()
        document.storeToURL(
            uno.systemPathToFileUrl(str(PDF)),
            (prop("FilterName", "writer_pdf_Export"), prop("Overwrite", True)),
        )
        document.close(True)
        desktop.terminate()
        process.wait(timeout=10)
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
        shutil.rmtree(profile, ignore_errors=True)
    return PDF


if __name__ == "__main__":
    print(render())

