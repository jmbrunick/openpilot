"""Launch helper for NAP scripts (pedal calibration, EPAS flash, maps).

Hardware tools spawn scripts/nap/run_script.py as a detached process. That
runner kills the comma tmux session, takes over the display, and uses
NAPScriptRunning so manager stops pandad for Panda USB.

Map sqlite jobs (Refresh maps, Download US Maps) stay in-process: a
full-screen script-runner widget is pushed so Settings is not drawn
underneath. Exit pops back to Settings.
"""
from __future__ import annotations

import os
import signal
import subprocess
import threading

from openpilot.common.basedir import BASEDIR
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.mici.widgets.script_runner_app import MiciScriptRunnerApp, ScriptState
from openpilot.system.ui.widgets import Widget
from scripts.nap.script_lifecycle import script_reboots_on_exit


class _MapScriptOverlay(Widget):
  """Opaque full-screen map-script UI. Settings under this widget is not shown."""

  def __init__(self, title: str, instructions: str, module: str):
    super().__init__()
    self._module = module
    self._process: subprocess.Popen | None = None
    self._reader_thread: threading.Thread | None = None
    self._prepared = False
    self._app = MiciScriptRunnerApp(
      title=title,
      instructions=instructions,
      on_start=self._on_start,
      on_exit=self._on_exit,
    )

  def show_event(self):
    if not self._prepared:
      self._app.prepare()
      self._prepared = True

  def _render(self, _rect):
    self._app.render_overlay()

  def _on_start(self) -> None:
    try:
      self._process = subprocess.Popen(
        ["python", "-m", self._module],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        cwd=BASEDIR,
        text=True,
        encoding="utf-8",
        bufsize=1,
      )
    except Exception as e:
      self._app.append_output(f"Error starting script: {e}")
      self._app.set_state(ScriptState.ERROR)
      return
    self._app.append_output("Starting script...")
    self._app.set_state(ScriptState.RUNNING)
    self._reader_thread = threading.Thread(target=self._read_output, daemon=True)
    self._reader_thread.start()

  def _read_output(self) -> None:
    try:
      assert self._process is not None
      if self._process.stdout is not None:
        for line in iter(self._process.stdout.readline, ''):
          if line:
            self._app.append_output(line.rstrip())
          if self._process.poll() is not None:
            break
      return_code = self._process.wait()
      if return_code == 0:
        self._app.append_output("")
        self._app.append_output("[Script completed successfully]")
        self._app.set_state(ScriptState.COMPLETED)
      else:
        self._app.append_output("")
        self._app.append_output(f"[Script exited with code {return_code}]")
        self._app.set_state(ScriptState.ERROR)
    except Exception as e:
      self._app.append_output(f"[Error reading output: {e}]")
      self._app.set_state(ScriptState.ERROR)

  def _on_exit(self) -> None:
    if self._process is not None and self._process.poll() is None:
      self._process.send_signal(signal.SIGINT)
      try:
        self._process.wait(timeout=5)
      except subprocess.TimeoutExpired:
        self._process.terminate()
        try:
          self._process.wait(timeout=2)
        except subprocess.TimeoutExpired:
          self._process.kill()
          try:
            self._process.wait(timeout=2)
          except subprocess.TimeoutExpired:
            self._app.append_output("[ERROR] script did not exit")
            self._app.set_state(ScriptState.ERROR)
            return
    gui_app.pop_widget()


def launch_script(title: str, instructions: str, script_module: str) -> None:
  """Show a map job in-process, or spawn run_script.py for hardware tools."""
  if not script_reboots_on_exit(script_module):
    gui_app.push_widget(_MapScriptOverlay(title, instructions, script_module))
    return

  script_path = os.path.join(BASEDIR, "scripts", "nap", "run_script.py")
  log_path = "/tmp/nap_script_runner.log"
  with open(log_path, "w") as log_file:
    subprocess.Popen(
      ["python", script_path, title, script_module, instructions],
      cwd=BASEDIR,
      start_new_session=True,
      stdout=log_file,
      stderr=log_file,
    )
