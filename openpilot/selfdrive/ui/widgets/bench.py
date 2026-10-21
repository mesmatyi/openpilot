import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.manager.bench import BENCH_WARNING, request_start
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.button import Button, ButtonStyle
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog


def confirm_bench_start():
  if not ui_state.is_offroad():
    return

  def confirmed(result):
    request_start(ui_state.params, ui_state.is_offroad(), result == DialogResult.CONFIRM)

  content = "".join(f"<p>{warning}</p><br>" for warning in BENCH_WARNING)
  gui_app.push_widget(ConfirmDialog(content, "Start bench", rich=True, callback=confirmed))


class BenchOverlay:
  """Rendered above every navigation page, including the synthetic onroad UI."""
  def __init__(self):
    self.params = Params()
    self.stop = Button("BENCH MODE — NO VEHICLE CONTROL\nTAP TO STOP",
                       click_callback=lambda: self.params.put_bool("BenchModeRequested", False, block=True),
                       font_size=32 if gui_app.big_ui() else 22, button_style=ButtonStyle.DANGER)

  def render(self):
    if self.params.get_bool("BenchMode") or self.params.get_bool("BenchModeRequested"):
      self.stop.render(rl.Rectangle(0, 0, gui_app.width, 90 if gui_app.big_ui() else 70))
