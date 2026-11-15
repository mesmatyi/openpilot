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
    self._captured_slots = set()
    self._events = []
    self.stop = Button("BENCH MODE — NO VEHICLE CONTROL\nTAP TO STOP",
                       click_callback=lambda: self.params.put_bool("BenchModeRequested", False, block=True),
                       font_size=32 if gui_app.big_ui() else 22, button_style=ButtonStyle.DANGER)
    gui_app.add_nav_stack_tick(self.capture_input)

  @property
  def visible(self):
    return self.params.get_bool("BenchMode") or self.params.get_bool("BenchModeRequested")

  @property
  def height(self):
    return 90 if gui_app.big_ui() else 70

  def capture_input(self):
    self._events = []
    remaining = []
    for event in gui_app.mouse_events:
      if self.visible and event.left_pressed and 0 <= event.pos.x < gui_app.width and 0 <= event.pos.y < self.height:
        self._captured_slots.add(event.slot)
      if event.slot in self._captured_slots:
        self._events.append(event)
        if event.left_released:
          self._captured_slots.remove(event.slot)
      else:
        remaining.append(event)
    # Reserve the entire gesture before any navigation widget can process it.
    gui_app.mouse_events[:] = remaining

  def render(self):
    if self.visible:
      remaining = gui_app.mouse_events[:]
      gui_app.mouse_events[:] = self._events
      try:
        self.stop.render(rl.Rectangle(0, 0, gui_app.width, self.height))
      finally:
        gui_app.mouse_events[:] = remaining
