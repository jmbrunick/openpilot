import time
from openpilot.common.params import Params
from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.selfdrive.ui.mici.layouts.settings.toggles import TogglesLayoutMici
from openpilot.selfdrive.ui.mici.layouts.settings.network.network_layout import NetworkLayoutMici
from openpilot.selfdrive.ui.mici.layouts.settings.device import DeviceLayoutMici, PairBigButton
from openpilot.selfdrive.ui.mici.layouts.settings.developer import DeveloperLayoutMici
from openpilot.selfdrive.ui.mici.layouts.settings.firehose import FirehoseLayout
from openpilot.selfdrive.ui.mici.layouts.settings.hidden_toggles import HiddenTogglesOverlayMici
from openpilot.selfdrive.ui.mici.layouts.settings.nap import NAPLayoutMici
from openpilot.selfdrive.ui.layouts.settings.triple_tap import MICI_NAP_OPEN_DELAY_S, TripleTapDetector
from openpilot.system.ui.lib.application import gui_app, FontWeight


class SettingsBigButton(BigButton):
  def _get_label_font_size(self):
    return 64


class SettingsLayout(NavScroller):
  def __init__(self):
    super().__init__()
    self._params = Params()

    toggles_panel = TogglesLayoutMici()
    toggles_btn = SettingsBigButton("toggles", "", gui_app.texture("icons_mici/settings.png", 64, 64))
    toggles_btn.set_click_callback(lambda: gui_app.push_widget(toggles_panel))

    network_panel = NetworkLayoutMici()
    network_btn = SettingsBigButton("network", "", gui_app.texture("icons_mici/settings/network/wifi_strength_full.png", 76, 56))
    network_btn.set_click_callback(lambda: gui_app.push_widget(network_panel))

    device_panel = DeviceLayoutMici()
    device_btn = SettingsBigButton("device", "", gui_app.texture("icons_mici/settings/device_icon.png", 72, 58))
    device_btn.set_click_callback(lambda: gui_app.push_widget(device_panel))

    developer_panel = DeveloperLayoutMici()
    developer_btn = SettingsBigButton("developer", "", gui_app.texture("icons_mici/settings/developer_icon.png", 64, 60))
    developer_btn.set_click_callback(lambda: gui_app.push_widget(developer_panel))

    firehose_panel = FirehoseLayout()
    firehose_btn = SettingsBigButton("firehose", "", gui_app.texture("icons_mici/settings/firehose.png", 52, 62))
    firehose_btn.set_click_callback(lambda: gui_app.push_widget(firehose_panel))

    self._nap_panel = NAPLayoutMici()
    nap_btn = SettingsBigButton("nap", "", gui_app.texture("icons_mici/settings/comma_icon.png", 33, 60))
    nap_btn.set_click_callback(self._on_nap_clicked)
    self._nap_triple_tap = TripleTapDetector()
    self._nap_open_at: float | None = None

    self._scroller.add_widgets([
      toggles_btn,
      network_btn,
      device_btn,
      PairBigButton(),
      #BigDialogButton("manual", "", "icons_mici/settings/manual_icon.png", "Check out the mici user\nmanual at comma.ai/setup"),
      nap_btn,
      firehose_btn,
      developer_btn,
    ])

    self._font_medium = gui_app.font(FontWeight.MEDIUM)

  def _on_nap_clicked(self):
    now = time.monotonic()
    if self._nap_triple_tap.tap(now):
      self._nap_open_at = None
      gui_app.push_widget(HiddenTogglesOverlayMici())
      return
    # Single / double tap: open NAP after a short quiet period so a quick
    # triple can still win. See TripleTapDetector / MICI_NAP_OPEN_DELAY_S.
    self._nap_open_at = now + MICI_NAP_OPEN_DELAY_S

  def _update_state(self):
    super()._update_state()
    if self._nap_open_at is not None and time.monotonic() >= self._nap_open_at:
      self._nap_open_at = None
      self._nap_triple_tap.reset()
      if self.enabled:
        gui_app.push_widget(self._nap_panel)
