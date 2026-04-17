#!/usr/bin/env python3
"""
蓝牙 SPP Agent — 注册 Serial Port Profile，接收手机发来的 '0'-'9' 写入 /tmp/car_mode
依赖: python3-dbus  (sudo apt install python3-dbus)
运行: sudo python3 bt_agent.py
"""
import dbus
import dbus.service
import dbus.mainloop.glib
from gi.repository import GLib
import os, sys

MODE_FILE = "/tmp/car_mode"
SPP_UUID  = "00001101-0000-1000-8000-00805f9b34fb"

class SerialProfile(dbus.service.Object):
    fd = -1

    @dbus.service.method("org.bluez.Profile1", in_signature="oha{sv}", out_signature="")
    def NewConnection(self, path, fd, properties):
        self.fd = fd.take()
        print(f"[BT] connected: {path}")
        GLib.io_add_watch(self.fd, GLib.IO_IN | GLib.IO_HUP, self._on_data)

    @dbus.service.method("org.bluez.Profile1", in_signature="o", out_signature="")
    def RequestDisconnection(self, path):
        print(f"[BT] disconnected: {path}")
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    @dbus.service.method("org.bluez.Profile1", in_signature="", out_signature="")
    def Release(self):
        print("[BT] profile released")

    def _on_data(self, fd, condition):
        if condition & GLib.IO_HUP:
            print("[BT] client hung up")
            os.close(self.fd)
            self.fd = -1
            return False
        data = os.read(fd, 64)
        for b in data:
            c = chr(b)
            if c.isdigit():
                with open(MODE_FILE, "w") as f:
                    f.write(c)
                print(f"[BT] mode -> {c}")
        return True


def main():
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    manager = dbus.Interface(bus.get_object("org.bluez", "/org/bluez"),
                             "org.bluez.ProfileManager1")

    profile_path = "/org/bluez/car_spp"
    profile = SerialProfile(bus, profile_path)

    manager.RegisterProfile(profile_path, SPP_UUID, {
        "Name":    "CarVision SPP",
        "Channel": dbus.UInt16(1),
        "Role":    "server",
        "AutoConnect": dbus.Boolean(True),
    })
    print("[BT] SPP profile registered on channel 1")

    # 确保可配对可发现
    adapter = dbus.Interface(bus.get_object("org.bluez", "/org/bluez/hci0"),
                             "org.freedesktop.DBus.Properties")
    adapter.Set("org.bluez.Adapter1", "Powered",      dbus.Boolean(True))
    adapter.Set("org.bluez.Adapter1", "Discoverable", dbus.Boolean(True))
    adapter.Set("org.bluez.Adapter1", "Pairable",     dbus.Boolean(True))

    GLib.MainLoop().run()

if __name__ == "__main__":
    main()
