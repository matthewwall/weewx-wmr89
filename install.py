# installer for wmr89
# Copyright 2018 Matthew Wall
# Distributed under the terms of the GNU Public License (GPLv3)

from setup import ExtensionInstaller

def loader():
    return WMR89Installer()

class WMR89Installer(ExtensionInstaller):
    def __init__(self):
        super(WMR89Installer, self).__init__(
            version="0.3",
            name='wmr89',
            description='Driver for Oregon Scientific WMR89',
            author="Matthew Wall",
            author_email="mwall@users.sourceforge.net",
            files=[('bin/user', ['bin/user/wmr89.py'])]
            )
