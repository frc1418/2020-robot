. venv/bin/activate
pip3 uninstall -y pyfrc
pip3 uninstall -y robotpy-build
pip3 uninstall -y robotpy-installer
pip3 uninstall -y robotpy-wpiutil
pip3 uninstall -y pyntcore
pip3 uninstall -y robotpy-hal
pip3 uninstall -y robotpy-halsim-gui
pip3 uninstall -y wpilib
pip3 uninstall -y robotpy-navx
pip3 uninstall -y robotpy-commands-v1
pip3 uninstall -y robotpy-rev-color
pip3 install -U robotpy-build --no-build-isolation
pip3 install -U robotpy-installer --no-build-isolation
pip3 install -U robotpy-wpiutil --no-build-isolation
pip3 install -U pyntcore --no-build-isolation
pip3 install -U robotpy-hal --no-build-isolation
pip3 install -U wpilib --no-build-isolation
pip3 install -U robotpy-navx --no-build-isolation
pip3 install -U robotpy-commands-v1 --no-build-isolation
pip3 install -U robotpy-rev-color --no-build-isolation
pip3 install -U robotpy-halsim-gui --no-build-isolation
pip3 install -U robotpy-wpilib-utilities
pip3 install -U pynetworktables
pip3 install -U pyfrc
deactivate