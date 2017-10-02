Start Miracenter:
miracenter application.xml -p1234


mirainspect channel echo /navigation/PilotEvent -k127.0.0.1:1234

in MiraCenter - Channels View -> show type of messages





## Install basic MIRA on a separate device
### Install dependencies
- instructions from: http://www.mira-project.org/MIRA-doc/RequirementsLinux.html#RequirementsLinux_Ubuntu
- this uninstalls ROS (because of libogre-1.9-dev

### Install MIRA
- use the /opt/MIRA/scripts/mira-installer-binary.sh to install the basic framework
- copy license into folder /opt/MIRA-licenses (name and path must match exactly!)
- start mirawizard and create new external project opt/mira-commercial (not necessary to follow the wizard further with creating contents)
- start mirapackage
-- open settings - repositories: there add the mira-commercial entry with all URL etc. (see email)
-- from the package list search for Mapping, Pilot, and Scitos and install the latest packages for each
-- apply changes

### Start Miracenter
miracenter -k 192.168.x.x:1234
