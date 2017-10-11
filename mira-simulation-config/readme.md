Start Miracenter:
miracenter application.xml -p1234
miracenter ~/git/care-o-bot-indigo/src/baker/mira-simulation-config/application.xml -p1234


mirainspect channel echo /navigation/PilotEvent -k127.0.0.1:1234

in MiraCenter - Channels View -> show type of messages





## Install basic MIRA on a separate device
### Install dependencies
- instructions from: http://www.mira-project.org/MIRA-doc/RequirementsLinux.html#RequirementsLinux_Ubuntu
- this uninstalls ROS (because of libogre-1.9-dev)

### Install MIRA
- use the /opt/MIRA/scripts/mira-installer-binary.sh to install the basic framework, e.g. into ~/opt/
- copy license into folder /opt/MIRA-licenses (name and path must match exactly!)
- start mirawizard and create new external project opt/mira-commercial (not necessary to follow the wizard further with creating contents)
- start mirapackage
-- open settings - repositories: there add the mira-commercial entry with all URL etc. (see email)
-- from the package list search for Mapping, Pilot, and Scitos and install the latest packages for each
-- apply changes

### Install Baker Addons
- copy BakerTest into ~/opt/
- update bashrc:
```
########## MIRA
# Mira base installation
export MIRA_PATH=/opt/MIRA
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/MIRA/lib
export PATH=${PATH}:/opt/MIRA/bin
source /opt/MIRA/scripts/mirabash

# Mira commercial installation
export MIRA_PATH=${MIRA_PATH}:/opt/MIRA-commercial:~/opt/BakerTest:/home/rmb/git/care-o-bot-indigo/src
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/MIRA-commercial/lib:~/opt/BakerTest/lib
```

### Start Miracenter
miracenter -k 192.168.x.x:1234
