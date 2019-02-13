At Metralabs:

0. login with baker
1. roslaunch baker_wet_cleaning_application basic_application.launch scitos_node_args:="-k192.168.5.1:1234" brush_cleaning_module_interface_args:="-k192.168.5.1:1234"
2. Start Rviz on robot PC
3. [NOT NECESSARY ANYMORE: this is now already started in basic_application.launch]: rosrun baker_wet_cleaning_application application_wet_cleaning.py
