
#to build as a make file in windows, change vclib to lib
win32{
TEMPLATE	= vclib
}else{
TEMPLATE = lib
}

LANGUAGE	= C++
CONFIG += qt dylib
QT += qt3support
DESTDIR = lib
MOC_DIR = build
OBJECTS_DIR = build

!exists($(GRASPIT)) {
   error("GRASPIT environment variable not set")
}

!exists($(GRASPIT_PLUGIN_DIR)) {
   error("GRASPIT_PLUGIN_DIR environment variable not set")
}

INCLUDEPATH += $(GRASPIT) $(GRASPIT)/qjson4 $(GRASPIT)/cmdline

DEPENDPATH += $(GRASPIT)/src 

HEADERS += $(GRASPIT)/include/plugin.h \
    graspGenerationPlugin.h

SOURCES += \
    main.cpp \
    graspGenerationPlugin.cpp

