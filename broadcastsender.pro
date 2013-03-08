HEADERS       = sender.h
SOURCES       = sender.cpp \
                main.cpp
QT           += network

# install
target.path = $$PWD
sources.files = $$SOURCES $$HEADERS $$RESOURCES $$FORMS broadcastsender.pro
sources.path = $$PWD
INSTALLS += target sources

symbian: {
    TARGET.CAPABILITY = NetworkServices
    include($$PWD/../../symbianpkgrules.pri)
}
maemo5: include($$PWD/../../maemo5pkgrules.pri)
contains(MEEGO_EDITION,harmattan): include($$PWD/../../harmattanpkgrules.pri)

CONFIG+=mobility
MOBILITY+=sensors
